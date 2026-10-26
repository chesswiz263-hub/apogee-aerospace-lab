"""APOGEE v1.7: experimental 12-error-state joint navigation integrity lab.

All measurements are synthetic in APOGEE_IDEAL_ECI.  Fault gates use nominal
3-DoF chi-square NIS, not calibrated hardware reliability probabilities.
Position/tracker channel isolation means *inconsistency by measurement type*;
it cannot prove sensor failure instead of a common dynamics/model error.
"""
from dataclasses import dataclass
import numpy as np
from scipy.stats import chi2

from .joint_navigation import (JointNavConfig, _initial_filter, predict, update)
from .precision_adcs import PrecisionADCSConfig, simulate_precision_adcs, tracker_visibility, symmetric_psd
from .attitude import q_normalize, q_mul, q_conj, rotate_BI
from .adcs import rotation_vector_quaternion as expq
from .integrated_nav import FRAME
from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE


@dataclass(frozen=True)
class JointIntegrityConfig(JointNavConfig):
    duration_minutes: float = 12.
    samples: int = 48
    navigation_period_s: float = 60.
    tracker_period_s: float = 60.
    telemetry_latency_s: float = 15.
    tracker_latency_s: float = 15.
    burn_start_s: float = 180.
    burn_duration_s: float = 120.
    sensor_correlation: float = .25
    fault_channel: str = 'position'    # none, position, tracker, both
    fault_start_s: float = 360.
    position_fault_m: float = 250.
    tracker_fault_deg: float = 8.
    engine_model_error_pct: float = 0.    # signed estimator thrust-scale error; truth always nominal
    thrust_sigma_pct: float = 5.         # process-noise inflation, NOT scale estimation
    gate_probability: float = .9973
    reject_faults: bool = True
    monte_carlo_runs: int = 3
    tracker_visibility_enabled: bool = False
    seed: int = 29

    @property
    def assumed_thrust_factor(self):
        return 1 + self.engine_model_error_pct / 100

    def validate(self):
        super().validate()
        nums=[self.sensor_correlation,self.fault_start_s,self.position_fault_m,
              self.tracker_fault_deg,self.engine_model_error_pct,self.thrust_sigma_pct,
              self.gate_probability]
        if (not np.isfinite(nums).all() or not (0 <= self.sensor_correlation < .99)
            or self.fault_channel not in ('none','position','tracker','both')
            or not (0 <= self.fault_start_s <= self.duration_minutes*60)
            or not (0 <= self.position_fault_m <= 5000)
            or not (0 <= self.tracker_fault_deg <= 90)
            or not (-30 <= self.engine_model_error_pct <= 30)
            or not (0 <= self.thrust_sigma_pct <= 50)
            or not (.8 <= self.gate_probability < 1.)
            or not (1 <= self.monte_carlo_runs <= 8)
            or self.samples > 96 or self.reference_frame != FRAME):
            raise ValueError('Invalid v1.7 joint integrity configuration or reference frame')


def _ar1(rng, samples, sigma, rho):
    errors=np.empty((samples,3))
    errors[0]=rng.normal(0,sigma,3)
    for j in range(1,samples):
        errors[j]=rho*errors[j-1]+np.sqrt(1-rho*rho)*rng.normal(0,sigma,3)
    return errors


def _create_packets(c, truth, times, seed):
    rng=np.random.default_rng(seed)
    gyro=truth[:,10:13]+np.deg2rad(c.gyro_bias_deg_s)+rng.normal(
        0,np.deg2rad(c.gyro_sigma_deg_s),size=(len(times),3))
    position_errors=_ar1(rng,len(times),c.position_sigma_m/1000,c.sensor_correlation)
    tracker_errors=_ar1(rng,len(times),np.deg2rad(c.tracker_sigma_deg),c.sensor_correlation)
    pcfg=PrecisionADCSConfig(tracker_visibility_enabled=c.tracker_visibility_enabled)
    packets=[];blocked=0
    for i,t in enumerate(times):
        if i==0:continue
        if np.isclose(t/c.navigation_period_s,round(t/c.navigation_period_s),atol=1e-8):
            active=(c.fault_channel in ('position','both') and t>=c.fault_start_s-1e-9)
            err=position_errors[i]+(np.array([c.position_fault_m/1000,0,0]) if active else 0)
            packets.append({'kind':'position','frame':FRAME,'capture_s':float(t),
                            'receipt_s':float(t+c.telemetry_latency_s),
                            'z':(truth[i,:3]+err).tolist(),'fault_injected':bool(active)})
        if c.tracker_enabled and np.isclose(t/c.tracker_period_s,round(t/c.tracker_period_s),atol=1e-8):
            allowed,_,_=tracker_visibility(truth[i,:3],truth[i,6:10],pcfg)
            if c.tracker_visibility_enabled and not allowed:
                blocked+=1;continue
            active=(c.fault_channel in ('tracker','both') and t>=c.fault_start_s-1e-9)
            rv=tracker_errors[i]+(np.array([0,0,np.deg2rad(c.tracker_fault_deg)]) if active else 0)
            q=q_normalize(q_mul(truth[i,6:10],expq(rv)))
            packets.append({'kind':'tracker','frame':FRAME,'capture_s':float(t),
                            'receipt_s':float(t+c.tracker_latency_s),
                            'z':q.tolist(),'fault_injected':bool(active)})
    return gyro,packets,blocked


def _predict_integrity(s,P,a,b,gyro,c):
    s,P=predict(s,P,a,b,gyro,c)
    mid=(a+b)*.5
    if c.burn_start_s <= mid < c.burn_start_s+c.burn_duration_s and c.thrust_sigma_pct>0:
        # White acceleration scale approximation for process error in thrust.
        # Inflates orbital covariance in estimated body-axis thrust direction.
        dt=b-a
        m=max(600.,750.-850.*(mid-c.burn_start_s)/(315.*9.80665))
        sigma_a=850./m/1000.*c.thrust_sigma_pct/100.
        u=rotate_BI(s[6:10],[1.,0.,0.]);U=np.outer(u,u)*sigma_a**2
        # A bounded constant-acceleration model error over this sample interval:
        # delta_r=0.5 a dt², delta_v=a dt, including proper cross covariance.
        G=np.zeros((12,3));G[:3,:]=.5*dt*dt*np.eye(3);G[3:6,:]=dt*np.eye(3)
        P=symmetric_psd(P+G@U@G.T,1e-23)
    return s,P


def score_packet(s,P,p,c):
    """Return chi-square NIS; never use simulated truth to score a packet."""
    if p['frame']!=FRAME:
        raise ValueError('Incompatible navigation measurement frame')
    H=np.zeros((3,12))
    if p['kind']=='position':
        H[:,:3]=np.eye(3)
        y=np.asarray(p['z'])-s[:3]
        R=np.eye(3)*(c.position_sigma_m/1000)**2
    elif p['kind']=='tracker':
        from .adcs import quaternion_rotation_vector as logq
        H[:,6:9]=np.eye(3)
        y=logq(q_mul(q_conj(s[6:10]),q_normalize(p['z'])))
        R=np.eye(3)*np.deg2rad(c.tracker_sigma_deg)**2
    else:raise ValueError('Unknown measurement channel')
    S=H@P@H.T+R
    return float(y@np.linalg.solve(S,y))


def _replay_integrity(s0,P0,times,gyro,packets,now,c):
    permitted=sorted((p for p in packets if p['receipt_s']<=now+1e-9),
                     key=lambda p:(p['capture_s'],p['kind']))
    s=s0.copy();P=P0.copy();at=0;events=[]
    gate=float(chi2.ppf(c.gate_probability,3))
    for i in range(len(times)-1):
        if times[i]>=now-1e-9:break
        stamp=float(times[i+1]);ending=min(stamp,now)
        s,P=_predict_integrity(s,P,float(times[i]),ending,.5*(gyro[i]+gyro[i+1]),c)
        if ending < stamp-1e-9:break
        while at<len(permitted) and permitted[at]['capture_s']<=stamp+1e-9:
            p=permitted[at];at+=1
            if not np.isclose(p['capture_s'],stamp,atol=1e-7):
                raise ValueError('Navigation sensor capture must match simulation grid')
            score=score_packet(s,P,p,c)
            alarm=score>gate
            accepted=not(alarm and c.reject_faults)
            if accepted:s,P,_=update(s,P,p,c)
            events.append({'capture_s':p['capture_s'],'receipt_s':p['receipt_s'],
                'kind':p['kind'],'nis':score,'alarm':bool(alarm),
                'accepted':bool(accepted),'fault_injected':p['fault_injected']})
    return s,P,events


def _single(c,truth,times,seed,series=False):
    gyro,packets,blocked=_create_packets(c,truth,times,seed)
    initial,P0=_initial_filter(c,truth)
    events=[];pe=[];uncertainty=[];ang=[];record=[];min_eigen=np.inf
    snapshots=list(range(len(times))) if series else [len(times)-1]
    from .adcs import quaternion_rotation_vector as logq
    for i in snapshots:
        s,P,events=_replay_integrity(initial,P0,times,gyro,packets,float(times[i]),c)
        pe.append(float(np.linalg.norm((s[:3]-truth[i,:3])*1000)))
        uncertainty.append(float(np.sqrt(np.trace(P[:3,:3]))*1000))
        ang.append(float(np.rad2deg(np.linalg.norm(logq(q_mul(q_conj(s[6:10]),truth[i,6:10]))))))
        min_eigen=min(min_eigen,float(np.linalg.eigvalsh(P).min()))
        record.append(int(sum(e['alarm'] for e in events)))
    arrived=[e for e in events]
    alarms=[e for e in arrived if e['alarm']]
    fault_events=[e for e in arrived if e['fault_injected']]
    faulty_alarms=[e for e in fault_events if e['alarm']]
    pos=sum(e['kind']=='position' for e in alarms)
    tracker=sum(e['kind']=='tracker' for e in alarms)
    # Channel identification is statistical classification, not definitive root cause.
    classification=('multiple_channels_or_model_mismatch' if pos and tracker else
                    'position_channel_inconsistent' if pos else
                    'tracker_channel_inconsistent' if tracker else 'no_alarms')
    result={
        'seed':int(seed),'final_position_error_m':pe[-1],
        'final_position_1sigma_rss_m':uncertainty[-1],
        'final_attitude_error_deg':ang[-1],
        'final_error_over_sigma_rss':pe[-1]/max(uncertainty[-1],1e-12),
        'alarms_by_channel':{'position':pos,'tracker':tracker},
        'classification':classification,'total_alarms':len(alarms),
        'rejected_packets':sum(not e['accepted'] for e in arrived),
        'received_packets':len(arrived),'generated_packets':len(packets),
        'fault_measurements_received':len(fault_events),
        'fault_alarms':len(faulty_alarms),
        'first_fault_alarm_s':faulty_alarms[0]['capture_s'] if faulty_alarms else None,
        'pre_fault_alarms':sum(e['alarm'] and not e['fault_injected'] for e in arrived),
        'min_covariance_eigenvalue':min_eigen,
        'tracker_visibility_blocked':blocked,
        'gate_chi2_3dof':float(chi2.ppf(c.gate_probability,3))}
    if series:
        result.update({'position_error_m':pe,'position_1sigma_rss_m':uncertainty,
                       'attitude_error_deg':ang,'cumulative_alarms':record,
                       'packet_events':arrived,'time_s':times.tolist()})
    return result


def simulate_joint_integrity(c:JointIntegrityConfig):
    c.validate()
    acfg=PrecisionADCSConfig(altitude_km=c.altitude_km,inclination_deg=c.inclination_deg,
        duration_minutes=c.duration_minutes,samples=c.samples,seed=c.seed,
        burn_start_s=c.burn_start_s,burn_duration_s=c.burn_duration_s,
        j2_enabled=c.j2_enabled,tracker_enabled=c.tracker_enabled,
        tracker_visibility_enabled=c.tracker_visibility_enabled,gyro_bias_deg_s=c.gyro_bias_deg_s)
    out=simulate_precision_adcs(acfg)
    times=np.asarray(out['trajectory']['time_s'],float)
    truth=np.c_[out['trajectory']['position_km'],out['trajectory']['velocity_km_s'],
                out['attitude']['quaternion_BI_wxyz'],out['attitude']['angular_velocity_body_rad_s']]
    runs=[_single(c,truth,times,c.seed+7100+i,series=(i==0)) for i in range(c.monte_carlo_runs)]
    primary=runs[0]
    vals=np.asarray([r['final_position_error_m'] for r in runs])
    all_faults=sum(r['fault_measurements_received'] for r in runs)
    fault_detections=sum(r['fault_alarms'] for r in runs)
    out['joint_integrity']={**primary,'monte_carlo':{
        'runs':len(runs),'median_final_position_error_m':float(np.median(vals)),
        'worst_final_position_error_m':float(np.max(vals)),
        'injected_measurement_alarm_fraction':float(fault_detections/all_faults) if all_faults else None,
        'seeds':[r['seed'] for r in runs],
        'per_run':[{'seed':r['seed'],'final_position_error_m':r['final_position_error_m'],
                   'total_alarms':r['total_alarms'],'classification':r['classification']} for r in runs]}}
    out['name']='Joint navigation · channel-wise statistical integrity checks'
    out['input']=c.__dict__.copy()
    out['model']['name']='v1.7 joint 12-error-state covariance with channel NIS gates'
    out['model']['reference_frame']=FRAME
    out['model']['assumptions'] += [
        'Synthetic 3-axis Cartesian position and quaternion tracker fixes; nominal 3-DoF NIS gates',
        'Finite-duration true thrust in 17-state ADCS truth, optionally mismatched in estimator',
        'Estimator thrust scale differs from truth by configured percentage; scale is not estimated',
        'Approximate rank-one thrust uncertainty is injected into the shared joint covariance',
        'Time-correlated AR1 noise violates the white-innovation statistical gate assumptions',
        'Packets assimilated only when received; replay at original capture epochs',
        'Channel anomaly attribution is not a verified hardware fault diagnosis']
    out['warnings'] += [
        'Educational fault classification only: position anomalies may reflect engine/model error, not a bad sensor.',
        'Nominal chi-square exceedance probabilities do not hold under correlated errors or model mismatch.',
        'Truth comes from the existing 17-state ADCS integrator; this is not independent flight data.']
    return out
