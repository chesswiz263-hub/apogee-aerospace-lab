"""APOGEE v1.8 — experimental coupled navigation fault response.

State error: [dr(3), dv(3), dtheta(3), db(3), dthrust_scale(1)].
Nominal state: [r(3), v(3), q(4), gyro bias(3)] + scalar scale.
Sequential, time-tagged, causal replay of synthetic position and star-tracker
packets. The scale is identifiable only through powered-flight measurements.
This is NOT a fault-tolerant flight navigator or an independently calibrated filter.
"""
from dataclasses import dataclass, replace
from math import sqrt
import numpy as np
from scipy.linalg import expm
from scipy.stats import chi2

from .joint_integrity import JointIntegrityConfig, _create_packets
from .joint_navigation import _initial_filter, predict, dynamics_jacobian
from .precision_adcs import PrecisionADCSConfig, simulate_precision_adcs, symmetric_psd, cross_matrix
from .attitude import q_normalize, q_mul, q_conj, rotate_BI, G0
from .adcs import rotation_vector_quaternion as expq, quaternion_rotation_vector as logq
from .integrated_nav import FRAME


@dataclass(frozen=True)
class AdaptiveNavConfig(JointIntegrityConfig):
    fault_channel: str = 'none'
    monte_carlo_runs: int = 1
    actual_thrust_error_pct: float = 5.
    estimate_thrust_scale: bool = True
    initial_thrust_sigma_pct: float = 15.
    process_thrust_sigma_pct_sqrt_s: float = 0.005
    measurement_response: str = 'gate'     # monitor | gate | quarantine
    quarantine_after_alarms: int = 2
    quarantine_duration_s: float = 120.
    adaptive_noise: bool = True
    adaptive_gain: float = .15

    def validate(self):
        super().validate()
        nums=(self.actual_thrust_error_pct,self.initial_thrust_sigma_pct,
              self.process_thrust_sigma_pct_sqrt_s,self.quarantine_duration_s,self.adaptive_gain)
        if (not np.isfinite(nums).all() or abs(self.actual_thrust_error_pct)>30
            or not 0<=self.initial_thrust_sigma_pct<=30
            or not 0<=self.process_thrust_sigma_pct_sqrt_s<=.5
            or self.measurement_response not in ('monitor','gate','quarantine')
            or not 1<=self.quarantine_after_alarms<=10
            or not 0<=self.quarantine_duration_s<=1200
            or not 0<=self.adaptive_gain<=1
            or self.monte_carlo_runs>4):
            raise ValueError('Invalid v1.8 adaptive navigation configuration')


def _config_for_scale(c,scale):
    # estimator scale, never the hidden truth thrust factor
    return replace(c,engine_model_error_pct=100*(float(scale)-1))


def _initial_adaptive(c,truth):
    nominal,P12=_initial_filter(c,truth)
    P=np.zeros((13,13))
    P[:12,:12]=P12
    if c.estimate_thrust_scale:
        P[12,12]=(c.initial_thrust_sigma_pct/100)**2
    else:
        P[12,12]=1e-24
    return nominal, P, 1+c.engine_model_error_pct/100


def predict_adaptive(s,P,scale,t0,t1,gyro,c):
    """Coupled covariance linearization and inherited RK4 nominal-state solver.

    Uses the same approximate first-order error-state dynamics as v1.5, with
    the 13th column coupling uncertain scalar engine scale to acceleration.
    """
    cf=_config_for_scale(c,scale)
    snew,Q12=predict(s,np.zeros((12,12)),t0,t1,gyro,cf)
    dt=t1-t0
    if dt<=0:return s.copy(),P.copy(),float(scale)
    mid=(t0+t1)*.5
    F=np.zeros((13,13))
    F[:12,:12]=dynamics_jacobian(s[:6],s[6:10],s[10:13],gyro,mid,cf)
    firing=(c.burn_start_s<=mid<c.burn_start_s+c.burn_duration_s)
    if firing and c.estimate_thrust_scale:
        mass=max(600.,750.-850./(315.*G0)*(mid-c.burn_start_s))
        F[3:6,12]=850./mass/1000*rotate_BI(s[6:10],[1,0,0])
    phi=expm(F*dt)
    Q=np.zeros((13,13));Q[:12,:12]=Q12
    if c.estimate_thrust_scale:
        Q[12,12]=(c.process_thrust_sigma_pct_sqrt_s/100)**2*dt
    else:
        Q[12,12]=1e-24
    Pnew=symmetric_psd(phi@P@phi.T+Q,1e-23)
    return snew,Pnew,float(scale)


def innovation(s,P,p,c,noise_multiplier=1.):
    if p['frame']!=FRAME:raise ValueError('Incompatible sensor coordinate frame')
    H=np.zeros((3,13))
    if p['kind']=='position':
        H[:,:3]=np.eye(3)
        y=np.asarray(p['z'])-s[:3]
        R=np.eye(3)*(c.position_sigma_m/1000)**2*noise_multiplier
    elif p['kind']=='tracker':
        H[:,6:9]=np.eye(3)
        y=logq(q_mul(q_conj(s[6:10]),q_normalize(p['z'])))
        R=np.eye(3)*np.deg2rad(c.tracker_sigma_deg)**2*noise_multiplier
    else:raise ValueError('Unknown sensor channel')
    S=H@P@H.T+R
    nis=float(y@np.linalg.solve(S,y))
    return y,H,R,S,nis


def update_adaptive(s,P,scale,p,c,noise_multiplier=1.):
    y,H,R,S,nis=innovation(s,P,p,c,noise_multiplier)
    K=np.linalg.solve(S,(P@H.T).T).T
    if not c.estimate_thrust_scale: K[12,:]=0
    dx=K@y
    snew=s.copy();snew[:6]+=dx[:6]
    snew[6:10]=q_normalize(q_mul(s[6:10],expq(dx[6:9])))
    snew[10:13]+=dx[9:12]
    scale_new=float(np.clip(scale+dx[12],.70,1.30))
    I=np.eye(13); A=I-K@H
    Pnew=A@P@A.T+K@R@K.T
    reset=np.eye(13);reset[6:9,6:9]-=.5*cross_matrix(dx[6:9])
    Pnew=symmetric_psd(reset@Pnew@reset.T,1e-23)
    return snew,Pnew,scale_new,nis


def replay_adaptive(s0,P0,scale0,times,gyro,packets,now,c):
    """Only assimilate observations received by now, at their capture epochs."""
    permitted=sorted((p for p in packets if p['receipt_s']<=now+1e-9),
                     key=lambda p:(p['capture_s'],p['kind']))
    s=s0.copy();P=P0.copy();scale=float(scale0)
    counts={'position':0,'tracker':0}
    blocked_until={'position':-1.,'tracker':-1.}
    Rscale={'position':1.,'tracker':1.}
    events=[]; at=0
    gate=float(chi2.ppf(c.gate_probability,3))
    for i in range(len(times)-1):
        if times[i]>=now-1e-9:break
        end=float(min(now,times[i+1]))
        s,P,scale=predict_adaptive(s,P,scale,float(times[i]),end,
                                   .5*(gyro[i]+gyro[i+1]),c)
        if end<float(times[i+1])-1e-9:break
        stamp=float(times[i+1])
        while at<len(permitted) and permitted[at]['capture_s']<=stamp+1e-9:
            p=permitted[at];at+=1
            if not np.isclose(p['capture_s'],stamp,atol=1e-7):
                raise ValueError('Packet capture outside navigation grid')
            kind=p['kind']
            if c.measurement_response=='quarantine' and stamp<blocked_until[kind]:
                events.append({'capture_s':stamp,'receipt_s':p['receipt_s'],'kind':kind,
                               'nis':None,'alarm':False,'accepted':False,
                               'action':'quarantined','fault_injected':p['fault_injected'],
                               'thrust_scale':scale})
                continue
            _,_,_,_,nis=innovation(s,P,p,c,Rscale[kind])
            alarm=nis>gate
            counts[kind]=counts[kind]+1 if alarm else 0
            action='accepted'
            if alarm and c.measurement_response in ('gate','quarantine'):
                action='rejected'
                if (c.measurement_response=='quarantine' and
                    counts[kind]>=c.quarantine_after_alarms):
                    blocked_until[kind]=stamp+c.quarantine_duration_s
                    counts[kind]=0
                    action='quarantine_started'
            if action=='accepted':
                s,P,scale,_=update_adaptive(s,P,scale,p,c,Rscale[kind])
                # This EWMA is a *heuristic*, not proven covariance matching.
                if c.adaptive_noise:
                    target=np.clip(nis/3,1.,25.)
                    Rscale[kind]=float(np.clip((1-c.adaptive_gain)*Rscale[kind]+
                                                        c.adaptive_gain*target,1.,25.))
            events.append({'capture_s':stamp,'receipt_s':p['receipt_s'],'kind':kind,
                           'nis':nis,'alarm':bool(alarm),'accepted':action=='accepted',
                           'action':action,'fault_injected':p['fault_injected'],
                           'thrust_scale':scale,'R_multiplier':Rscale[kind]})
    return s,P,scale,events,Rscale


def _single(c,truth,times,seed,series=False):
    gyro,packets,blocked=_create_packets(c,truth,times,seed)
    s0,P0,scale0=_initial_adaptive(c,truth)
    position=[]; sigma=[]; estimation=[]; alarms=[]; scale_history=[]
    snapshots=range(len(times)) if series else [len(times)-1]
    for i in snapshots:
        s,P,scale,events,Rscale=replay_adaptive(s0,P0,scale0,times,gyro,packets,float(times[i]),c)
        position.append(float(np.linalg.norm((s[:3]-truth[i,:3])*1000)))
        sigma.append(float(sqrt(max(0,np.trace(P[:3,:3])))*1000))
        estimation.append(float(np.rad2deg(np.linalg.norm(logq(q_mul(q_conj(s[6:10]),truth[i,6:10]))))))
        scale_history.append(float(scale))
        alarms.append(sum(bool(e['alarm']) for e in events))
    flags=[e for e in events if e['alarm']]
    result={'seed':int(seed),'final_position_error_m':position[-1],
        'final_position_1sigma_rss_m':sigma[-1],
        'final_attitude_error_deg':estimation[-1],
        'actual_thrust_scale':1+c.actual_thrust_error_pct/100,
        'estimated_thrust_scale':float(scale),
        'thrust_scale_1sigma':float(sqrt(max(0,P[12,12]))),
        'total_alarms':len(flags),
        'alarms_by_channel':{k:sum(e['kind']==k for e in flags) for k in ('position','tracker')},
        'rejected_packets':sum(e['action']!='accepted' for e in events),
        'quarantined_packets':sum(e['action']=='quarantined' for e in events),
        'received_packets':len(events),'generated_packets':len(packets),
        'final_noise_multipliers':Rscale,
        'min_covariance_eigenvalue':float(np.linalg.eigvalsh(P).min()),
        'tracker_visibility_blocked':blocked,
        'first_alarm_s':flags[0]['capture_s'] if flags else None,
        'fault_alarms':sum(e['alarm'] and e['fault_injected'] for e in events)}
    if series:
        result.update({'time_s':times.tolist(),'position_error_m':position,
            'position_1sigma_rss_m':sigma,'attitude_error_deg':estimation,
            'estimated_thrust_scale_history':scale_history,
            'cumulative_alarms':alarms,'packet_events':events})
    return result


def simulate_adaptive_navigation(c:AdaptiveNavConfig):
    c.validate()
    truthcfg=PrecisionADCSConfig(altitude_km=c.altitude_km,inclination_deg=c.inclination_deg,
        duration_minutes=c.duration_minutes,samples=c.samples,seed=c.seed,
        burn_start_s=c.burn_start_s,burn_duration_s=c.burn_duration_s,
        thrust_n=850*(1+c.actual_thrust_error_pct/100),j2_enabled=c.j2_enabled,
        tracker_enabled=c.tracker_enabled,
        tracker_visibility_enabled=c.tracker_visibility_enabled,
        gyro_bias_deg_s=c.gyro_bias_deg_s)
    output=simulate_precision_adcs(truthcfg)
    times=np.asarray(output['trajectory']['time_s'],float)
    truth=np.c_[output['trajectory']['position_km'],output['trajectory']['velocity_km_s'],
               output['attitude']['quaternion_BI_wxyz'],output['attitude']['angular_velocity_body_rad_s']]
    runs=[_single(c,truth,times,c.seed+8100+k,series=(k==0)) for k in range(c.monte_carlo_runs)]
    primary=runs[0]; errors=np.asarray([r['final_position_error_m'] for r in runs])
    output['adaptive_navigation']={**primary,'monte_carlo':{
        'runs':len(runs),'median_final_position_error_m':float(np.median(errors)),
        'worst_final_position_error_m':float(np.max(errors)),
        'seeds':[r['seed'] for r in runs],
        'per_run':[{'seed':r['seed'],'final_position_error_m':r['final_position_error_m'],
                    'estimated_thrust_scale':r['estimated_thrust_scale'],
                    'total_alarms':r['total_alarms']} for r in runs]}}
    output['name']='Adaptive navigation · experimental thrust estimation and sensor recovery'
    output['input']=c.__dict__.copy()
    output['model']['name']='v1.8 nominal 13-error-state EKF with thrust-scale parameter'
    output['model']['reference_frame']=FRAME
    output['model']['assumptions'] += [
        'Synthetic Cartesian fixes and star tracker updates, matched at capture epoch',
        'One scalar thrust scale is an estimated parameter; engine thrust truth is configurable',
        'First-order thrust-scale sensitivity and heuristic process noise, not calibrated',
        'Sensor noise EWMA is heuristic and can mask slowly developing faults',
        'Per-channel quarantine is a temporary software policy, not certified FDIR',
        'AR(1) sensor error violates nominal independent-innovation chi-square assumptions']
    output['warnings'] += [
        'Innovation alarms do not identify root cause. Model errors can resemble sensor faults.',
        'Parameter estimates are poorly observable outside a burn; do not infer calibration precision.',
        'Replaying all available observations each time step is educational and computationally costly.',
        'Truth, measurements, and sensor geometry are simulated, not validated spacecraft data.']
    return output
