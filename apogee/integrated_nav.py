"""APOGEE v1.3: causal, time-tagged translational navigation alongside the v1.2 MEKF.

This is a LOOSELY COUPLED estimator: attitude estimates from the existing MEKF
provide commanded thrust axes to an independent 6-state Cartesian EKF. No
joint position/attitude cross-covariance is represented. Packets are immutable
and are only assimilated once their receipt time is reached. On receipt the
filter is reconstructed chronologically using ONLY received packets, re-playing
old fixes at their capture epochs (bounded offline fixed-lag demonstration).
All vectors use APOGEE's idealized common inertial axes; NOT ICRF or ITRF.
"""
from dataclasses import dataclass, replace
from math import sqrt
import numpy as np
from scipy.linalg import expm
from .constants import EARTH_MU_KM3_S2 as MU
from .perturbations import j2_acceleration
from .physics import state_from_elements
from .attitude import rotate_BI
from .powered_covariance import powered_attitude_discrete_cov
from .precision_adcs import PrecisionADCSConfig, simulate_precision_adcs, symmetric_psd
from .robust_guidance import stations, ground_observation, process_noise_covariance

FRAME = 'APOGEE_IDEAL_ECI'

@dataclass(frozen=True)
class IntegratedNavConfig:
    altitude_km: float = 400.
    inclination_deg: float = 51.6
    duration_minutes: float = 18.
    samples: int = 72
    navigation_period_s: float = 60.
    telemetry_latency_s: float = 45.
    position_sigma_m: float = 30.
    initial_position_offset_m: float = 600.
    initial_velocity_offset_m_s: float = .3
    measurement_mode: str = 'cartesian'  # cartesian or ground_range_rate
    tracker_enabled: bool = True
    gyro_bias_deg_s: float = .015
    burn_start_s: float = 300.
    burn_duration_s: float = 0.
    j2_enabled: bool = True
    seed: int = 27
    reference_frame: str = FRAME
    propagate_attitude_covariance: bool = False

    def validate(self):
        vals=[getattr(self,k) for k in ('altitude_km','inclination_deg','duration_minutes','navigation_period_s',
              'telemetry_latency_s','position_sigma_m','initial_position_offset_m','initial_velocity_offset_m_s',
              'gyro_bias_deg_s','burn_start_s','burn_duration_s')]
        if not all(np.isfinite(vals)) or not (200<=self.altitude_km<=5000 and 0<=self.inclination_deg<=180
              and 2<=self.duration_minutes<=60 and self.samples in range(12,121)
              and 10<=self.navigation_period_s<=600 and 0<=self.telemetry_latency_s<=600
              and .01<=self.position_sigma_m<=1000 and 0<=self.initial_position_offset_m<=10000
              and 0<=self.initial_velocity_offset_m_s<=5 and 0<=self.gyro_bias_deg_s<=2
              and 0<=self.burn_start_s and 0<=self.burn_duration_s
              and self.burn_start_s+self.burn_duration_s<=self.duration_minutes*60+1e-9
              and self.measurement_mode in ('cartesian','ground_range_rate')
              and isinstance(self.seed,int) and 0<=self.seed<=2**31-1
              and isinstance(self.j2_enabled,bool) and isinstance(self.tracker_enabled,bool)
              and isinstance(self.propagate_attitude_covariance,bool)):
            raise ValueError('Integrated navigation inputs out of bounds')
        if self.reference_frame!=FRAME:
            raise ValueError('Frame mismatch: only APOGEE_IDEAL_ECI is supported; do not mix with TEME/ICRF/ITRF')
        # Evaluation at grid boundaries only: avoid interpreting rounded timestamps as exact.
        dt=self.duration_minutes*60/self.samples
        if not np.isclose(self.navigation_period_s/dt,round(self.navigation_period_s/dt),atol=1e-9):
            raise ValueError('Navigation period must be an integer multiple of output step')


def make_packet(capture_s, latency_s, observation, frame=FRAME, kind='cartesian', station=None):
    if frame!=FRAME: raise ValueError('Incompatible measurement frame')
    if not np.isfinite([capture_s,latency_s]).all() or capture_s<0 or latency_s<0:
        raise ValueError('Invalid packet timestamps')
    obs=np.asarray(observation,float)
    if obs.shape != ((3,) if kind=='cartesian' else (2,)) or not np.isfinite(obs).all():
        raise ValueError('Invalid sensor data')
    if kind not in ('cartesian','ground_range_rate'):
        raise ValueError('Unknown sensor packet')
    return {'capture_s':float(capture_s),'receipt_s':float(capture_s+latency_s),
            'frame':frame,'kind':kind,'value':obs.tolist(),'station':station}


def _estimated_force(t,x,cfg,times,qs):
    r=x[:3];rad=np.linalg.norm(r)
    grav=-MU*r/rad**3
    if cfg.j2_enabled:grav+=j2_acceleration(r)
    if cfg.burn_start_s<=t<cfg.burn_start_s+cfg.burn_duration_s:
        # Zero-order held ATTITUDE ESTIMATE, never hidden attitude truth.
        idx=min(len(qs)-1,max(0,int(np.searchsorted(times,t,side='right')-1)))
        q=qs[idx]
        mass=max(600.,750.-850.*(t-cfg.burn_start_s)/(315.*9.80665))
        grav+=850./(mass*1000.)*rotate_BI(q,[1,0,0])
    return np.r_[x[3:],grav]


def _jacobian_r(r,with_j2):
    rad=np.linalg.norm(r)
    mat=MU*(3.*np.outer(r,r)/rad**5-np.eye(3)/rad**3)
    if with_j2:
        # Centered derivative of the perturbation only. J2 is small compared
        # with point-mass gravity, and omitting it biases covariance propagation.
        eps=.01
        for i in range(3):
            d=np.zeros(3);d[i]=eps
            mat[:,i]+=(j2_acceleration(r+d)-j2_acceleration(r-d))/(2.*eps)
    return mat


def _predict_interval(x,P,a,b,cfg,times,qs,attitude_std=None):
    if b<a-1e-9:raise ValueError('Nonmonotonic estimator time')
    if b-a<1e-10:return x,P
    # Partition exactly at attitude grid boundaries and engine on/off events.
    boundaries=[float(z) for z in times if a+1e-9<z<b-1e-9]
    boundaries += [z for z in (cfg.burn_start_s,cfg.burn_start_s+cfg.burn_duration_s) if a+1e-9<z<b-1e-9]
    for t0,t1 in zip([a]+sorted(set(boundaries)),sorted(set(boundaries))+[b]):
        dt=t1-t0
        def f(t,s):return _estimated_force(t,s,cfg,times,qs)
        k1=f(t0,x);k2=f(t0+dt/2,x+dt*k1/2)
        k3=f(t0+dt/2,x+dt*k2/2);k4=f(t1-1e-10,x+dt*k3)
        x=x+dt*(k1+2*k2+2*k3+k4)/6
        A=np.zeros((6,6));A[:3,3:]=np.eye(3);A[3:,:3]=_jacobian_r(x[:3],cfg.j2_enabled)
        Phi=expm(A*dt)
        P=Phi@P@Phi.T+process_noise_covariance(dt,.002)
        if cfg.propagate_attitude_covariance and attitude_std is not None and \
              cfg.burn_start_s<=(t0+t1)/2<cfg.burn_start_s+cfg.burn_duration_s:
            i=min(len(qs)-1,max(0,int(np.searchsorted(times,(t0+t1)/2,side='right')-1)))
            thrust_axis=rotate_BI(qs[i],[1,0,0])
            mass=max(600.,750.-850.*(max(0.,(t0+t1)/2-cfg.burn_start_s))/(315.*9.80665))
            accel=(850./mass/1000.)*thrust_axis
            P+=powered_attitude_discrete_cov(accel,dt,attitude_std[i])
        P=symmetric_psd(P,1e-19)
    return x,P


def _update_packet(x,P,packet,sigma_m):
    if packet['frame']!=FRAME:raise ValueError('Incompatible measurement frame')
    if packet['kind']=='cartesian':
        z=np.asarray(packet['value']);pred=x[:3]
        H=np.zeros((3,6));H[:,:3]=np.eye(3)
        R=np.eye(3)*(sigma_m/1000.)**2
    else:
        st=packet['station']
        out=ground_observation(x,*st,packet['capture_s'],min_elevation_deg=-90)
        pred,H,_=out
        z=np.asarray(packet['value'])
        R=np.diag([(sigma_m/1000.)**2,(.03/1000.)**2])
    innov=z-pred
    S=H@P@H.T+R
    K=np.linalg.solve(S,(P@H.T).T).T
    x=x+K@innov
    IKH=np.eye(6)-K@H
    P=symmetric_psd(IKH@P@IKH.T+K@R@K.T,1e-19)
    return x,P,float(innov@np.linalg.solve(S,innov))


def reconstruct_estimate(initial_x,initial_P,packets,now_s,cfg,times,estimated_q,attitude_std=None):
    """Replay only packets received *by now*, at their original capture epochs."""
    x=np.array(initial_x,float,copy=True);P=np.array(initial_P,float,copy=True)
    processed=sorted((p for p in packets if p['receipt_s']<=now_s+1e-8),
                     key=lambda p:(p['capture_s'],p['receipt_s']))
    cursor=0.;nis=[]
    for p in processed:
        if p['capture_s']>now_s+1e-8:raise ValueError('Future observation')
        x,P=_predict_interval(x,P,cursor,p['capture_s'],cfg,times,estimated_q,attitude_std)
        cursor=p['capture_s']
        x,P,stat=_update_packet(x,P,p,cfg.position_sigma_m)
        nis.append(stat)
    x,P=_predict_interval(x,P,cursor,now_s,cfg,times,estimated_q,attitude_std)
    return x,P,nis,len(processed)


def simulate_integrated_navigation(cfg:IntegratedNavConfig):
    cfg.validate()
    acfg=PrecisionADCSConfig(altitude_km=cfg.altitude_km,inclination_deg=cfg.inclination_deg,
         duration_minutes=cfg.duration_minutes,samples=cfg.samples,tracker_enabled=cfg.tracker_enabled,
         gyro_bias_deg_s=cfg.gyro_bias_deg_s,burn_start_s=cfg.burn_start_s,
         burn_duration_s=cfg.burn_duration_s,j2_enabled=cfg.j2_enabled,seed=cfg.seed)
    out=simulate_precision_adcs(acfg)
    t=np.asarray(out['trajectory']['time_s'],float)
    truth=np.c_[out['trajectory']['position_km'],out['trajectory']['velocity_km_s']]
    q=np.asarray(out['precision_adcs']['estimated_quaternion_BI_wxyz'],float)
    attitude_std=np.asarray(out['precision_adcs']['estimated_attitude_3axis_sigma_deg'],float)
    rng=np.random.default_rng(cfg.seed+100003)
    packets=[];skip=0
    for k,sec in enumerate(t):
        if k==0 or not np.isclose(sec/cfg.navigation_period_s,round(sec/cfg.navigation_period_s),atol=1e-8):continue
        if cfg.measurement_mode=='cartesian':
            obs=truth[k,:3]+rng.normal(0,cfg.position_sigma_m/1000.,3)
            packets.append(make_packet(sec,cfg.telemetry_latency_s,obs))
        else:
            match=None
            for st in stations(9):
                found=ground_observation(truth[k],*st,sec,5.)
                if found is not None:match=(st,found[0]);break
            if match is None:skip+=1;continue
            st,obs=match
            noise=np.array([cfg.position_sigma_m/1000.,.03/1000.])
            packets.append(make_packet(sec,cfg.telemetry_latency_s,obs+rng.normal(0,1,2)*noise,
                                       kind='ground_range_rate',station=list(st)))
    # Prior comes from user-specified ideal orbital state, NOT actual perturbed truth.
    x0=np.asarray(state_from_elements(6378.137+cfg.altitude_km,0.,cfg.inclination_deg),float)
    radial=x0[:3]/np.linalg.norm(x0[:3]);along=x0[3:]/np.linalg.norm(x0[3:])
    x0[:3]+=radial*cfg.initial_position_offset_m/1000.
    x0[3:]+=along*cfg.initial_velocity_offset_m_s/1000.
    P0=np.diag([max(.1,cfg.initial_position_offset_m*2)/1000.] * 3 +
               [max(.001,cfg.initial_velocity_offset_m_s*2)/1000.] * 3)**2
    estimates=[];poserr=[];sigmas=[];velerr=[];num=[];mean_nis=[]
    for now in t:
        x,P,nis,count=reconstruct_estimate(x0,P0,packets,float(now),cfg,t,q,attitude_std)
        estimates.append(x.tolist())
        poserr.append(float(np.linalg.norm(x[:3]-truth[len(estimates)-1,:3])*1000.))
        velerr.append(float(np.linalg.norm(x[3:]-truth[len(estimates)-1,3:])*1000.))
        sigmas.append(float(sqrt(np.trace(P[:3,:3]))*1000.))
        num.append(count);mean_nis.append(float(np.mean(nis)) if nis else None)
    out['name']='Integrated Navigation · causal time-tagged position EKF + attitude MEKF'
    out['input']=cfg.__dict__.copy()
    out['integrated_navigation']={
        'estimated_state_km_km_s':estimates,'position_error_m':poserr,'velocity_error_m_s':velerr,
        'position_1sigma_rss_m':sigmas,'received_packet_count':num,
        'mean_nis_received':mean_nis,'packets':packets,'packets_generated':len(packets),
        'packets_skipped_visibility':skip,'final_position_error_m':poserr[-1],
        'final_velocity_error_m_s':velerr[-1],
        'final_position_1sigma_rss_m':sigmas[-1],
        'final_attitude_estimation_error_deg':out['precision_adcs']['final_estimation_error_deg'],
        'tracker_fixes':out['precision_adcs']['tracker_fixes'],
        'received_at_finish':num[-1],
        'late_or_pending_at_finish':len(packets)-num[-1],
        'frame':FRAME,'fusion_architecture':'loosely coupled 6+6, no joint cross-covariance',
        'attitude_covariance_augmented':cfg.propagate_attitude_covariance,
        'final_position_error_over_sigma_rss':float(poserr[-1]/max(sigmas[-1],1e-9)),
    }
    out['model']['name']='v1.3 causal delayed-message Cartesian EKF alongside v1.2 MEKF'
    out['model']['reference_frame']=FRAME
    out['model']['assumptions']+=['sensor timestamps measured in elapsed seconds from common t0',
         'captured packets assimilated only after receipt by causal chronological replay',
         'navigation state is separate from MEKF; no position-attitude cross-covariance',
         'the ideal inertial reference axes are not ICRF, GCRS, TEME, or ITRF',
         'estimated MEKF attitude drives predicted engine direction, known nominal mass/thrust',
         'sensor position fixes in ideal inertial axes; ground stations are schematic ideal spin']
    if cfg.propagate_attitude_covariance:
        out['model']['assumptions']+=['First-order attitude-to-translation covariance injection during finite thrust; isotropic MEKF angle RSS proxy',
          'Time-correlated pointing uncertainty and position-attitude cross covariance are not modeled']
        out['warnings']+=['Powered covariance augmentation is approximate; it does not correct biased thrust prediction or replace a joint filter.']
    out['warnings']+=['Navigation packets use synthetic position or fictional ground-station tracking.',
         'No hardware clock bias, correlated errors, real Earth orientation, IGRF, or JPL solar ephemeris.',
         'The attitude covariance is not fused into translational covariance; uncertainty may be optimistic.']
    return out
