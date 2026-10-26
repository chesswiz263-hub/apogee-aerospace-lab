"""APOGEE v1.5 -- research-grade joint orbit/attitude/bias error-state EKF.

Nominal state: inertial r, v (km, km/s), body-to-inertial q (Hamilton scalar-first),
body gyro bias (rad/s). Error state: [dr_I, dv_I, dtheta_B, db_B] (12 dimensions).
Full 12x12 Joseph covariance includes position-attitude and velocity-bias blocks.
Tracker/position packets are applied at their capture epochs ONLY AFTER receipt.
The simulator's true state is used exclusively to synthesize measurements and
assess accuracy. All axes are APOGEE_IDEAL_ECI, NOT GCRS/ITRS/TEME.

Limits: fixed nominal thrust and mass model, no gyro scale factor, no correlated
navigation hardware clocks, no real star catalog/EOP and no flight qualification.
"""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import expm
from .integrated_nav import IntegratedNavConfig, FRAME, _jacobian_r
from .precision_adcs import (PrecisionADCSConfig, simulate_precision_adcs,
                             symmetric_psd, cross_matrix, tracker_visibility)
from .adcs import rotation_vector_quaternion as expq, quaternion_rotation_vector as logq
from .attitude import q_normalize, q_mul, q_conj, rotate_BI, G0
from .physics import state_from_elements
from .constants import EARTH_MU_KM3_S2 as MU
from .perturbations import j2_acceleration

@dataclass(frozen=True)
class JointNavConfig(IntegratedNavConfig):
    tracker_period_s: float = 60.
    tracker_latency_s: float = 5.
    tracker_sigma_deg: float = .04
    gyro_sigma_deg_s: float = .015
    initial_attitude_offset_deg: float = 3.

    def validate(self):
        super().validate()
        nums=(self.tracker_period_s,self.tracker_latency_s,self.tracker_sigma_deg,
              self.gyro_sigma_deg_s,self.initial_attitude_offset_deg)
        if not np.isfinite(nums).all() or not (10<=self.tracker_period_s<=600
           and 0<=self.tracker_latency_s<=600 and .001<=self.tracker_sigma_deg<=5
           and .00001<=self.gyro_sigma_deg_s<=2 and 0<=self.initial_attitude_offset_deg<=90
           ):
            raise ValueError('Invalid joint navigation sensor configuration')
        step=self.duration_minutes*60/self.samples
        if not np.isclose(self.tracker_period_s/step,round(self.tracker_period_s/step),atol=1e-9):
            raise ValueError('Tracker period must be an integer multiple of output step')
        if self.samples<31:
            raise ValueError('Joint navigation requires at least 31 output steps for ADCS integration')
        if self.measurement_mode!='cartesian':
            raise ValueError('v1.5 joint filter currently supports synthetic Cartesian position fixes only')


def _rotation(q):
    return np.column_stack([rotate_BI(q,[1.,0.,0.]),rotate_BI(q,[0.,1.,0.]),rotate_BI(q,[0.,0.,1.])])


def dynamics_jacobian(x,q,bias,gyro,t,config):
    """12x12 continuous linearization; thrust pointing uses BODY error coordinates."""
    F=np.zeros((12,12));F[:3,3:6]=np.eye(3)
    F[3:6,:3]=_jacobian_r(x[:3],config.j2_enabled)
    omega=np.asarray(gyro)-bias
    F[6:9,6:9]=-cross_matrix(omega)
    F[6:9,9:12]=-np.eye(3)
    if config.burn_start_s<=t<config.burn_start_s+config.burn_duration_s:
        flow=850./(315.*G0)
        mass=max(600.,750.-flow*max(0.,t-config.burn_start_s))
        F[3:6,6:9]=-(850./mass/1000.)*(_rotation(q)@cross_matrix([1.,0.,0.]))
    return F


def _accel(t,x,q,config):
    r=x[:3]; n=np.linalg.norm(r)
    a=-MU*r/n**3
    if config.j2_enabled: a+=j2_acceleration(r)
    if config.burn_start_s<=t<config.burn_start_s+config.burn_duration_s:
        mass=max(600.,750.-850./(315.*G0)*(t-config.burn_start_s))
        a+=(850.*getattr(config,"assumed_thrust_factor",1.)/mass/1000.)*rotate_BI(q,[1.,0.,0.])
    return a


def predict(s,P,t0,t1,gyro,config):
    """One propagation interval; split at exact burn switching events."""
    s=np.array(s,float,copy=True);P=np.array(P,float,copy=True)
    edges=sorted(set([t0,t1]+[z for z in (config.burn_start_s,config.burn_start_s+config.burn_duration_s)
                                if t0+1e-9<z<t1-1e-9]))
    for a,b in zip(edges[:-1],edges[1:]):
        dt=b-a
        if dt<1e-10:continue
        x=s[:6].copy();q=s[6:10].copy();bias=s[10:13].copy()
        omega=np.asarray(gyro)-bias
        half=q_normalize(q_mul(q,expq(omega*dt/2)))
        full=q_normalize(q_mul(q,expq(omega*dt)))
        def deriv(t,y,att):return np.r_[y[3:],_accel(t,y,att,config)]
        k1=deriv(a,x,q)
        k2=deriv(a+dt/2,x+k1*dt/2,half)
        k3=deriv(a+dt/2,x+k2*dt/2,half)
        k4=deriv(b-1e-10,x+k3*dt,full)
        s[:6]=x+dt*(k1+2*k2+2*k3+k4)/6
        s[6:10]=full
        F=dynamics_jacobian(s[:6],half,bias,gyro,(a+b)/2,config)
        Phi=expm(F*dt)
        # Continuous acceleration noise, gyro white rate noise and gyro-bias RW.
        # The acceleration white-noise block includes correct position/velocity integration.
        Q=np.zeros((12,12))
        sigma_a=2e-7  # km/s^2 / sqrt(Hz): illustrative process noise
        I=np.eye(3)
        Q[:3,:3]=sigma_a**2*dt**3/3*I
        Q[:3,3:6]=sigma_a**2*dt**2/2*I
        Q[3:6,:3]=Q[:3,3:6]
        Q[3:6,3:6]=sigma_a**2*dt*I
        sigma_g=np.deg2rad(config.gyro_sigma_deg_s)
        sigma_b=np.deg2rad(.00003)
        Q[6:9,6:9]=I*sigma_g**2*dt**2
        Q[9:12,9:12]=I*sigma_b**2*dt
        P=symmetric_psd(Phi@P@Phi.T+Q,1e-23)
    return s,P


def update(s,P,packet,config):
    """Apply joint EKF update, attitude error injection/reset, Joseph covariance."""
    if packet['frame']!=FRAME:raise ValueError('Incompatible navigation measurement frame')
    if packet['kind']=='position':
        H=np.zeros((3,12));H[:,:3]=np.eye(3)
        residual=np.asarray(packet['z'])-s[:3]
        R=np.eye(3)*(config.position_sigma_m/1000.)**2
    elif packet['kind']=='tracker':
        H=np.zeros((3,12));H[:,6:9]=np.eye(3)
        residual=logq(q_mul(q_conj(s[6:10]),q_normalize(packet['z'])))
        R=np.eye(3)*np.deg2rad(config.tracker_sigma_deg)**2
    else:raise ValueError('Unknown measurement type')
    S=H@P@H.T+R
    gain=np.linalg.solve(S,(P@H.T).T).T
    delta=gain@residual
    new=s.copy();new[:6]+=delta[:6]
    new[6:10]=q_normalize(q_mul(s[6:10],expq(delta[6:9])))
    new[10:13]+=delta[9:12]
    I=np.eye(12);IKH=I-gain@H
    post=IKH@P@IKH.T+gain@R@gain.T
    reset=I.copy();reset[6:9,6:9]-=.5*cross_matrix(delta[6:9])
    post=symmetric_psd(reset@post@reset.T,1e-23)
    nis=float(residual@np.linalg.solve(S,residual))
    return new,post,nis


def _initial_filter(config,truth):
    x0=np.asarray(state_from_elements(6378.137+config.altitude_km,0.,config.inclination_deg),float)
    x0[:3]+=x0[:3]/np.linalg.norm(x0[:3])*config.initial_position_offset_m/1000.
    x0[3:]+=x0[3:]/np.linalg.norm(x0[3:])*config.initial_velocity_offset_m_s/1000.
    q0=q_normalize(q_mul(truth[0,6:10],expq([0.,0.,np.deg2rad(config.initial_attitude_offset_deg)])))
    state=np.r_[x0,q0,np.zeros(3)]
    p=(max(0.1,config.initial_position_offset_m*2)/1000.)**2
    v=(max(.001,config.initial_velocity_offset_m_s*2)/1000.)**2
    P=np.diag([p]*3+[v]*3+[np.deg2rad(max(1.,config.initial_attitude_offset_deg))**2]*3+
              [np.deg2rad(.025)**2]*3)
    return state,P


def _replay(s0,P0,times,gyro,packets,now,config):
    """Offline fixed-lag replay from t0 (transparent, expensive but reproducible)."""
    permitted=sorted((p for p in packets if p['receipt_s']<=now+1e-9),
                     key=lambda p:(p['capture_s'],p['kind']))
    s=s0.copy();P=P0.copy();cursor=0;used=0;nis=[]
    for i in range(len(times)-1):
        if times[i]>=now-1e-9:break
        s,P=predict(s,P,float(times[i]),float(min(times[i+1],now)),0.5*(gyro[i]+gyro[i+1]),config)
        stamp=times[i+1]
        if stamp>now+1e-9:break
        while cursor<len(permitted) and permitted[cursor]['capture_s']<=stamp+1e-9:
            # The only supported sensor times are grid times; a delayed observation
            # received at t_now must update at its capture epoch, never at receipt.
            pkt=permitted[cursor]
            if not np.isclose(pkt['capture_s'],stamp,atol=1e-7):
                raise RuntimeError('Packet must be updated at capture epoch')
            s,P,z=update(s,P,pkt,config);nis.append(z);cursor+=1;used+=1
    return s,P,used,nis


def simulate_joint_navigation(config:JointNavConfig):
    config.validate()
    acfg=PrecisionADCSConfig(altitude_km=config.altitude_km,inclination_deg=config.inclination_deg,
                            duration_minutes=config.duration_minutes,samples=config.samples,
                            tracker_enabled=config.tracker_enabled,gyro_bias_deg_s=config.gyro_bias_deg_s,
                            burn_start_s=config.burn_start_s,burn_duration_s=config.burn_duration_s,
                            j2_enabled=config.j2_enabled,seed=config.seed)
    out=simulate_precision_adcs(acfg)
    time=np.asarray(out['trajectory']['time_s'],float)
    truth=np.c_[out['trajectory']['position_km'],out['trajectory']['velocity_km_s'],
                out['attitude']['quaternion_BI_wxyz'],out['attitude']['angular_velocity_body_rad_s']]
    true_bias=np.asarray(out['precision_adcs']['gyro_bias_true_rad_s'])
    rng=np.random.default_rng(config.seed+150015)
    gyro=truth[:,10:13]+true_bias+rng.normal(0,np.deg2rad(config.gyro_sigma_deg_s),(len(time),3))
    packet_list=[]
    visibility_blocked=0
    for i,now in enumerate(time):
        if i==0:continue
        if np.isclose(now/config.navigation_period_s,round(now/config.navigation_period_s),atol=1e-8):
            z=truth[i,:3]+rng.normal(0,config.position_sigma_m/1000.,3)
            packet_list.append({'kind':'position','frame':FRAME,'capture_s':float(now),
                                'receipt_s':float(now+config.telemetry_latency_s),'z':z.tolist()})
        if config.tracker_enabled and np.isclose(now/config.tracker_period_s,round(now/config.tracker_period_s),atol=1e-8):
            allowed,*_=tracker_visibility(truth[i,:3],truth[i,6:10],acfg)
            if not allowed:
                visibility_blocked+=1
                continue
            z=q_normalize(q_mul(truth[i,6:10],expq(rng.normal(0,np.deg2rad(config.tracker_sigma_deg),3))))
            packet_list.append({'kind':'tracker','frame':FRAME,'capture_s':float(now),
                                'receipt_s':float(now+config.tracker_latency_s),'z':z.tolist()})
    initial,P0=_initial_filter(config,truth)
    output=[];perr=[];v_err=[];att_err=[];sig=[];s_att=[];cross=[];used=[];nises=[];biases=[]
    min_eigen=1.
    for now in time:
        x,P,count,nis=_replay(initial,P0,time,gyro,packet_list,float(now),config)
        k=len(output);output.append(x[:6].tolist());biases.append(x[10:13].tolist())
        perr.append(float(np.linalg.norm(x[:3]-truth[k,:3])*1000.))
        v_err.append(float(np.linalg.norm(x[3:6]-truth[k,3:6])*1000.))
        angle=logq(q_mul(q_conj(x[6:10]),truth[k,6:10]))
        att_err.append(float(np.rad2deg(np.linalg.norm(angle))))
        sig.append(float(np.sqrt(np.trace(P[:3,:3]))*1000.))
        s_att.append(float(np.rad2deg(np.sqrt(np.trace(P[6:9,6:9])))))
        # Dimensionless correlation of each position/attitude component.
        den=np.sqrt(np.outer(np.maximum(P.diagonal()[:3],1e-25),np.maximum(P.diagonal()[6:9],1e-25)))
        cross.append(float(np.max(np.abs(P[:3,6:9]/den))))
        used.append(count);nises.append(float(np.mean(nis)) if nis else None)
        min_eigen=min(min_eigen,float(np.linalg.eigvalsh(P).min()))
    last=packet_list
    result={'estimated_state_km_km_s':output,'position_error_m':perr,
            'velocity_error_m_s':v_err,'attitude_error_deg':att_err,
            'position_1sigma_rss_m':sig,'attitude_1sigma_rss_deg':s_att,
            'position_attitude_max_abs_correlation':cross,
            'estimated_gyro_bias_rad_s':biases,
            'received_packet_count':used,'mean_nis_received':nises,
            'packets_generated':len(last),'received_at_finish':used[-1],
            'pending_at_finish':len(last)-used[-1],
            'tracker_visibility_blocked':visibility_blocked,
            'final_position_error_m':perr[-1],'final_position_1sigma_rss_m':sig[-1],
            'final_attitude_estimation_error_deg':att_err[-1],
            'final_attitude_1sigma_rss_deg':s_att[-1],
            'final_position_attitude_max_abs_correlation':cross[-1],
            'max_position_attitude_abs_correlation':max(cross),
            'min_covariance_eigenvalue':min_eigen,
            'final_error_over_position_sigma_rss':perr[-1]/max(sig[-1],1e-9),
            'cross_covariance_modeled':True,'filter_error_dimension':12,
            'reference_frame':FRAME,'packet_metadata':[{'kind':p['kind'],'capture_s':p['capture_s'],
                              'receipt_s':p['receipt_s'],'frame':p['frame']} for p in packet_list]}
    out['joint_navigation']=result
    out['name']='Joint navigation · 12-state orbit/attitude/bias error EKF'
    out['input']=config.__dict__.copy()
    out['model']['name']='v1.5 12-error-state joint Cartesian+MEKF navigation prototype'
    out['model']['reference_frame']=FRAME
    out['model']['assumptions']+=['Joint position, velocity, body-angle and gyro-bias 12x12 covariance',
        'Tracker and Cartesian fixes are synthetic and time-tagged; late packets replayed at capture epoch',
        'Gyro measurements are simulated from true body rates with noisy bias; truth is not used by filter updates',
        'Nominal mass and thrust perfectly known; separate 17-state ADCS truth is held external to estimator',
        'Linearized covariance approximates nonlinear dynamics, process noise is illustrative',
        'Earth-fixed, TEME, GCRS and ITRS states are not interchangeable with APOGEE_IDEAL_ECI']
    out['warnings'] += ['12x12 joint covariance is an educational error-state EKF, not a flight-qualified navigation estimator.',
                        'True attitude dynamics and process noise are not independently validated with flight telemetry.',
                        'Measurement replay is complete from t0 for clarity; computational cost grows quadratically.']
    return out
