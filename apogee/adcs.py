"""APOGEE v1.1: educational closed-loop attitude determination & reaction wheels.

Truth state: r_I (km), v_I (km/s), q_BI (scalar-first), omega_B (rad/s),
wet mass (kg), and three orthogonal body-axis wheel momenta h_B (N m s).
Estimation: gyro-aided complementary quaternion filter, not an EKF.
All controller commands use estimated attitude / measured gyro rate, never truth.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.integrate import solve_ivp
from .attitude import (G0, MU, RE, initial_state, AttitudeConfig, q_normalize,
                       q_conj, q_mul, rotate_BI, target_attitude, attitude_error_deg,
                       gravity_gradient_torque)
from .physics import elements_from_state
from .perturbations import j2_acceleration


def rotation_vector_quaternion(vec):
    """Hamilton scalar-first quaternion, body-frame right perturbation."""
    vec=np.asarray(vec,dtype=float)
    theta=float(np.linalg.norm(vec))
    if theta<1e-12:
        return q_normalize(np.r_[1., .5*vec])
    return np.r_[math.cos(theta/2), math.sin(theta/2)/theta*vec]


def quaternion_rotation_vector(q):
    q=q_normalize(q)
    if q[0]<0:q=-q
    s=float(np.linalg.norm(q[1:]))
    if s<1e-12:return 2*q[1:]
    return 2*math.atan2(s,q[0])*q[1:]/s


def quaternion_distance_deg(a,b):
    return float(np.rad2deg(np.linalg.norm(quaternion_rotation_vector(q_mul(q_conj(q_normalize(a)),q_normalize(b))))))


@dataclass(frozen=True)
class ADCSConfig:
    altitude_km: float=400.
    inclination_deg: float=51.6
    duration_minutes: float=18.
    samples: int=109
    initial_misalignment_deg: float=25.
    initial_estimate_error_deg: float=4.
    target_mode: str='prograde'
    controller_enabled: bool=True
    tracker_enabled: bool=True
    gyro_bias_deg_s: float=.015
    gyro_noise_deg_s: float=.003
    tracker_noise_deg: float=.035
    tracker_interval_s: float=30.
    tracker_dropout_start_s: float=400.
    tracker_dropout_duration_s: float=0.
    attitude_blend: float=.75
    bias_adaptation: float=.08
    wheel_max_torque_nm: float=.20
    wheel_momentum_limit_nms: float=8.
    kp_nm_per_rad: float=1.2
    kd_nms_per_rad: float=16.
    dry_mass_kg: float=600.
    fuel_mass_kg: float=150.
    thrust_n: float=850.
    specific_impulse_s: float=315.
    burn_start_s: float=300.
    burn_duration_s: float=120.
    j2_enabled: bool=True
    gravity_gradient_enabled: bool=False
    seed: int=27
    inertia_kg_m2: tuple=(120.,100.,80.)

    def validate(self):
        values=[v for k,v in vars(self).items() if isinstance(v,(int,float)) and not isinstance(v,bool)]
        if not np.all(np.isfinite(values)) or not np.all(np.isfinite(self.inertia_kg_m2)):
            raise ValueError('ADCS parameters must be finite')
        if not 200<=self.altitude_km<=5000 or not 0<=self.inclination_deg<=180:raise ValueError('Unsupported initial orbit')
        if not 1<=self.duration_minutes<=90 or not 31<=self.samples<=601:raise ValueError('Unsupported ADCS duration/samples')
        if not 0<=self.initial_misalignment_deg<=180 or not 0<=self.initial_estimate_error_deg<=90:raise ValueError('Invalid initial attitude')
        if self.target_mode not in ('prograde','retrograde','inertial_x'):raise ValueError('Invalid target mode')
        if min(self.inertia_kg_m2)<=0:raise ValueError('Inertia must be positive')
        if min(self.gyro_bias_deg_s,self.gyro_noise_deg_s,self.tracker_noise_deg)<0:raise ValueError('Sensor errors must be nonnegative')
        if self.tracker_interval_s<1 or self.tracker_dropout_start_s<0 or self.tracker_dropout_duration_s<0:raise ValueError('Invalid tracker schedule')
        if not 0<=self.attitude_blend<=1 or not 0<=self.bias_adaptation<=1:raise ValueError('Filter gains must lie in [0,1]')
        if min(self.wheel_max_torque_nm,self.wheel_momentum_limit_nms,self.kp_nm_per_rad,self.kd_nms_per_rad)<0:raise ValueError('Invalid wheel/control settings')
        if self.dry_mass_kg<=0 or self.fuel_mass_kg<0 or self.thrust_n<0 or self.specific_impulse_s<=0:raise ValueError('Invalid propulsion')
        if self.burn_start_s<0 or self.burn_duration_s<0 or self.burn_start_s+self.burn_duration_s>self.duration_minutes*60+1e-8:raise ValueError('Burn outside mission')
        if not 0<=self.seed<=2**32-1:raise ValueError('Seed out of range')


def estimated_body_torque(q_est, measured_rate, r, v, cfg):
    """Quaternion error PD, derived solely from estimate, measurements and target."""
    if not cfg.controller_enabled:return np.zeros(3)
    q_d,w_d=target_attitude(r,v,cfg.target_mode)
    error=q_mul(q_conj(q_normalize(q_est)),q_d)
    sign=1 if error[0]>=0 else -1
    return 2*cfg.kp_nm_per_rad*sign*error[1:]+cfg.kd_nms_per_rad*(w_d-measured_rate)


def allocate_wheel_motor_torque(desired_body_torque, wheel_h, dt, cfg):
    """h_dot=-tau_body; per-axis motor torque and reachable h bounds."""
    if not cfg.controller_enabled:return np.zeros(3)
    hdot=np.clip(-np.asarray(desired_body_torque,float),-cfg.wheel_max_torque_nm,cfg.wheel_max_torque_nm)
    L=cfg.wheel_momentum_limit_nms
    return np.minimum(np.maximum(hdot,(-L-wheel_h)/dt),(L-wheel_h)/dt)


def adcs_dynamics(t,y,cfg,hdot,engine_on):
    r,v,q,w=y[:3],y[3:6],y[6:10],y[10:13]
    mass=y[13];h=y[14:17]
    a=-MU*r/np.linalg.norm(r)**3
    if cfg.j2_enabled:a+=j2_acceleration(r)
    if engine_on and cfg.thrust_n>0 and mass>cfg.dry_mass_kg+1e-8:
        a+=cfg.thrust_n/mass/1000*rotate_BI(q,[1.,0,0])
        mdot=-cfg.thrust_n/(cfg.specific_impulse_s*G0)
    else:mdot=0.
    I=np.asarray(cfg.inertia_kg_m2)
    tau_external=gravity_gradient_torque(y,I) if cfg.gravity_gradient_enabled else np.zeros(3)
    wdot=(tau_external-hdot-np.cross(w,I*w+h))/I
    return np.r_[v,a,.5*q_mul(q,np.r_[0.,w]),wdot,mdot,hdot]


def _integrate_interval(y,lo,hi,hdot,cfg):
    """Split power boundaries and exact ideal mass exhaustion time."""
    limits=[lo,hi]
    burn_lo, burn_hi=cfg.burn_start_s,cfg.burn_start_s+cfg.burn_duration_s
    for t in (burn_lo,burn_hi):
        if lo+1e-9<t<hi-1e-9:limits.append(t)
    limits.sort()
    state=np.array(y,copy=True)
    for a,b in zip(limits[:-1],limits[1:]):
        firing=burn_lo<=.5*(a+b)<burn_hi and cfg.thrust_n>0
        if firing:
            massflow=cfg.thrust_n/(cfg.specific_impulse_s*G0)
            exhaustion=a+max(0.,state[13]-cfg.dry_mass_kg)/massflow
            segments=[(a,min(b,exhaustion),True)] if exhaustion>a+1e-10 else []
            if exhaustion<b-1e-10:segments.append((max(a,exhaustion),b,False))
        else:segments=[(a,b,False)]
        for aa,bb,power in segments:
            if bb-aa<1e-10:continue
            sol=solve_ivp(lambda t,s:adcs_dynamics(t,s,cfg,hdot,power),
                          [aa,bb],state,method='DOP853',rtol=2e-10,atol=1e-12,max_step=3.)
            if not sol.success:raise RuntimeError('ADCS integration failed: '+sol.message)
            state=sol.y[:,-1].copy()
            state[6:10]=q_normalize(state[6:10])
            state[13]=max(cfg.dry_mass_kg,state[13])
    return state


def simulate_adcs(cfg:ADCSConfig)->dict:
    cfg.validate()
    rng=np.random.default_rng(cfg.seed)
    base=AttitudeConfig(altitude_km=cfg.altitude_km,inclination_deg=cfg.inclination_deg,
                        initial_misalignment_deg=cfg.initial_misalignment_deg,target_mode=cfg.target_mode,
                        dry_mass_kg=cfg.dry_mass_kg,fuel_mass_kg=cfg.fuel_mass_kg)
    y=np.r_[initial_state(base),np.zeros(3)]
    q_est=q_normalize(q_mul(y[6:10],rotation_vector_quaternion([0,0,np.deg2rad(cfg.initial_estimate_error_deg)])))
    bias_true=np.array([0.,0.,np.deg2rad(cfg.gyro_bias_deg_s)])
    b_est=np.zeros(3)
    dt=cfg.duration_minutes*60/cfg.samples
    times=np.arange(cfg.samples+1)*dt
    rows=[];q_est_rows=[];err_true=[];err_est=[];err_nav=[];wheelcmd=[];tracker_events=[]
    tracking_count=0;tracker_outage_count=0;saturated_count=0;last_tracker=-float('inf')
    last_gyro=None
    for j,t in enumerate(times):
        # A sample is taken at the current true state; the truth is never directly
        # provided to the controller or attitude estimator.
        gyro=y[10:13]+bias_true+rng.normal(0,np.deg2rad(cfg.gyro_noise_deg_s),3)
        if j>0:
            q_est=q_normalize(q_mul(q_est,rotation_vector_quaternion((last_gyro-b_est)*dt)))
        in_outage=cfg.tracker_dropout_start_s<=t<cfg.tracker_dropout_start_s+cfg.tracker_dropout_duration_s
        tracker_ok=cfg.tracker_enabled and not in_outage and t-last_tracker>=cfg.tracker_interval_s-1e-8
        if tracker_ok:
            observation=q_mul(y[6:10],rotation_vector_quaternion(rng.normal(0,np.deg2rad(cfg.tracker_noise_deg),3)))
            innovation=quaternion_rotation_vector(q_mul(q_conj(q_est),observation))
            q_est=q_normalize(q_mul(q_est,rotation_vector_quaternion(cfg.attitude_blend*innovation)))
            if last_tracker>-1e20:
                elapsed=t-last_tracker
                b_est-=cfg.bias_adaptation*innovation/max(elapsed,1)
                b_est=np.clip(b_est,-.02,.02)
            tracking_count+=1;last_tracker=t
            tracker_events.append({'type':'star_tracker_fix','time_s':float(t)})
        elif cfg.tracker_enabled and in_outage:tracker_outage_count+=1
        rows.append(y.copy());q_est_rows.append(q_est.copy())
        err_true.append(attitude_error_deg(y[6:10],y[:3],y[3:6],cfg.target_mode))
        err_est.append(attitude_error_deg(q_est,y[:3],y[3:6],cfg.target_mode))
        err_nav.append(quaternion_distance_deg(q_est,y[6:10]))
        if j==cfg.samples:break
        command=estimated_body_torque(q_est,gyro-b_est,y[:3],y[3:6],cfg)
        hdot=allocate_wheel_motor_torque(command,y[14:17],dt,cfg)
        if np.any(np.abs(hdot+command)>1e-7):saturated_count+=1
        wheelcmd.append(hdot.copy())
        y=_integrate_interval(y,t,t+dt,hdot,cfg)
        y[14:17]=np.clip(y[14:17],-cfg.wheel_momentum_limit_nms,cfg.wheel_momentum_limit_nms)
        last_gyro=gyro
    s=np.asarray(rows); qs=np.asarray(q_est_rows)
    error_true=np.asarray(err_true);nav_error=np.asarray(err_nav)
    masses=s[:,13];fuel_used=float(cfg.dry_mass_kg+cfg.fuel_mass_kg-masses[-1])
    altitudes=np.linalg.norm(s[:,:3],axis=1)-RE
    motion_speeds=np.linalg.norm(s[:,3:6],axis=1)
    final_mass=float(masses[-1])
    firing_t=min(cfg.burn_duration_s,max(0.,(cfg.fuel_mass_kg*cfg.specific_impulse_s*G0)/cfg.thrust_n)) if cfg.thrust_n>0 else 0
    elapsed_burn=min(cfg.burn_duration_s,firing_t)
    burn_end=cfg.burn_start_s+elapsed_burn
    cumulative=cfg.specific_impulse_s*G0*np.log((cfg.dry_mass_kg+cfg.fuel_mass_kg)/np.maximum(masses,1e-9))
    stats={'final_pointing_error_deg':float(error_true[-1]),
           'peak_pointing_error_deg':float(error_true.max()),
           'final_estimation_error_deg':float(nav_error[-1]),
           'rms_estimation_error_deg':float(np.sqrt(np.mean(nav_error**2))),
           'tracker_fixes':tracking_count,'tracker_outage_samples':tracker_outage_count,
           'wheel_limit_steps':saturated_count,
           'max_wheel_momentum_nms':float(np.max(np.abs(s[:,14:17]))),
           'max_wheel_torque_nm':float(np.max(np.abs(wheelcmd))) if wheelcmd else 0.,
           'propellant_used_kg':fuel_used,'propellant_remaining_kg':float(final_mass-cfg.dry_mass_kg),
           'ideal_engine_delta_v_m_s':float(cumulative[-1])}
    traj={'time_s':times.tolist(),'position_km':s[:,:3].tolist(),'velocity_km_s':s[:,3:6].tolist(),
          'altitude_km':altitudes.tolist(),'speed_km_s':motion_speeds.tolist(),
          'wet_mass_kg':masses.tolist(),'propellant_remaining_kg':(masses-cfg.dry_mass_kg).tolist(),
          'thrust_active':((times>=cfg.burn_start_s)&(times<burn_end)&(cfg.fuel_mass_kg>0)).tolist(),
          'cumulative_ideal_delta_v_m_s':cumulative.tolist()}
    attitude={'quaternion_BI_wxyz':s[:,6:10].tolist(),'angular_velocity_body_rad_s':s[:,10:13].tolist(),
              'body_x_inertial':[rotate_BI(q,[1,0,0]).tolist() for q in s[:,6:10]],
              'pointing_error_deg':error_true.tolist(),
              'control_torque_nm':np.linalg.norm(wheelcmd,axis=1).tolist()+[0],
              'final_pointing_error_deg':stats['final_pointing_error_deg'],
              'peak_pointing_error_deg':stats['peak_pointing_error_deg']}
    metrics={'elapsed_s':float(times[-1]),'samples':len(times),'step_s':dt,
             'min_altitude_km':float(altitudes.min()),'max_altitude_km':float(altitudes.max()),
             'initial_propellant_kg':cfg.fuel_mass_kg,'propellant_used_kg':fuel_used,
             'propellant_remaining_kg':stats['propellant_remaining_kg'],
             'ideal_thrust_delta_v_m_s':stats['ideal_engine_delta_v_m_s'],
             'fuel_exhausted':bool(cfg.fuel_mass_kg>0 and fuel_used>=cfg.fuel_mass_kg-1e-6),
             'conservation_applicable':False,'max_relative_energy_drift':None,'max_relative_angular_momentum_drift':None}
    return {'name':'ADCS: gyro+star tracker estimation and momentum-limited reaction wheels',
            'input':cfg.__dict__, 'trajectory':traj,'attitude':attitude,'adcs':{
               'estimated_quaternion_BI_wxyz':qs.tolist(),
               'true_quaternion_BI_wxyz':s[:,6:10].tolist(),
               'estimated_pointing_error_deg':list(map(float,err_est)),
               'attitude_estimation_error_deg':nav_error.tolist(),
               'estimated_gyro_bias_rad_s':b_est.tolist(),
               'wheel_momentum_body_nms':s[:,14:17].tolist(),
               'wheel_commanded_momentum_rate_nm':[w.tolist() for w in wheelcmd]+[[0.,0.,0.]],
               **stats},'metrics':metrics,
            'orbital_elements':elements_from_state(s[0,:6]).to_dict(),
            'final_orbital_elements':elements_from_state(s[-1,:6]).to_dict(),
            'events':tracker_events,
            'model':{'name':'17-state rigid body with reaction-wheel stored momentum; gyro/star tracker complementary attitude estimator',
                     'reference_frame':'Idealized ECI and body-to-inertial scalar-first active quaternion',
                     'integrator':'DOP853 stepwise with zero-order-held wheel motor commands; exact burn-time subdivision',
                     'assumptions':['three orthogonal ideal reaction wheels','per-axis torque and angular momentum bounds',
                                    'constant diagonal spacecraft inertia; no wheel rotor imbalance','star tracker gives direct noisy quaternion with optional blackout',
                                    'gyro has fixed body-Z bias and white sample noise','complementary quaternion estimator, not covariance-calibrated MEKF',
                                    'no momentum dumping; no sensor occultation model']},
            'warnings':['Educational research simulation only, not spacecraft flight software.',
                        'Wheel saturation and tracker blackout can leave residual pointing error.','No external momentum-unloading actuator is implemented.']}
