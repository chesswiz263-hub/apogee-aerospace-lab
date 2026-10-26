"""APOGEE v1.2 research-only multiplicative attitude EKF and dipole unloading.

17-state rigid spacecraft, 6-state right-multiplicative attitude error filter,
star-tracker exclusion-cone gating, gyro bias random walk, and 3-axis wheel
momentum exchange with geomagnetically constrained magnetorquer unloading.
No flight-grade ephemeris, magnetosphere, or sensor/covariance calibration.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.integrate import solve_ivp
from .adcs import (ADCSConfig, rotation_vector_quaternion as expq,
                   quaternion_rotation_vector as logq, quaternion_distance_deg,
                   estimated_body_torque)
from .attitude import (AttitudeConfig, initial_state, q_normalize, q_conj, q_mul,
                       rotate_BI, target_attitude, attitude_error_deg, gravity_gradient_torque,
                       G0, MU, RE)
from .perturbations import j2_acceleration
from .physics import elements_from_state


def cross_matrix(v):
    x,y,z=np.asarray(v,dtype=float)
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def symmetric_psd(m, floor=0.):
    s=.5*(m+m.T)
    ev,u=np.linalg.eigh(s)
    return (u*np.maximum(ev,floor))@u.T


@dataclass(frozen=True)
class PrecisionADCSConfig(ADCSConfig):
    gyro_bias_random_walk_deg_s_sqrt_s: float=.000015
    gyro_initial_bias_sigma_deg_s: float=.025
    gyro_bias_process_sigma_deg_s_sqrt_s: float=.00003
    star_tracker_sun_exclusion_deg: float=25.
    star_tracker_earth_limb_margin_deg: float=8.
    tracker_visibility_enabled: bool=True
    desaturation_enabled: bool=True
    magnetorquer_max_dipole_am2: float=120.
    desaturation_gain_per_s: float=.01
    initial_wheel_momentum_nms: float=2.

    def validate(self):
        super().validate()
        params=(self.gyro_bias_random_walk_deg_s_sqrt_s,self.gyro_initial_bias_sigma_deg_s,
                self.gyro_bias_process_sigma_deg_s_sqrt_s,self.star_tracker_sun_exclusion_deg,
                self.star_tracker_earth_limb_margin_deg,self.magnetorquer_max_dipole_am2,
                self.desaturation_gain_per_s,self.initial_wheel_momentum_nms)
        if not all(np.isfinite(params)) or min(params)<0:
            raise ValueError('Precision ADCS parameters must be finite and nonnegative')
        if self.star_tracker_sun_exclusion_deg>180 or self.star_tracker_earth_limb_margin_deg>60:
            raise ValueError('Invalid star tracker exclusion angle')
        if self.initial_wheel_momentum_nms>self.wheel_momentum_limit_nms:
            raise ValueError('Initial wheel momentum exceeds capacity')
        if self.magnetorquer_max_dipole_am2>2000 or self.desaturation_gain_per_s>5:
            raise ValueError('Magnetic unloading input beyond simulator limit')


class AttitudeMEKF:
    """Right-multiplicative small-angle MEKF, state [dtheta_B, db_B].

    gyro samples are zero-order held during each update period. P is 6x6.
    No Kalman gain accesses simulated truth; truth is used in diagnostics only.
    """
    def __init__(self, q0, sigma_attitude_deg=4., sigma_bias_deg_s=.025):
        self.q=q_normalize(q0)
        self.b=np.zeros(3)
        s_theta=np.deg2rad(sigma_attitude_deg)
        s_b=np.deg2rad(sigma_bias_deg_s)
        self.P=np.diag([s_theta*s_theta]*3+[s_b*s_b]*3)
        self.innovations=[]
        self.nis=[]

    def predict(self, gyro, dt, gyro_sigma_deg_s, bias_rw_deg_s_sqrt_s):
        rate=np.asarray(gyro)-self.b
        self.q=q_normalize(q_mul(self.q,expq(rate*dt)))
        F=np.zeros((6,6))
        F[:3,:3]=-cross_matrix(rate)
        F[:3,3:]=-np.eye(3)
        Phi=np.eye(6)+F*dt+.5*(F@F)*dt*dt
        sr=np.deg2rad(gyro_sigma_deg_s)
        sb=np.deg2rad(bias_rw_deg_s_sqrt_s)
        Q=np.diag([max(sr*sr*dt*dt,1e-22)]*3+[max(sb*sb*dt,1e-24)]*3)
        self.P=symmetric_psd(Phi@self.P@Phi.T+Q,1e-24)

    def update(self, q_observed, tracker_noise_deg):
        innov=logq(q_mul(q_conj(self.q),q_normalize(q_observed)))
        H=np.zeros((3,6)); H[:,:3]=np.eye(3)
        R=np.eye(3)*max(np.deg2rad(tracker_noise_deg)**2,1e-18)
        S=H@self.P@H.T+R
        gain=np.linalg.solve(S,(self.P@H.T).T).T
        dx=gain@innov
        self.q=q_normalize(q_mul(self.q,expq(dx[:3])))
        self.b+=dx[3:]
        IKH=np.eye(6)-gain@H
        # Joseph form and right-error reset map; helps ensure PSD and symmetry.
        post=IKH@self.P@IKH.T+gain@R@gain.T
        G=np.eye(6);G[:3,:3]-=.5*cross_matrix(dx[:3])
        self.P=symmetric_psd(G@post@G.T,1e-24)
        nis=float(innov@np.linalg.solve(S,innov))
        self.innovations.append(innov.copy())
        self.nis.append(nis)
        return nis


def magnetic_field_inertial_tesla(r_km):
    """Ideal stationary, centered axial dipole, NOT IGRF or Earth-fixed."""
    r=np.asarray(r_km,dtype=float)
    norm=np.linalg.norm(r)
    if norm<RE*.9:raise ValueError('Magnetic field requested inside Earth')
    n=r/norm; m=np.array([0.,0.,1.])
    return 3.12e-5*(RE/norm)**3*(3*n*np.dot(m,n)-m)


def tracker_visibility(r,q,cfg):
    """Synthetic boresight body +X; fixed inertial 'Sun' +X, not ephemeris."""
    boresight=rotate_BI(q,[1.,0,0]);boresight/=np.linalg.norm(boresight)
    dot_sun=np.clip(float(boresight[0]),-1,1)
    sun_angle=float(np.rad2deg(np.arccos(dot_sun)))
    earth_dir=-np.asarray(r)/np.linalg.norm(r)
    earth_angle=float(np.rad2deg(np.arccos(np.clip(boresight@earth_dir,-1,1))))
    earth_half=float(np.rad2deg(np.arcsin(np.clip(RE/np.linalg.norm(r),0,1))))
    allowed=sun_angle>=cfg.star_tracker_sun_exclusion_deg and earth_angle>=earth_half+cfg.star_tracker_earth_limb_margin_deg
    return bool(allowed),sun_angle,earth_angle-(earth_half+cfg.star_tracker_earth_limb_margin_deg)


def magnetic_unload_torque(r,q,h,cfg):
    """External body torque = m x B; no controllability along local B."""
    if not cfg.desaturation_enabled or not cfg.controller_enabled or cfg.magnetorquer_max_dipole_am2<=0:
        return np.zeros(3),np.zeros(3)
    B=rotate_BI(q_conj(q_normalize(q)),magnetic_field_inertial_tesla(r))
    b2=float(B@B)
    tau_d=-cfg.desaturation_gain_per_s*np.asarray(h,dtype=float)
    m=np.cross(B,tau_d)/max(b2,1e-30)
    mnorm=np.linalg.norm(m)
    if mnorm>cfg.magnetorquer_max_dipole_am2:
        m*=cfg.magnetorquer_max_dipole_am2/mnorm
    return np.cross(m,B),m


def _dynamics(t,y,cfg,hdot,magnetic_dipole,engine_on):
    r,v,q,w=y[:3],y[3:6],y[6:10],y[10:13];mass=y[13];h=y[14:17]
    a=-MU*r/np.linalg.norm(r)**3
    if cfg.j2_enabled:a+=j2_acceleration(r)
    if engine_on and cfg.thrust_n>0 and mass>cfg.dry_mass_kg+1e-8:
        a+=cfg.thrust_n/mass/1000*rotate_BI(q,[1.,0,0])
        mdot=-cfg.thrust_n/(cfg.specific_impulse_s*G0)
    else:mdot=0.
    I=np.asarray(cfg.inertia_kg_m2)
    # A *dipole command* is held; magnetic field and body orientation evolve.
    B_body=rotate_BI(q_conj(q_normalize(q)),magnetic_field_inertial_tesla(r))
    tau_mag=np.cross(magnetic_dipole,B_body)
    ext=tau_mag+(gravity_gradient_torque(y,I) if cfg.gravity_gradient_enabled else 0)
    wdot=(ext-hdot-np.cross(w,I*w+h))/I
    return np.r_[v,a,.5*q_mul(q,np.r_[0.,w]),wdot,mdot,hdot]


def _integrate(y,t0,t1,hdot,magnetic_dipole,cfg):
    bounds=sorted(set([t0,t1]+[t for t in (cfg.burn_start_s,cfg.burn_start_s+cfg.burn_duration_s) if t0<t<t1]))
    state=np.asarray(y,dtype=float).copy()
    for a,b in zip(bounds[:-1],bounds[1:]):
        firing=cfg.burn_start_s<=(a+b)/2<cfg.burn_start_s+cfg.burn_duration_s and cfg.thrust_n>0
        if firing:
            md=cfg.thrust_n/(cfg.specific_impulse_s*G0)
            dry_t=a+max(0.,state[13]-cfg.dry_mass_kg)/md
            pieces=[]
            if dry_t>a+1e-10:pieces.append((a,min(b,dry_t),True))
            if dry_t<b-1e-10:pieces.append((max(a,dry_t),b,False))
        else:pieces=[(a,b,False)]
        for aa,bb,on in pieces:
            if bb-aa<=1e-10:continue
            sol=solve_ivp(lambda t,s:_dynamics(t,s,cfg,hdot,magnetic_dipole,on),
                         (aa,bb),state,method='DOP853',rtol=2e-10,atol=1e-12,max_step=3.)
            if not sol.success:raise RuntimeError('Precision ADCS integration failed: '+sol.message)
            state=sol.y[:,-1].copy()
            state[6:10]=q_normalize(state[6:10]);state[13]=max(cfg.dry_mass_kg,state[13])
    return state


def simulate_precision_adcs(cfg:PrecisionADCSConfig):
    cfg.validate()
    rng=np.random.default_rng(cfg.seed)
    basic=AttitudeConfig(altitude_km=cfg.altitude_km,inclination_deg=cfg.inclination_deg,
                         initial_misalignment_deg=cfg.initial_misalignment_deg,target_mode=cfg.target_mode,
                         dry_mass_kg=cfg.dry_mass_kg,fuel_mass_kg=cfg.fuel_mass_kg)
    truth=np.r_[initial_state(basic),[0.,cfg.initial_wheel_momentum_nms,0.]]
    initial_q=q_mul(truth[6:10],expq([0.,0.,np.deg2rad(cfg.initial_estimate_error_deg)]))
    kf=AttitudeMEKF(initial_q,max(1.,cfg.initial_estimate_error_deg),cfg.gyro_initial_bias_sigma_deg_s)
    true_bias=np.array([0.,0.,np.deg2rad(cfg.gyro_bias_deg_s)])
    times=np.linspace(0,cfg.duration_minutes*60,cfg.samples+1);dt=float(times[1]-times[0])
    truth_rows=[];est_q_rows=[];wheel_cmd=[];mag_cmd=[];mag_moment=[];true_err=[];nav_err=[]
    bias_rows=[];bias_true_rows=[];att_std=[];nis_series=[];nees_series=[];events=[]
    tracker_count=0;blocked_sun=0;blocked_earth=0;blackout=0;limit_steps=0;mag_active=0
    last_tracker=-float('inf');last_gyro=None
    for j,t in enumerate(times):
        if j>0:
            true_bias+=rng.normal(0,np.deg2rad(cfg.gyro_bias_random_walk_deg_s_sqrt_s)*np.sqrt(dt),3)
        gyro=truth[10:13]+true_bias+rng.normal(0,np.deg2rad(cfg.gyro_noise_deg_s),3)
        if j>0:
            # Trapezoidal integration of two *measured* gyro samples, not truth.
            kf.predict(.5*(last_gyro+gyro),dt,cfg.gyro_noise_deg_s,cfg.gyro_bias_process_sigma_deg_s_sqrt_s)
        nis=float('nan')
        scheduled=cfg.tracker_enabled and t-last_tracker>=cfg.tracker_interval_s-1e-8
        in_outage=cfg.tracker_dropout_start_s<=t<cfg.tracker_dropout_start_s+cfg.tracker_dropout_duration_s
        view_ok,sun_angle,earth_margin=tracker_visibility(truth[:3],truth[6:10],cfg)
        if scheduled and in_outage:blackout+=1
        if scheduled and not in_outage:
            if cfg.tracker_visibility_enabled and not view_ok:
                if sun_angle<cfg.star_tracker_sun_exclusion_deg:blocked_sun+=1
                if earth_margin<0:blocked_earth+=1
            else:
                obs=q_mul(truth[6:10],expq(rng.normal(0,np.deg2rad(cfg.tracker_noise_deg),3)))
                nis=kf.update(obs,cfg.tracker_noise_deg)
                tracker_count+=1;last_tracker=t
                events.append({'type':'tracker_fix','time_s':float(t)})
        truth_rows.append(truth.copy());est_q_rows.append(kf.q.copy())
        true_err.append(attitude_error_deg(truth[6:10],truth[:3],truth[3:6],cfg.target_mode))
        nav_err.append(quaternion_distance_deg(kf.q,truth[6:10]))
        bias_rows.append(kf.b.copy());bias_true_rows.append(true_bias.copy())
        att_std.append(float(np.rad2deg(np.sqrt(max(0.,np.trace(kf.P[:3,:3]))))))
        nis_series.append(None if not np.isfinite(nis) else float(nis))
        delta=logq(q_mul(q_conj(kf.q),truth[6:10]))
        dx=np.r_[delta,true_bias-kf.b]
        nees=float(dx@np.linalg.solve(kf.P,dx))
        nees_series.append(nees)
        if j==cfg.samples:break
        desired=estimated_body_torque(kf.q,gyro-kf.b,truth[:3],truth[3:6],cfg)
        tau_mag,m=magnetic_unload_torque(truth[:3],truth[6:10],truth[14:17],cfg)
        if np.linalg.norm(m)>1e-10:mag_active+=1
        # The magnetic external moment is countered by the wheels when possible,
        # allowing wheel-momentum unloading with bounded attitude disturbance.
        hdot=np.clip(tau_mag-desired,-cfg.wheel_max_torque_nm,cfg.wheel_max_torque_nm)
        L=cfg.wheel_momentum_limit_nms
        hdot=np.clip(hdot,(-L-truth[14:17])/dt,(L-truth[14:17])/dt)
        if np.any(np.abs((tau_mag-desired)-hdot)>1e-8):limit_steps+=1
        wheel_cmd.append(hdot.copy());mag_cmd.append(tau_mag.copy());mag_moment.append(m.copy())
        truth=_integrate(truth,t,t+dt,hdot,m,cfg)
        truth[14:17]=np.clip(truth[14:17],-L,L)
        last_gyro=gyro
    s=np.asarray(truth_rows);qs=np.asarray(est_q_rows);masses=s[:,13]
    alts=np.linalg.norm(s[:,:3],axis=1)-RE;speeds=np.linalg.norm(s[:,3:6],axis=1)
    err=np.asarray(nav_err);std=np.asarray(att_std)
    consumed=float(cfg.dry_mass_kg+cfg.fuel_mass_kg-masses[-1]);delta_v=cfg.specific_impulse_s*G0*np.log(masses[0]/masses)
    stats={'final_pointing_error_deg':float(true_err[-1]),'final_estimation_error_deg':float(err[-1]),
           'rms_estimation_error_deg':float(np.sqrt(np.mean(err**2))),'tracker_fixes':tracker_count,
           'tracker_blocked_sun':blocked_sun,'tracker_blocked_earth':blocked_earth,
           'tracker_outage_samples':blackout,'wheel_limit_steps':limit_steps,'magnetorquer_active_steps':mag_active,
           'max_wheel_momentum_nms':float(np.max(np.abs(s[:,14:17]))),
           'final_wheel_momentum_norm_nms':float(np.linalg.norm(s[-1,14:17])),
           'max_wheel_torque_nm':float(max((np.max(np.abs(x)) for x in wheel_cmd),default=0.)),
           'mean_nis':float(np.mean(kf.nis)) if kf.nis else None,
           'mean_nees':float(np.mean(nees_series)),
           'final_attitude_3axis_sigma_deg':float(std[-1]),
           'estimated_final_bias_deg_s':np.rad2deg(kf.b).tolist(),
           'true_final_bias_deg_s':np.rad2deg(true_bias).tolist(),
           'propellant_used_kg':consumed,'propellant_remaining_kg':float(masses[-1]-cfg.dry_mass_kg),
           'ideal_engine_delta_v_m_s':float(delta_v[-1])}
    firing_until=cfg.burn_start_s+min(cfg.burn_duration_s,cfg.fuel_mass_kg*cfg.specific_impulse_s*G0/cfg.thrust_n) if cfg.thrust_n else cfg.burn_start_s
    traj={'time_s':times.tolist(),'position_km':s[:,:3].tolist(),'velocity_km_s':s[:,3:6].tolist(),
          'altitude_km':alts.tolist(),'speed_km_s':speeds.tolist(),'wet_mass_kg':masses.tolist(),
          'propellant_remaining_kg':(masses-cfg.dry_mass_kg).tolist(),
          'cumulative_ideal_delta_v_m_s':delta_v.tolist(),
          'thrust_active':((times>=cfg.burn_start_s)&(times<firing_until)&(cfg.fuel_mass_kg>0)).tolist()}
    att={'quaternion_BI_wxyz':s[:,6:10].tolist(),'angular_velocity_body_rad_s':s[:,10:13].tolist(),
         'body_x_inertial':[rotate_BI(q,[1,0,0]).tolist() for q in s[:,6:10]],
         'pointing_error_deg':list(map(float,true_err)),
         'control_torque_nm':list(map(float,[np.linalg.norm(w) for w in wheel_cmd]))+[0.],
         'final_pointing_error_deg':stats['final_pointing_error_deg']}
    return {'name':'Precision ADCS · MEKF and magnetic momentum unloading','input':cfg.__dict__,
            'trajectory':traj,'attitude':att,'precision_adcs':{
                'estimated_quaternion_BI_wxyz':qs.tolist(),
                'attitude_estimation_error_deg':err.tolist(),
                'estimated_attitude_3axis_sigma_deg':std.tolist(),
                'gyro_bias_estimated_rad_s':np.asarray(bias_rows).tolist(),
                'gyro_bias_true_rad_s':np.asarray(bias_true_rows).tolist(),
                'wheel_momentum_body_nms':s[:,14:17].tolist(),
                'wheel_commanded_momentum_rate_nm':[w.tolist() for w in wheel_cmd]+[[0.,0.,0.]],
                'magnetic_torque_nm':[w.tolist() for w in mag_cmd]+[[0.,0.,0.]],
                'magnetic_dipole_am2':[w.tolist() for w in mag_moment]+[[0.,0.,0.]],
                'nis':nis_series,'nees':nees_series,**stats},
            'metrics':{'elapsed_s':float(times[-1]),'samples':len(times),'step_s':dt,
                       'min_altitude_km':float(alts.min()),'max_altitude_km':float(alts.max()),
                       'initial_propellant_kg':cfg.fuel_mass_kg,'propellant_used_kg':consumed,
                       'propellant_remaining_kg':stats['propellant_remaining_kg'],
                       'ideal_thrust_delta_v_m_s':float(delta_v[-1]),
                       'fuel_exhausted':bool(cfg.fuel_mass_kg>0 and consumed>=cfg.fuel_mass_kg-1e-7),
                       'conservation_applicable':False,'max_relative_energy_drift':None,
                       'max_relative_angular_momentum_drift':None},
            'orbital_elements':elements_from_state(s[0,:6]).to_dict(),
            'final_orbital_elements':elements_from_state(s[-1,:6]).to_dict(),
            'events':events,'model':{'name':'17-state rigid body + 6-state MEKF + synthetic magnetic unloading',
                                     'reference_frame':'Idealized ECI; scalar-first Hamilton quaternion body to inertial',
                                     'integrator':'DOP853 with zero-order-held motor and magnetic dipole commands (B and attitude evaluated continuously)',
                                     'assumptions':['fixed inertial axial dipole, not IGRF',
                                                    'fixed inertial Sun +X, not a solar ephemeris',
                                                    'one synthetic star tracker with body +X boresight',
                                                    'constant diagonal spacecraft inertia',
                                                    'no motor dynamics, no actuator latency, no celestial ephemeris',
                                                    'MEKF covariance is conditional on approximate discretization']},
            'warnings':['Educational simulation, not flight software. NIS/NEES need ensemble calibration.',
                        'Stationary synthetic Sun/dipole, not physical reference ephemerides.',
                        'Magnetic torque has no component parallel to B and cannot unload all wheel axes at once.']}
