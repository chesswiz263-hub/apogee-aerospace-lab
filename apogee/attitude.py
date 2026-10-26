"""APOGEE v1.0: rigid-body attitude + orbit + body-fixed finite propulsion.

State [r km (3), v km/s (3), q_BI scalar-first (4), omega_B rad/s (3), wet mass kg].
q_BI actively rotates body vectors to the idealized inertial frame.  Its derivative
is 0.5 * q_BI ⊗ [0, omega_B] (Hamilton multiplication).  Body +X is thrust axis.

Educational model: diagonal and constant inertia, ideal PD torque with saturation;
no reaction-wheel storage, gimbals, torque allocation, sensor or delay model.
"""
from dataclasses import dataclass
import numpy as np
from scipy.integrate import solve_ivp
from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state
from .perturbations import j2_acceleration, EARTH_SPIN_RAD_S

G0=9.80665


def q_normalize(q):
    q=np.asarray(q,dtype=float)
    n=np.linalg.norm(q)
    if q.shape!=(4,) or not np.all(np.isfinite(q)) or n<=0: raise ValueError('Quaternion must be finite and nonzero')
    return q/n


def q_mul(a,b):
    w,x,y,z=a; W,X,Y,Z=b
    return np.array([w*W-x*X-y*Y-z*Z,
                     w*X+x*W+y*Z-z*Y,
                     w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W])


def q_conj(q): return np.array([q[0],-q[1],-q[2],-q[3]])


def rotate_BI(q,vector):
    q=q_normalize(q)
    v=np.asarray(vector,float)
    # Unit quaternion, 3D optimized Hamilton rotation.
    u=q[1:]; return v+2*q[0]*np.cross(u,v)+2*np.cross(u,np.cross(u,v))


def rotate_IB(q,vector): return rotate_BI(q_conj(q_normalize(q)),vector)


def q_from_dcm(R):
    """Active rotation matrix (columns are body axes in inertial coordinates)."""
    R=np.asarray(R,float)
    if R.shape!=(3,3) or not np.allclose(R.T@R,np.eye(3),atol=1e-9) or np.linalg.det(R)<0.999999:
        raise ValueError('Requires a right-handed orthonormal 3x3 rotation matrix')
    trace=float(np.trace(R)); q=np.empty(4)
    if trace>0:
        s=2*np.sqrt(trace+1); q[:]=[s/4,(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s]
    else:
        k=int(np.argmax(np.diag(R)))
        if k==0:
            s=2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]);q[:]=[(R[2,1]-R[1,2])/s,s/4,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s]
        elif k==1:
            s=2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2]);q[:]=[(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,s/4,(R[1,2]+R[2,1])/s]
        else:
            s=2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1]);q[:]=[(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,s/4]
    return q_normalize(q)


def target_attitude(r,v,mode):
    if mode=='inertial_x':return np.array([1.,0.,0.,0.]),np.zeros(3)
    h=np.cross(r,v); h/=np.linalg.norm(h)
    x=v/np.linalg.norm(v)
    if mode=='retrograde': x=-x
    y=np.cross(h,x);y/=np.linalg.norm(y)
    z=np.cross(x,y)
    R=np.column_stack((x,y,z))
    rate=np.linalg.norm(np.cross(r,v))/np.linalg.norm(r)**2
    # Retrograde with z reversed flips sign of demanded body-z angular rate.
    return q_from_dcm(R),np.array([0.,0.,rate])


def attitude_error_deg(q,r,v,mode):
    requested={'prograde':v/np.linalg.norm(v),'retrograde':-v/np.linalg.norm(v),'inertial_x':np.array([1.,0.,0.])}[mode]
    actual=rotate_BI(q,[1,0,0])
    return float(np.degrees(np.arccos(np.clip(np.dot(actual,requested),-1,1))))


@dataclass(frozen=True)
class AttitudeConfig:
    altitude_km: float=400.
    inclination_deg: float=51.6
    duration_minutes: float=35.
    samples: int=210
    dry_mass_kg: float=600.
    fuel_mass_kg: float=150.
    thrust_n: float=850.
    specific_impulse_s: float=315.
    burn_start_s: float=300.
    burn_duration_s: float=120.
    initial_misalignment_deg: float=25.
    target_mode: str='prograde'
    controller_enabled: bool=True
    max_control_torque_nm: float=.25
    kp_nm_per_rad: float=1.2
    kd_nms_per_rad: float=18.
    inertia_kg_m2: tuple=(120.,100.,80.)
    j2_enabled: bool=True
    gravity_gradient_enabled: bool=False

    def validate(self):
        vals=[self.altitude_km,self.inclination_deg,self.duration_minutes,self.dry_mass_kg,
              self.fuel_mass_kg,self.thrust_n,self.specific_impulse_s,self.burn_start_s,
              self.burn_duration_s,self.initial_misalignment_deg,self.max_control_torque_nm,
              self.kp_nm_per_rad,self.kd_nms_per_rad,*self.inertia_kg_m2]
        if not np.all(np.isfinite(vals)): raise ValueError('All parameters must be finite')
        if not 200<=self.altitude_km<=5000 or not 0<=self.inclination_deg<=180: raise ValueError('Unsupported initial orbit')
        if not 1<=self.duration_minutes<=180 or not 30<=self.samples<=800: raise ValueError('Duration or sample count out of bounds')
        if self.dry_mass_kg<=0 or self.fuel_mass_kg<0 or self.thrust_n<0 or self.specific_impulse_s<=0:raise ValueError('Invalid engine or mass')
        if self.burn_start_s<0 or self.burn_duration_s<0 or self.burn_start_s+self.burn_duration_s>self.duration_minutes*60+1e-9:raise ValueError('Burn must be fully contained in simulation')
        if not 0<=self.initial_misalignment_deg<=180 or self.target_mode not in ('prograde','retrograde','inertial_x'):raise ValueError('Invalid attitude target')
        if min(self.inertia_kg_m2)<=0 or self.max_control_torque_nm<0 or min(self.kp_nm_per_rad,self.kd_nms_per_rad)<0:raise ValueError('Invalid spacecraft inertia or controller gains')


def initial_state(cfg):
    rv=state_from_elements(RE+cfg.altitude_km,0.,cfg.inclination_deg)
    q0,omega_target=target_attitude(rv[:3],rv[3:],cfg.target_mode)
    delta=np.deg2rad(cfg.initial_misalignment_deg)/2
    qmis=np.array([np.cos(delta),0.,0.,np.sin(delta)])
    # Misalignment about body Z: body axis rotates away from target.
    q0=q_normalize(q_mul(q0,qmis))
    return np.r_[rv,q0,omega_target,cfg.dry_mass_kg+cfg.fuel_mass_kg]


def control_torque(y,cfg):
    if not cfg.controller_enabled or cfg.max_control_torque_nm==0:return np.zeros(3)
    q=q_normalize(y[6:10]);qd,wd=target_attitude(y[:3],y[3:6],cfg.target_mode)
    err=q_mul(q_conj(q),qd)
    s=1 if err[0]>=0 else -1
    torque=2*cfg.kp_nm_per_rad*s*err[1:]+cfg.kd_nms_per_rad*(wd-y[10:13])
    n=np.linalg.norm(torque)
    if n>cfg.max_control_torque_nm:torque*=cfg.max_control_torque_nm/n
    return torque


def gravity_gradient_torque(y,inertia):
    r=y[:3]; u=rotate_IB(y[6:10],r/np.linalg.norm(r))
    return 3*MU/np.linalg.norm(r)**3*np.cross(u,inertia*u)


def dynamics(t,y,cfg,powered):
    r=y[:3];v=y[3:6];q=y[6:10];w=y[10:13];m=y[13]
    a=-MU*r/np.linalg.norm(r)**3
    if cfg.j2_enabled:a+=j2_acceleration(r)
    fuel_available=m>cfg.dry_mass_kg+1e-7
    if powered and fuel_available and cfg.thrust_n>0:
        a=a+cfg.thrust_n/m/1000*rotate_BI(q,[1,0,0])
        mdot=-cfg.thrust_n/(cfg.specific_impulse_s*G0)
    else:mdot=0.
    inertia=np.asarray(cfg.inertia_kg_m2)
    tau=control_torque(y,cfg)
    if cfg.gravity_gradient_enabled:tau+=gravity_gradient_torque(y,inertia)
    wdot=(tau-np.cross(w,inertia*w))/inertia
    qdot=.5*q_mul(q,np.r_[0.,w])
    return np.r_[v,a,qdot,wdot,mdot]


def simulate_attitude(cfg:AttitudeConfig)->dict:
    cfg.validate()
    duration=cfg.duration_minutes*60.
    mdot=cfg.thrust_n/(cfg.specific_impulse_s*G0) if cfg.thrust_n>0 else 0
    exhaust_time=cfg.burn_start_s+cfg.fuel_mass_kg/mdot if mdot>0 else float('inf')
    active_end=min(cfg.burn_start_s+cfg.burn_duration_s,exhaust_time)
    breaks=sorted(set([0.,duration,cfg.burn_start_s,cfg.burn_start_s+cfg.burn_duration_s] +
                      ([exhaust_time] if cfg.burn_start_s<exhaust_time<cfg.burn_start_s+cfg.burn_duration_s else [])))
    y=initial_state(cfg); tparts=[]; pieces=[]
    for lo,hi in zip(breaks[:-1],breaks[1:]):
        if hi-lo<1e-9:continue
        firing=bool(cfg.burn_start_s<=.5*(lo+hi)<active_end and mdot>0)
        sol=solve_ivp(lambda t,s:dynamics(t,s,cfg,firing),[lo,hi],y,method='DOP853',
                      dense_output=True,rtol=2e-10,atol=1e-12,max_step=12.)
        if not sol.success:raise RuntimeError('Attitude propagator failed: '+sol.message)
        y=sol.y[:,-1].copy();y[6:10]=q_normalize(y[6:10]);
        if abs(hi-exhaust_time)<1e-7:y[13]=cfg.dry_mass_kg
        pieces.append((lo,hi,sol.sol))
    times=np.linspace(0,duration,cfg.samples+1)
    states=[]
    for t in times:
        item=next((p for p in pieces if t<=p[1]+1e-8),pieces[-1])
        yy=item[2](t).ravel();yy[6:10]=q_normalize(yy[6:10])
        yy[13]=max(cfg.dry_mass_kg,yy[13]);states.append(yy)
    states=np.asarray(states)
    fuel_used=cfg.dry_mass_kg+cfg.fuel_mass_kg-states[-1,13]
    e=np.array([attitude_error_deg(s[6:10],s[:3],s[3:6],cfg.target_mode) for s in states])
    torques=np.array([np.linalg.norm(control_torque(s,cfg)) for s in states])
    omega_norm=np.linalg.norm(states[:,10:13],axis=1)
    altitude=np.linalg.norm(states[:,:3],axis=1)-RE
    speeds=np.linalg.norm(states[:,3:6],axis=1)
    mass=states[:,13]
    cumulative=np.zeros(len(times))
    for j in range(1,len(times)):
        dt=times[j]-times[j-1]
        # Ideal engine ∫T/m dt from mass ratios (per output interval)
        if mass[j]<mass[j-1]-1e-8:cumulative[j]=cumulative[j-1]+cfg.specific_impulse_s*G0*np.log(mass[j-1]/mass[j])
        else:cumulative[j]=cumulative[j-1]
    consumed_at_end=fuel_used>=cfg.fuel_mass_kg-1e-6 and cfg.fuel_mass_kg>0 and active_end<cfg.burn_start_s+cfg.burn_duration_s-1e-8
    events=[]
    if cfg.burn_duration_s>0 and cfg.thrust_n>0 and cfg.fuel_mass_kg>0:
        for kind,t in [('ignition',cfg.burn_start_s),('fuel_exhaustion' if consumed_at_end else 'burn_cutoff',active_end)]:
            if 0<=t<=duration:
                i=min(len(times)-1,int(round(t/duration*cfg.samples)))
                events.append({'type':kind,'time_s':t,'position_km':states[i,:3].tolist()})
    attitude={'quaternion_BI_wxyz':states[:,6:10].tolist(),'angular_velocity_body_rad_s':states[:,10:13].tolist(),
              'pointing_error_deg':e.tolist(),'control_torque_nm':torques.tolist(),
              'body_x_inertial':[rotate_BI(s[6:10],[1,0,0]).tolist() for s in states],
              'max_quaternion_raw_norm_error':float(max(abs(np.linalg.norm(s[6:10])-1) for s in states)),
              'peak_pointing_error_deg':float(e.max()),'final_pointing_error_deg':float(e[-1]),
              'peak_body_rate_deg_s':float(np.rad2deg(omega_norm.max())),
              'max_control_torque_nm_observed':float(torques.max()),
              'mean_burn_pointing_error_deg':float(np.mean(e[(times>=cfg.burn_start_s)&(times<=active_end)])) if np.any((times>=cfg.burn_start_s)&(times<=active_end)) and active_end>cfg.burn_start_s else None}
    metrics={'elapsed_s':duration,'step_s':duration/cfg.samples,'samples':len(times),
             'min_altitude_km':float(altitude.min()),'max_altitude_km':float(altitude.max()),
             'initial_propellant_kg':cfg.fuel_mass_kg,'propellant_used_kg':float(fuel_used),
             'propellant_remaining_kg':float(mass[-1]-cfg.dry_mass_kg),
             'ideal_thrust_delta_v_m_s':float(cumulative[-1]),
             'fuel_exhausted':bool(consumed_at_end),'conservation_applicable':False,
             'max_relative_energy_drift':None,'max_relative_angular_momentum_drift':None}
    trajectory={'time_s':times.tolist(),'position_km':states[:,:3].tolist(),'velocity_km_s':states[:,3:6].tolist(),
                'altitude_km':altitude.tolist(),'speed_km_s':speeds.tolist(),
                'wet_mass_kg':mass.tolist(),'propellant_remaining_kg':(mass-cfg.dry_mass_kg).tolist(),
                'thrust_active':((times>=cfg.burn_start_s)&(times<active_end-1e-8)&(cfg.fuel_mass_kg>0)&(cfg.thrust_n>0)).tolist(),
                'cumulative_ideal_delta_v_m_s':cumulative.tolist()}
    return {'name':'Coupled orbit–attitude and body-fixed propulsion','input':{**cfg.__dict__,
            'burns':[{'start_s':cfg.burn_start_s,'duration_s':cfg.burn_duration_s,'thrust_n':cfg.thrust_n}]},
            'orbital_elements':elements_from_state(states[0,:6]).to_dict(),
            'final_orbital_elements':elements_from_state(states[-1,:6]).to_dict(),
            'trajectory':trajectory,'attitude':attitude,'metrics':metrics,'events':events,
            'model':{'name':'Rigid body + orbit with optional J2 and gravity gradient torque',
                     'reference_frame':'Idealized Earth-centered inertial Cartesian; body-to-inertial active quaternion, scalar first',
                     'integrator':'DOP853; separated at burn/fuel boundaries',
                     'force_terms':['Newtonian gravity']+(['J2'] if cfg.j2_enabled else [])+['body-fixed finite engine'],
                     'assumptions':['diagonal constant inertia','idealized saturated PD attitude control','no reaction-wheel momentum states',
                                    'no engine gimbal, thruster misalignment, sensor noise, actuator delay or third-body forces']},
            'warnings':['Research/education simulation only, not flight-qualified guidance. The attitude PD assumes direct bounded control torques.']}
