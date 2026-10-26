"""APOGEE v0.8: simulated position-only EKF and bounded impulsive feedback.

The reference and simulated truth share an idealized inertial frame and an epoch.
The controller sees measurements and EKF state only, never the true state.
All burns are commanded *ideal instantaneous* velocity increments with exact
execution; this is not a flight-qualified GN&C, finite thrust, or rendezvous tool.

Cartesian state [km, km/s], seconds, and covariance with matching mixed units.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.integrate import solve_ivp

from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state
from .perturbations import ForceModel, acceleration, j2_acceleration


@dataclass(frozen=True)
class GuidanceConfig:
    altitude_km: float = 400.
    inclination_deg: float = 51.6
    duration_minutes: float = 90.
    measurement_interval_s: int = 90
    measurement_sigma_m: float = 30.
    initial_position_error_m: float = 600.
    initial_velocity_error_m_s: float = .40
    max_total_delta_v_m_s: float = 2.0
    guidance_enabled: bool = True
    corrections: int = 3
    seed: int = 42
    j2_enabled: bool = False

    def validate(self):
        v=np.array([self.altitude_km,self.inclination_deg,self.duration_minutes,
                    self.measurement_sigma_m,self.initial_position_error_m,
                    self.initial_velocity_error_m_s,self.max_total_delta_v_m_s],float)
        if not np.isfinite(v).all() or not (200<=self.altitude_km<=2000
            and 0<=self.inclination_deg<=180 and 20<=self.duration_minutes<=180
            and self.measurement_interval_s in (30,60,90,120,180,300)
            and 0<self.measurement_sigma_m<=1000
            and 0<=self.initial_position_error_m<=5000
            and 0<=self.initial_velocity_error_m_s<=5
            and 0<=self.max_total_delta_v_m_s<=30
            and self.corrections in (0,1,2,3,4)
            and isinstance(self.seed,int) and 0<=self.seed<=2147483647):
            raise ValueError('Guidance input outside bounded v0.8 demonstration limits')
        if not isinstance(self.guidance_enabled,bool) or not isinstance(self.j2_enabled,bool):
            raise ValueError('Mode flags must be boolean')


def _dynamics(t, x, model):
    return np.r_[x[3:6],acceleration(x[:3],x[3:6],model)]


def _variational(t, augmented, model):
    x=augmented[:6]
    r=x[:3]; rn=float(np.linalg.norm(r))
    # Analytic Newtonian gravity gradient, km/s^2 per km.
    tidal=-MU*(np.eye(3)/rn**3 - 3*np.outer(r,r)/rn**5)
    if model.j2:
        # Symmetric finite difference of just the J2 term; 20 m displacement.
        eps=.02
        for k in range(3):
            direction=np.zeros(3);direction[k]=eps
            tidal[:,k]+=(j2_acceleration(r+direction)-j2_acceleration(r-direction))/(2*eps)
    A=np.zeros((6,6))
    A[:3,3:]=np.eye(3)
    A[3:,:3]=tidal
    phi=augmented[6:].reshape((6,6))
    return np.r_[_dynamics(t,x,model), (A@phi).reshape(-1)]


def _flow(x, span, model, variational=False, atol=1e-11):
    t0,t1=map(float,span)
    if t1<=t0: raise ValueError('Nonpositive guidance propagation interval')
    y=np.r_[x,np.eye(6).reshape(-1)] if variational else np.asarray(x,float)
    fun=(lambda t,s:_variational(t,s,model)) if variational else (lambda t,s:_dynamics(t,s,model))
    result=solve_ivp(fun,(t0,t1),y,method='DOP853',rtol=3e-10,
                     atol=atol,max_step=90.)
    if not result.success or not np.isfinite(result.y).all():
        raise ValueError('Guidance integration failed')
    last=result.y[:,-1]
    if np.linalg.norm(last[:3])<=RE: raise ValueError('Trajectory intersects Earth')
    if variational:return last[:6],last[6:].reshape((6,6))
    return last


def ekf_position_update(prior, P, measurement_km, sigma_km):
    """Joseph-stabilized additive EKF update for a 3-axis inertial position fix."""
    H=np.zeros((3,6));H[:,:3]=np.eye(3)
    R=np.eye(3)*sigma_km**2
    innovation=np.asarray(measurement_km)-H@prior
    S=H@P@H.T+R
    K=np.linalg.solve(S,(P@H.T).T).T
    posterior=prior+K@innovation
    J=np.eye(6)-K@H
    updated=J@P@J.T+K@R@K.T
    updated=(updated+updated.T)/2
    if np.linalg.eigvalsh(updated).min() < -1e-13:
        raise ValueError('EKF covariance numerically indefinite')
    return posterior,updated,innovation,float(innovation@np.linalg.solve(S,innovation))


def target_velocity_impulse(estimate, ref_terminal, now_s, end_s, model, max_dv_m_s):
    """Single-shooting linearized feedback targeting at end epoch.

    Uses terminal 3D *position* and a modest terminal velocity penalty. It is a
    bounded, one-step least-squares correction, not an optimization certificate.
    """
    predicted, phi=_flow(estimate,(now_s,end_s),model,variational=True)
    # Position in km, velocity in km/s; 120 seconds brings the latter to a
    # comparable residual length for the regularized constrained solution.
    W=np.diag([1.,1.,1.,120.,120.,120.])
    B=W@phi[:,3:6]
    rhs=W@(ref_terminal-predicted)
    damp=1e-5 * max(1.0,float(np.linalg.norm(B,ord=2)))
    delta=np.linalg.solve(B.T@B+damp*damp*np.eye(3),B.T@rhs)
    requested=float(np.linalg.norm(delta)*1000)
    if not np.isfinite(requested):raise ValueError('Targeting solution not finite')
    if requested>max_dv_m_s and requested>0:
        delta*=max_dv_m_s/requested
    return delta,requested, float(np.linalg.norm(rhs)), phi


def simulate_guidance(config: GuidanceConfig | None = None, **kwargs):
    if config is None: config=GuidanceConfig(**kwargs)
    config.validate()
    model=ForceModel(j2=config.j2_enabled,drag=False)
    t_end=float(config.duration_minutes*60)
    # Only grid-aligned measurements; a short final step is appended for an
    # exact requested end time (there is no extra sensor at that final epoch).
    times=list(np.arange(0,t_end,config.measurement_interval_s,dtype=float))+[t_end]
    times=np.asarray(sorted(set(times)),float)
    reference0=np.asarray(state_from_elements(RE+config.altitude_km,0.,config.inclination_deg),float)
    radial=reference0[:3]/np.linalg.norm(reference0[:3])
    normal=np.cross(reference0[:3],reference0[3:]);normal/=np.linalg.norm(normal)
    along=np.cross(normal,radial)
    err0=np.r_[radial*(config.initial_position_error_m/1000.),
               along*(config.initial_velocity_error_m_s/1000.)]
    truth=reference0+err0
    estimate=reference0.copy()       # Prior has no access to injected actual error.
    baseline=truth.copy()
    reference=reference0.copy()
    sigma_prior_position_km=max(config.initial_position_error_m*2,config.measurement_sigma_m*3,100)/1000.
    sigma_prior_velocity_km_s=max(config.initial_velocity_error_m_s*2,.2)/1000.
    P=np.diag([sigma_prior_position_km**2]*3+[sigma_prior_velocity_km_s**2]*3)
    rng=np.random.default_rng(config.seed)
    measurement_sigma_km=config.measurement_sigma_m/1000.
    history_truth=[truth.copy()]
    history_estimate=[estimate.copy()]
    history_ref=[reference.copy()]
    history_baseline=[baseline.copy()]
    sigmas=[float(np.sqrt(np.trace(P[:3,:3]))*1000)]
    innovations=[None]
    nis=[None]
    events=[]
    used_dv_m_s=0.
    # Schedule corrections evenly through the middle 20%–70% of the simulation;
    # the final segment remains for checking the effect of the last correction.
    possible_indices=np.flatnonzero((times>=.2*t_end)&(times<=.7*t_end))
    chosen=[]
    if config.corrections and len(possible_indices):
        candidate=np.linspace(0,len(possible_indices)-1,config.corrections)
        chosen=sorted(set(int(possible_indices[int(round(k))]) for k in candidate))
    # Reference terminal position is produced independently of simulated truth.
    target=_flow(reference0,(0,t_end),model)

    for k in range(1,len(times)):
        t0,t1=float(times[k-1]),float(times[k])
        truth=_flow(truth,(t0,t1),model)
        baseline=_flow(baseline,(t0,t1),model)
        reference=_flow(reference,(t0,t1),model)
        estimate,phi=_flow(estimate,(t0,t1),model,variational=True)
        P=phi@P@phi.T
        P=(P+P.T)/2
        if k<len(times)-1 or np.isclose((t1/config.measurement_interval_s)%1,0,atol=1e-9):
            measurement=truth[:3]+rng.normal(0,measurement_sigma_km,3)
            estimate,P,innovation,stat=ekf_position_update(estimate,P,measurement,measurement_sigma_km)
            innovations.append(float(np.linalg.norm(innovation)*1000))
            nis.append(stat)
        else:
            innovations.append(None);nis.append(None)
        if config.guidance_enabled and k in chosen and used_dv_m_s<config.max_total_delta_v_m_s-1e-12:
            available=config.max_total_delta_v_m_s-used_dv_m_s
            command,asked,before,_=target_velocity_impulse(estimate,target,t1,t_end,model,available)
            dv=float(np.linalg.norm(command)*1000)
            if dv>1e-6:
                truth[3:]+=command
                estimate[3:]+=command  # assume perfect execution; covariance unaffected
                used_dv_m_s+=dv
                events.append({'time_s':t1,'type':'guidance_burn','name':f'Correction {len(events)+1}',
                               'delta_v_m_s':dv,'unclipped_dv_m_s':asked,
                               'command_vector_m_s':(command*1000).tolist(),
                               'terminal_weighted_residual_before':before,
                               'remaining_budget_m_s':max(0,config.max_total_delta_v_m_s-used_dv_m_s)})
        if not (np.isfinite(estimate).all() and np.isfinite(P).all()):
            raise ValueError('Navigation filter diverged')
        history_truth.append(truth.copy())
        history_ref.append(reference.copy())
        history_baseline.append(baseline.copy())
        history_estimate.append(estimate.copy())
        sigmas.append(float(np.sqrt(max(0,np.trace(P[:3,:3])))*1000))

    traj=np.array(history_truth);ref=np.array(history_ref)
    bg=np.array(history_baseline);est=np.array(history_estimate)
    position_error_m=np.linalg.norm(traj[:,:3]-ref[:,:3],axis=1)*1000
    baseline_position_error_m=np.linalg.norm(bg[:,:3]-ref[:,:3],axis=1)*1000
    estimate_position_error_m=np.linalg.norm(est[:,:3]-traj[:,:3],axis=1)*1000
    pos=np.linalg.norm(traj[:,:3],axis=1)
    speed=np.linalg.norm(traj[:,3:],axis=1)
    metrics={'final_guided_position_error_m':float(position_error_m[-1]),
             'final_unguided_position_error_m':float(baseline_position_error_m[-1]),
             'final_estimation_position_error_m':float(estimate_position_error_m[-1]),
             'final_guided_velocity_error_m_s':float(np.linalg.norm(traj[-1,3:]-ref[-1,3:])*1000),
             'final_unguided_velocity_error_m_s':float(np.linalg.norm(bg[-1,3:]-ref[-1,3:])*1000),
             'improvement_pct':float(100*(1-position_error_m[-1]/max(baseline_position_error_m[-1],1e-12))),
             'total_guidance_delta_v_m_s':float(used_dv_m_s),
             'correction_count':len(events), 'measurement_count':int(sum(x is not None for x in nis)),
             'final_estimated_position_1sigma_rss_m':float(sigmas[-1]),
             'final_position_covariance_min_eigenvalue_km2':float(np.linalg.eigvalsh(P[:3,:3]).min()),
             'samples':len(times),'elapsed_s':t_end,'step_s':float(np.median(np.diff(times))),
             'min_altitude_km':float(pos.min()-RE),'max_altitude_km':float(pos.max()-RE),
             'conservation_applicable':False,'max_relative_energy_drift':None,
             'max_relative_angular_momentum_drift':None}
    return {'name':'Closed-loop navigation · simulated position-only EKF',
            'input':dict(config.__dict__),
            'spacecraft_mass_kg':750.,
            'orbital_elements':elements_from_state(reference0).to_dict(),
            'model':{'name':'Estimated-state EKF with bounded impulsive position-targeting corrections',
                     'reference_frame':'shared idealized Earth-centered inertial Cartesian axes; not TEME or ICRF',
                     'integrator':'SciPy DOP853 + simultaneously integrated 6x6 variational STM',
                     'force_terms':['Newtonian Earth gravity']+(['Earth J2'] if model.j2 else []),
                     'assumptions':['Synthetic isotropic Cartesian position fixes; fixed measurement noise; no sensor biases',
                                    'Only injected deterministic initial navigation error; no stochastic truth process noise',
                                    'Perfectly executed ideal instantaneous velocity impulses, no thruster time, attitude errors, or burn uncertainty',
                                    'Shared reference state and epoch; no external ephemerides']},
            'guidance':{'time_s':times.tolist(),
                        'guided_position_error_m':position_error_m.tolist(),
                        'unguided_position_error_m':baseline_position_error_m.tolist(),
                        'estimate_position_error_m':estimate_position_error_m.tolist(),
                        'estimate_position_1sigma_rss_m':sigmas,
                        'innovation_norm_m':innovations,
                        'normalized_innovation_squared':nis,
                        'final_estimate_6_state_km_units':est[-1].tolist(),
                        'final_covariance_6x6_km_units':P.tolist(),
                        'reference_position_km':ref[:,:3].tolist(),
                        'baseline_position_km':bg[:,:3].tolist(),
                        'position_observation_model':'synthetic true 3-axis ECI position plus seeded Gaussian 1-sigma measurement noise',
                        'feedback_target':'reference terminal state, one-step linearized bounded position+velocity weighted least-squares'},
            'metrics':metrics,
            'trajectory':{'time_s':times.tolist(),'position_km':traj[:,:3].tolist(),
                          'velocity_km_s':traj[:,3:].tolist(),'altitude_km':(pos-RE).tolist(),
                          'speed_km_s':speed.tolist(),
                          'relative_energy_drift':[], 'relative_angular_momentum_drift':[]},
            'events':events,
            'warnings':['Educational synthetic sensor and navigation demonstration. Never use these results to command an actual spacecraft.',
                        'Controller targets an idealized end-state with instantaneous, perfectly executed burns; it does not guarantee rendezvous, docking, collision avoidance or flight safety.',
                        'Filter covariance conditions on the assumed dynamics and measurement statistics; it is not observed or validated mission navigation accuracy.']}
