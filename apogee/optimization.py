"""APOGEE v0.6: event-based, constrained ideal-impulse phase search.

Finds a tangential departure impulse whose *propagated first opposite apsis*
meets a specified radius, under two-body or Earth J2 dynamics. Evaluates a
bounded grid of departure orbital phases for minimum two-impulse propellant.
This is a finite discrete search, NOT a continuous global optimizer or
finite-thrust mission optimization.
"""
from math import pi, sqrt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state
from .perturbations import ForceModel, acceleration
from .propulsion import rocket_equation_budget


def _event_transfer(state, raising: bool, j2: bool, record: bool = False):
    """First opposite apsis, located at zero radial velocity (not a sample)."""
    model=ForceModel(j2=j2,drag=False)
    def f(t,y):
        return np.r_[y[3:],acceleration(y[:3],y[3:],model)]
    def radial(t,y):
        return float(np.dot(y[:3],y[3:])/np.linalg.norm(y[:3]))
    radial.direction = -1 if raising else 1
    radial.terminal=True
    radius=float(np.linalg.norm(state[:3]))
    speed=float(np.linalg.norm(state[3:]))
    energy=speed**2/2-MU/radius
    if energy>=0:
        raise ValueError('Departure impulse produced an unbound orbit')
    semimajor=-MU/(2*energy)
    period=2*pi*sqrt(semimajor**3/MU)
    # Avoid counting the launch t=0 radial-zero as an apsis event.
    dt0=0.1
    initial=solve_ivp(f,(0,dt0),state,method='DOP853',rtol=2e-11,atol=1e-12,max_step=.1)
    if not initial.success: raise ValueError('Integration failed at departure')
    solved=solve_ivp(f,(dt0,1.2*period),initial.y[:,-1],events=radial,
                     method='DOP853',rtol=2e-10,atol=2e-11,max_step=100,
                     dense_output=record)
    if not solved.success or not len(solved.t_events[0]):
        raise ValueError('Opposite apsis not found within search horizon')
    event_time=float(solved.t_events[0][0])
    arrival=np.asarray(solved.y_events[0][0],float)
    if float(np.linalg.norm(arrival[:3]))<=RE:
        raise ValueError('Transfer intersects Earth')
    return event_time,arrival,(initial,solved) if record else None


def optimize_impulsive_transfer(start_altitude_km: float=400, target_altitude_km: float=700,
                                inclination_deg: float=51.6, dry_mass_kg:float=600,
                                propellant_kg:float=150, specific_impulse_s:float=315,
                                phase_candidates:int=8, j2_enabled:bool=True,
                                samples:int=360) -> dict:
    numbers=(start_altitude_km,target_altitude_km,inclination_deg,dry_mass_kg,propellant_kg,specific_impulse_s)
    if not np.isfinite(numbers).all() or not (180<=start_altitude_km<=20000 and
       180<=target_altitude_km<=20000 and abs(target_altitude_km-start_altitude_km)>=20 and
       0<=inclination_deg<=180 and dry_mass_kg>0 and propellant_kg>=0 and
       50<=specific_impulse_s<=5000 and phase_candidates in (1,4,8,12) and
       120<=samples<=1000):
        raise ValueError('Invalid optimizer inputs or target less than 20 km from starting altitude')
    r0=RE+start_altitude_km; target_r=RE+target_altitude_km
    vc=sqrt(MU/r0); raising=target_r>r0
    phases=np.linspace(0,360,phase_candidates,endpoint=False)
    trials=[]; best=None
    for phase in phases:
        initial=state_from_elements(r0,0,inclination_deg,true_anomaly_deg=float(phase))
        u=initial[3:]/np.linalg.norm(initial[3:])
        # Avoid near-parabolic or Earth-impacting candidate speeds.
        ideal=sqrt(MU*(2/r0-2/(r0+target_r)))-vc
        margin=max(0.012 if j2_enabled else 0.0005,abs(ideal)*0.25)
        bounds=(max(1e-6,ideal-margin),ideal+margin) if raising else (ideal-margin,min(-1e-6,ideal+margin))
        def run(dv):
            s=initial.copy();s[3:]+=dv*u
            t,arrival,_=_event_transfer(s,raising,j2_enabled)
            return float(np.linalg.norm(arrival[:3])-target_r),t,arrival,s
        try:
            ea,*_=run(bounds[0]);eb,*_=run(bounds[1])
            if ea*eb>0:
                raise ValueError('Target altitude was not bracketed for this departure phase')
            root=brentq(lambda d:run(d)[0],*bounds,xtol=2e-10,rtol=1e-12,maxiter=45)
            residual,t,arrival,departure=run(root)
            rhat=arrival[:3]/np.linalg.norm(arrival[:3]);
            # Match a circular velocity in the transfer orbit's instantaneous plane.
            h=np.cross(arrival[:3],arrival[3:]);h/=np.linalg.norm(h)
            tangent=np.cross(h,rhat)
            circular_velocity=tangent*sqrt(MU/np.linalg.norm(arrival[:3]))
            arrival_burn=circular_velocity-arrival[3:]
            dv2=float(np.linalg.norm(arrival_burn))*1000
            dv1=abs(root)*1000
            budget=rocket_equation_budget([dv1,dv2],dry_mass_kg,propellant_kg,specific_impulse_s)
            trial={'phase_deg':float(phase),'departure_delta_v_m_s':float(root*1000),
                'arrival_delta_v_m_s':dv2,'total_delta_v_m_s':float(dv1+dv2),
                'fuel_required_kg':float(budget['required_propellant_kg']),
                'target_radius_residual_m':float(residual*1000),
                'arrival_time_s':float(t),'feasible':bool(budget['feasible'])}
            trials.append(trial)
            if best is None or trial['fuel_required_kg']<best[0]['fuel_required_kg']:
                best=(trial,departure,arrival,budget)
        except (ValueError,RuntimeError,OverflowError) as exc:
            trials.append({'phase_deg':float(phase),'error':str(exc)})
    if best is None:
        raise ValueError('No departure phase produced a feasible apsis crossing; change orbit constraints')
    choice,departure,arrival,budget=best
    # Re-evaluate selected departure to sample an event-terminated path.
    t_end,final,integrations=_event_transfer(departure,raising,j2_enabled,record=True)
    _,second=integrations
    ts=np.linspace(0,t_end,samples+1)
    traj=[]
    for t in ts:
        # t[1] is always > 0.1 s within this bounded orbit/sample domain.
        traj.append(departure if t == 0 else second.sol(t))
    states=np.asarray(traj)
    speeds=np.linalg.norm(states[:,3:],axis=1)
    radii=np.linalg.norm(states[:,:3],axis=1)
    return {
        'name':'Event-targeted impulsive transfer · bounded phase optimization',
        'optimization':{'method':'bounded grid search over departure phase; Brent root of numerically propagated apsis radius',
            'phase_candidates':int(phase_candidates),'j2_enabled':bool(j2_enabled),
            'optimality_scope':'minimum ideal two-impulse fuel among sampled departure phases only; not globally optimal',
            'candidate_results':trials,'selected':choice,'fuel_budget':budget,
            'starting_altitude_km':start_altitude_km,'target_altitude_km':target_altitude_km,
            'target_apsis':'apoapsis' if raising else 'periapsis'},
        'model':{'name':'Earth two-body plus optional J2, event-located apsis, ideal impulses',
            'reference_frame':'ideal Earth-centered inertial axes with spin-aligned z; not TEME/ICRF',
            'integrator':'SciPy DOP853 adaptive with radial-velocity root events',
            'force_terms':['Newtonian Earth gravity']+(['Earth J2'] if j2_enabled else []),
            'assumptions':['two instantaneous ideal burns; no drag, finite thrust or third bodies',
                 'arrival burn matches local two-body circular speed (not a J2-stationary orbit)',
                 'departure phase candidates only; optimize is discrete, not globally certified']},
        'initial_state':{'r_km':departure[:3].tolist(),'v_km_s':departure[3:].tolist()},
        'orbital_elements':elements_from_state(departure).to_dict(),
        'spacecraft_mass_kg':dry_mass_kg+propellant_kg,
        'metrics':{'samples':len(ts),'elapsed_s':float(t_end),'step_s':float(t_end/samples),
            'min_altitude_km':float(radii.min()-RE),'max_altitude_km':float(radii.max()-RE),
            'conservation_applicable':False,'max_relative_energy_drift':None,
            'max_relative_angular_momentum_drift':None,
            'diagnostic_scope':'conservation not plotted for this impulsive optimization'},
        'trajectory':{'time_s':ts.tolist(),'position_km':states[:,:3].tolist(),
            'velocity_km_s':states[:,3:].tolist(),'altitude_km':(radii-RE).tolist(),
            'speed_km_s':speeds.tolist(), 'relative_energy_drift':[],
            'relative_angular_momentum_drift':[]},
        'events':[{'type':'departure_impulse','time_s':0,'delta_v_m_s':choice['departure_delta_v_m_s'],
            'position_km':departure[:3].tolist()},
            {'type':'opposite_apsis_arrival','time_s':float(t_end),
             'delta_v_m_s':choice['arrival_delta_v_m_s'],
             'position_km':final[:3].tolist()}],
        'warnings':['Minimum propellant is reported among a finite set of departure phases, not a proven global optimum.',
            'Both maneuvers are ideal impulses; finite-burn losses, inclination targeting and drag are omitted.']+
            ([] if budget['feasible'] else ['INSUFFICIENT PROPELLANT: best candidate exceeds available fuel.'])
    }
