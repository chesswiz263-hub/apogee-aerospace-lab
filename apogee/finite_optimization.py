"""Bounded two-burn *finite thrust* orbital transfer search (v0.7).

This is NOT a continuous global optimizer. Prograde/retrograde local-steering
burns, constant thrust and Isp, and first opposite apsis detection define a
restricted design family. Burn #2 begins at the first opposite apsis, so the
result may retain finite-burn circularization error (reported explicitly).

Positions km, velocities km/s, mass kg, times s. Models are idealized ECI with
optional Earth J2, no drag or attitude transitions.
"""
from math import pi, sqrt
from dataclasses import replace
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, minimize_scalar

from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state
from .perturbations import ForceModel, acceleration
from .propulsion import Burn, steering_unit, G0_M_S2


def _ode(model: ForceModel, burn: Burn | None):
    def fun(t, y):
        r, v, mass = y[:3], y[3:6], float(y[6])
        a = acceleration(r, v, replace(model, mass_kg=mass))
        mdot = 0.
        if burn is not None:
            a = a + (burn.thrust_n/(1000*mass))*steering_unit(y, burn.direction)
            mdot = -burn.mass_flow_kg_s
        return np.r_[v, a, mdot]
    return fun


def _propagate(y, span, model, burn=None, sample_times=None, event=None):
    t0, t1 = map(float, span)
    if t1 <= t0: raise ValueError('Empty propagation interval')
    out = solve_ivp(_ode(model, burn), (t0,t1), np.asarray(y,float), method='DOP853',
                    rtol=3e-10, atol=2e-11, max_step=10 if burn else 75,
                    t_eval=sample_times, events=event, dense_output=sample_times is None)
    if not out.success or not np.all(np.isfinite(out.y)):
        raise ValueError('Finite-burn propagation failed')
    if np.min(np.linalg.norm(out.y[:3,:],axis=0)) <= RE:
        raise ValueError('Trajectory intersects Earth')
    return out


def _one_candidate(phase, args, samples=180, record=False):
    start, target, inc, dry, fuel, isp, thrust, j2 = args
    r0, r_target = RE+start, RE+target
    raising = target > start
    direction = 'prograde' if raising else 'retrograde'
    model = ForceModel(j2=j2, drag=False)
    state0 = state_from_elements(r0, 0, inc, true_anomaly_deg=float(phase))
    y0 = np.r_[state0, dry+fuel]
    mdot = thrust/(isp*G0_M_S2)
    if mdot <= 0 or fuel <= 0: raise ValueError('No propellant available for finite-thrust transfer')
    # Do not permit a virtual trajectory to burn fuel that is not onboard.
    fuel_duration = (fuel/mdot)*(1-1e-7)
    max_burn = min(5000.,fuel_duration,0.25*2*pi*sqrt(r0**3/MU))
    if max_burn < 0.5: raise ValueError('Insufficient fuel to sustain departure burn')

    radial_event_direction = -1 if raising else 1
    def radial_event(t,y): return float(np.dot(y[:3],y[3:6])/np.linalg.norm(y[:3]))
    radial_event.terminal=True
    radial_event.direction=radial_event_direction

    def departure(duration, keep=False):
        burn=Burn(0,float(duration),thrust,isp,direction,name='Departure')
        powered = _propagate(y0,(0,duration),model,burn)
        end=powered.y[:,-1]
        rn = np.linalg.norm(end[:3]); vn=np.linalg.norm(end[3:6]); energy=vn**2/2-MU/rn
        if energy >= 0: raise ValueError('Unbound finite-thrust candidate')
        period=2*pi*sqrt((-MU/(2*energy))**3/MU)
        # Opposite apsis must occur after cutoff; rising/falling radial velocity
        # should match selected transfer orientation.
        rdot = np.dot(end[:3],end[3:6])/rn
        if (rdot <= 1e-8 if raising else rdot >= -1e-8):
            raise ValueError('Departure cutoff did not precede opposite apsis')
        coast = _propagate(end,(duration,duration+max(1.2*period,3600)),model,
                           event=radial_event)
        if not len(coast.t_events[0]): raise ValueError('No opposite apsis found')
        event_t=float(coast.t_events[0][0]); event_y=np.asarray(coast.y_events[0][0],float)
        radius=float(np.linalg.norm(event_y[:3]))
        return radius-r_target,event_t,event_y,(powered,coast) if keep else None

    # An initial root guess from ideal Hohmann delta-v and rocket equation.
    vc=sqrt(MU/r0)
    dv1=abs(sqrt(MU*(2/r0-2/(r0+r_target)))-vc)*1000
    guess=(dry+fuel)*(1-np.exp(-dv1/(isp*G0_M_S2)))/mdot
    lo=max(0.001,guess*.45)
    hi=min(max_burn, max(guess*1.35,guess+12))
    if lo >= hi: raise ValueError('Fuel constraint prevents requested transfer')
    # Avoid extremely wide unphysical brackets where apogee event disappears.
    vlo=departure(lo)[0]; vhi=departure(hi)[0]
    if vlo*vhi>0:
        # Expand or contract deliberately, within remaining fuel and bounded time.
        for fac in (1.5, 2., 3., 4.):
            trial_hi=min(max_burn,max(hi,guess*fac))
            if trial_hi <= hi+1e-6: continue
            try:
                vhi=departure(trial_hi)[0]; hi=trial_hi
            except ValueError:
                continue
            if vlo*vhi<=0: break
        if vlo*vhi>0:
            raise ValueError('Target not bracketed within available fuel / engine constraints')
    d1=float(brentq(lambda x: departure(x)[0],lo,hi,xtol=1e-5,rtol=1e-10,maxiter=38))
    residual,t_apsis,at_apsis,_=departure(d1)

    # Second prograde/retrograde burn begins at opposite apsis. Optimize its
    # *finite duration* to minimize post-burn two-body orbital eccentricity.
    remaining=float(at_apsis[6]-dry)
    d2_limit=min(600.,max(0,remaining/mdot)*(1-1e-7), max(5., t_apsis*.20))
    if d2_limit<=0.1: raise ValueError('No fuel remains for arrival maneuver')

    def circularization(duration, keep=False):
        if duration <= 0: return float(elements_from_state(at_apsis[:6]).eccentricity),at_apsis,None
        b=Burn(t_apsis,float(duration),thrust,isp,direction,name='Arrival')
        out=_propagate(at_apsis,(t_apsis,t_apsis+duration),model,b)
        final=out.y[:,-1]
        ecc=float(elements_from_state(final[:6]).eccentricity)
        return ecc,final,out if keep else None

    optimum=minimize_scalar(lambda d:circularization(d)[0],bounds=(1e-4,d2_limit),
                            method='bounded',options={'xatol':1e-3,'maxiter':38})
    d2=float(optimum.x)
    ec, final, _=circularization(d2)
    # If optimum hit end-of-fuel limit, report true feasibility/shortfall limits.
    used=float(y0[6]-final[6])
    dv_ideal=float(isp*G0_M_S2*np.log(y0[6]/final[6]))
    el=elements_from_state(final[:6]).to_dict()
    trial={'phase_deg':float(phase),'departure_duration_s':d1,'arrival_duration_s':d2,
           'departure_cutoff_s':d1,'apsis_event_s':float(t_apsis),
           'propellant_used_kg':used,'propellant_remaining_kg':float(final[6]-dry),
           'delivered_ideal_delta_v_m_s':dv_ideal,
           'target_apsis_residual_m':float(residual*1000),
           'postburn_eccentricity':ec,
           'postburn_semimajor_axis_km':float(el['semi_major_axis_km']),
           'postburn_periapsis_altitude_km':float(el['periapsis_altitude_km']),
           'postburn_apoapsis_altitude_km':float(el['apoapsis_altitude_km']),
           'arrival_radius_error_km':float(np.linalg.norm(final[:3])-r_target),
           'fuel_feasible':bool(final[6]>=dry-1e-8),
           'circularization_fuel_limited':bool(d2_limit-d2<.05)}
    if not record: return trial,None
    _,_,_,pieces=departure(d1,True)
    _,_,last=circularization(d2,True)
    # Re-sample segments respecting ignition/cutoff/apsis boundaries.
    times=np.unique(np.r_[np.linspace(0,d1,max(12,samples//5)),
                         np.linspace(d1,t_apsis,max(70,3*samples//5)),
                         np.linspace(t_apsis,t_apsis+d2,max(12,samples//5)),
                         np.linspace(t_apsis+d2,t_apsis+d2+1000,48)])
    seg1=_propagate(y0,(0,d1),model,Burn(0,d1,thrust,isp,direction),sample_times=times[(times>=0)&(times<=d1)])
    segment1_last=seg1.y[:,-1]
    seg2=_propagate(segment1_last,(d1,t_apsis),model,sample_times=times[(times>=d1)&(times<=t_apsis)])
    segment2_last=seg2.y[:,-1]
    seg3=_propagate(segment2_last,(t_apsis,t_apsis+d2),model,
                    Burn(t_apsis,d2,thrust,isp,direction),sample_times=times[(times>=t_apsis)&(times<=t_apsis+d2)])
    seg4=_propagate(seg3.y[:,-1],(t_apsis+d2,t_apsis+d2+1000),model,
                    sample_times=times[(times>=t_apsis+d2)&(times<=t_apsis+d2+1000)])
    ts=np.r_[seg1.t,seg2.t[1:],seg3.t[1:],seg4.t[1:]]
    states=np.vstack([seg1.y.T,seg2.y.T[1:],seg3.y.T[1:],seg4.y.T[1:]])
    return trial,(ts,states,at_apsis)


def optimize_finite_transfer(start_altitude_km=400.,target_altitude_km=700.,
        inclination_deg=51.6,dry_mass_kg=600.,propellant_kg=150.,specific_impulse_s=315.,
        thrust_n=850.,phase_candidates=4,j2_enabled=False,samples=280):
    vals=np.asarray([start_altitude_km,target_altitude_km,inclination_deg,dry_mass_kg,
                     propellant_kg,specific_impulse_s,thrust_n],float)
    if not np.isfinite(vals).all() or not (200<=start_altitude_km<=3000 and
        200<=target_altitude_km<=3000 and 30<=abs(target_altitude_km-start_altitude_km)<=1000
        and 0<=inclination_deg<=180 and 1<=dry_mass_kg<=100000
        and 0<propellant_kg<=100000 and 50<=specific_impulse_s<=5000
        and 10<=thrust_n<=1000000 and phase_candidates in (1,4,8)
        and 120<=samples<=800):
        raise ValueError('Finite optimizer input outside validated prototype limits')
    args=(start_altitude_km,target_altitude_km,inclination_deg,dry_mass_kg,
          propellant_kg,specific_impulse_s,thrust_n,j2_enabled)
    candidates=[]; selection=None
    for phase in np.linspace(0,360,phase_candidates,endpoint=False):
        try:
            trial,_=_one_candidate(float(phase),args)
            candidates.append(trial)
            # Only select minimal actual fuel from trials that close the orbit to
            # a stated tolerance; otherwise best achievable is reported as such.
            key=(trial['postburn_eccentricity']>0.005,
                 trial['propellant_used_kg'],trial['postburn_eccentricity'])
            if selection is None or key<selection[0]: selection=(key,float(phase))
        except (ValueError,RuntimeError,OverflowError) as exc:
            candidates.append({'phase_deg':float(phase),'error':str(exc)})
    if selection is None:
        raise ValueError('No feasible two-burn finite-thrust trajectory found with available fuel and thrust')
    choice,path=_one_candidate(selection[1],args,samples,record=True)
    # Compare a bounded set of engine-thrust strategies at the *selected phase*
    # only. The candidate durations are solved afresh, not rescaled impulses.
    thrust_strategies=[]
    for ratio in (0.5,1.,2.):
        proposed_thrust=thrust_n*ratio
        if proposed_thrust>1000000 or proposed_thrust<10:
            thrust_strategies.append({'relative_thrust':ratio,'error':'Thrust outside bounds'})
            continue
        try:
            strategy,_=_one_candidate(selection[1],args[:-2]+(proposed_thrust,j2_enabled))
            thrust_strategies.append({'relative_thrust':ratio,'thrust_n':proposed_thrust,
                'fuel_used_kg':strategy['propellant_used_kg'],
                'postburn_eccentricity':strategy['postburn_eccentricity'],
                'departure_duration_s':strategy['departure_duration_s'],
                'arrival_duration_s':strategy['arrival_duration_s']})
        except ValueError as exc:
            thrust_strategies.append({'relative_thrust':ratio,'thrust_n':proposed_thrust,'error':str(exc)})
    times,ys,ap=path
    pos=ys[:,:3];vel=ys[:,3:6]; radius=np.linalg.norm(pos,axis=1)
    fuel=ys[:,6]-dry_mass_kg
    result={
        'name':'Constrained finite-thrust transfer · departure and arrival burn',
        'spacecraft_mass_kg':dry_mass_kg+propellant_kg,
        'input':{'burns':[{'start_s':0,'duration_s':choice['departure_duration_s']},
                          {'start_s':choice['apsis_event_s'],'duration_s':choice['arrival_duration_s']}]},
        'burn_reports':[{'name':'Departure','direction':'prograde' if target_altitude_km>start_altitude_km else 'retrograde',
                         'fuel_used_kg':thrust_n*choice['departure_duration_s']/(specific_impulse_s*G0_M_S2),
                         'delivered_ideal_delta_v_m_s':None},
                        {'name':'Arrival','direction':'prograde' if target_altitude_km>start_altitude_km else 'retrograde',
                         'fuel_used_kg':thrust_n*choice['arrival_duration_s']/(specific_impulse_s*G0_M_S2),
                         'delivered_ideal_delta_v_m_s':None}],
        'orbital_elements':elements_from_state(ys[0,:6]).to_dict(),
        'finite_optimization':{'selected':choice,'candidate_results':candidates,
             'phase_candidates':phase_candidates,'j2_enabled':j2_enabled,
             'thrust_strategy_comparison':thrust_strategies,
             'strategy_comparison_scope':'half / nominal / double thrust at SELECTED phase only; no global re-optimization',
             'method':'Brent root for departure engine duration; bounded 1-D eccentricity minimization for arrival duration; finite phase grid',
             'optimality_scope':'best fuel among sampled phases with postburn eccentricity <=0.005 when possible; otherwise best achievable sampled trial',
             'engine_thrust_n':thrust_n,'specific_impulse_s':specific_impulse_s,
             'initial_propellant_kg':propellant_kg,'dry_mass_kg':dry_mass_kg},
        'model':{'name':'Adaptive finite thrust and mass depletion with optional Earth J2',
                 'reference_frame':'ideal Earth-centered inertial axes, not TEME or ICRF',
                 'integrator':'SciPy DOP853; apsis event + two finite powered intervals',
                 'force_terms':['Newtonian Earth gravity']+(['Earth J2'] if j2_enabled else [])+['constant-thrust local steering'],
                 'assumptions':['Fixed engine thrust and Isp; both burns prograde or retrograde',
                     'Arrival burn ignites at opposite apsis; postburn circularity may be imperfect',
                     'No atmospheric drag, finite attitude dynamics, third bodies or global optimality guarantee']},
        'metrics':{'samples':len(times),'elapsed_s':float(times[-1]),
                   'step_s':float(np.median(np.diff(times))),
                   'min_altitude_km':float(radius.min()-RE),'max_altitude_km':float(radius.max()-RE),
                   'dry_mass_kg':dry_mass_kg,'initial_propellant_kg':propellant_kg,
                   'propellant_used_kg':choice['propellant_used_kg'],
                   'propellant_remaining_kg':choice['propellant_remaining_kg'],
                   'ideal_thrust_delta_v_m_s':choice['delivered_ideal_delta_v_m_s'],
                   'conservation_applicable':False,'max_relative_energy_drift':None,
                   'max_relative_angular_momentum_drift':None},
        'trajectory':{'time_s':times.tolist(),'position_km':pos.tolist(),
                      'velocity_km_s':vel.tolist(),'speed_km_s':np.linalg.norm(vel,axis=1).tolist(),
                      'altitude_km':(radius-RE).tolist(),'wet_mass_kg':ys[:,6].tolist(),
                      'propellant_remaining_kg':fuel.tolist(),
                      'thrust_active':((times<=choice['departure_duration_s']+1e-7)|((times>=choice['apsis_event_s']-1e-7)&(times<choice['apsis_event_s']+choice['arrival_duration_s']-1e-7))).tolist(),
                      'cumulative_ideal_delta_v_m_s':(specific_impulse_s*G0_M_S2*np.log(ys[0,6]/ys[:,6])).tolist(),
                      'relative_energy_drift':[],'relative_angular_momentum_drift':[]},
        'events':[{'type':'departure_ignition','time_s':0,'position_km':pos[0].tolist()},
                 {'type':'departure_cutoff','time_s':choice['departure_duration_s'],
                  'position_km':pos[np.searchsorted(times,choice['departure_duration_s'])].tolist()},
                 {'type':'opposite_apsis_arrival_ignition','time_s':choice['apsis_event_s'],
                  'position_km':ap[:3].tolist()},
                 {'type':'arrival_cutoff','time_s':choice['apsis_event_s']+choice['arrival_duration_s'],
                  'position_km':ys[np.searchsorted(times,choice['apsis_event_s']+choice['arrival_duration_s']),:3].tolist()}],
        'warnings':['Finite phase grid + two scalar optimizations: no global fuel optimality certificate.',
            'Arrival burn starts at apsis; final eccentricity and arrival-radius mismatch are reported, not asserted to be zero.']}
    if choice['postburn_eccentricity']>0.005:
        result['warnings'].append('POSTBURN CIRCULARIZATION OUTSIDE 0.005 ECCENTRICITY TOLERANCE.')
    if choice['circularization_fuel_limited']:
        result['warnings'].append('Arrival optimization may be limited by available fuel.')
    return result
