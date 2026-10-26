"""Numerically solve an ideal tangent-impulse apsis target.

Scope: two-body, circular starting orbit, impulse at t=0 at an apsis,
first opposite apsis at half the transfer period. NOT finite-burn targeting.
"""
from math import pi, sqrt
import numpy as np

from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state, integrate_orbit
from .propulsion import rocket_equation_budget


def solve_apsis_target(start_altitude_km: float = 400,
                       target_altitude_km: float = 700,
                       inclination_deg: float = 28.5,
                       dry_mass_kg: float = 600,
                       propellant_kg: float = 150,
                       specific_impulse_s: float = 315,
                       samples: int = 500) -> dict:
    """Bisection of post-burn osculating apsis radius (not a closed-form dv lookup)."""
    vals=(start_altitude_km, target_altitude_km, inclination_deg,
          dry_mass_kg, propellant_kg, specific_impulse_s)
    if not all(np.isfinite(vals)) or not (160<=start_altitude_km<=50000 and
        160<=target_altitude_km<=50000 and 0<=inclination_deg<=180 and
        dry_mass_kg>0 and propellant_kg>=0 and 50<=specific_impulse_s<=5000 and
        120<=samples<=2400):
        raise ValueError('Invalid targeting parameters')
    if abs(target_altitude_km-start_altitude_km)<1e-6:
        raise ValueError('Choose a different destination altitude')
    start_radius=RE+start_altitude_km
    target_radius=RE+target_altitude_km
    initial=state_from_elements(start_radius,0,inclination_deg)
    tangential=initial[3:]/np.linalg.norm(initial[3:])
    rising=target_radius>start_radius

    def candidate(dv_km_s):
        s=initial.copy()
        s[3:]+=dv_km_s*tangential
        el=elements_from_state(s)
        return s,el

    def error(dv_km_s):
        _,el=candidate(dv_km_s)
        opposite=el.apoapsis_altitude_km if rising else el.periapsis_altitude_km
        if opposite is None:
            return float('inf')
        return opposite+RE-target_radius

    v_circ=sqrt(MU/start_radius)
    # Two brackets of signed prograde delta-v, confirmed by numerical apsis evaluation.
    low,high=(0.0,(sqrt(2)-1)*v_circ*.9999) if rising else (-.90*v_circ,0.0)
    f_low,f_high=error(low),error(high)
    if not (f_low<=0<=f_high):
        raise ValueError('The requested apsis is outside the solvable bracket')
    iterations=0
    while iterations<100:
        iterations+=1
        mid=(low+high)/2
        fm=error(mid)
        if abs(fm)<1e-9 or abs(high-low)<1e-14:
            break
        if fm>0:high=mid
        else:low=mid
    departure_state,departure_elements=candidate(mid)
    if departure_elements.period_s is None:
        raise ValueError('Transfer orbit is not bound')
    duration=departure_elements.period_s/2
    propagation=integrate_orbit(departure_state,duration,samples)
    arrival=propagation.states[-1]
    coarse=integrate_orbit(departure_state,duration,max(60,samples//2))
    convergence_separation_m=float(np.linalg.norm(propagation.states[-1,:3]-coarse.states[-1,:3])*1000)
    radius=np.linalg.norm(arrival[:3])
    tangent=arrival[3:]-np.dot(arrival[3:],arrival[:3])/radius**2*arrival[:3]
    tangent/=np.linalg.norm(tangent)
    arrival_dv=sqrt(MU/radius)-np.dot(arrival[3:],tangent)
    after=arrival.copy()
    after[3:]+=arrival_dv*tangent
    after_elements=elements_from_state(after)
    budget=rocket_equation_budget([abs(mid)*1000,abs(arrival_dv)*1000],
                                  dry_mass_kg,propellant_kg,specific_impulse_s)
    return {
        'departure_delta_v_m_s': float(mid*1000),
        'arrival_delta_v_m_s':float(arrival_dv*1000),
        'total_delta_v_m_s':float((abs(mid)+abs(arrival_dv))*1000),
        'target_apsis': 'apoapsis' if rising else 'periapsis',
        'target_altitude_km': float(target_altitude_km),
        'start_altitude_km':float(start_altitude_km),
        'solved_opposite_apsis_altitude_km': float(departure_elements.apoapsis_altitude_km if rising else departure_elements.periapsis_altitude_km),
        'solver_residual_km':float(error(mid)),
        'numerical_arrival_radius_error_km': float(radius-target_radius),
        'post_circularization_eccentricity':float(after_elements.eccentricity),
        'transfer_time_s':float(duration), 'iterations':iterations,
        'step_refinement_position_difference_m':convergence_separation_m,
        'fuel_budget':budget,
        'departure_state':departure_state,
        'outcome':propagation,
        'maneuvers':[
            {'type':'impulse','name':'Numerically targeted departure', 'time_s':0,
             'delta_v_m_s':float(mid*1000),'position_km':initial[:3].tolist()},
            {'type':'impulse','name':'Circularization at opposite apsis',
             'time_s':float(duration), 'delta_v_m_s':float(arrival_dv*1000),
             'position_km':arrival[:3].tolist()}
        ]
    }
