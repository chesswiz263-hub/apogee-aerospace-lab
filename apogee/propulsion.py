"""APOGEE propulsion: rocket equation, continuously propelled RK4 missions.

Earth-centered inertial state [r km, v km/s, wet mass kg].
Thrust is constant at the requested throttle during each finite burn and is
steered continuously using a locally computed inertial direction. Gravity
remains central point-mass. Earth rotation, J2, aero drag, and engine gimbals
are intentionally not modeled here.
"""
from dataclasses import dataclass
from math import ceil, exp, log
from typing import Literal

import numpy as np

from .constants import EARTH_EQUATORIAL_RADIUS_KM as R_EARTH
from .constants import EARTH_MU_KM3_S2 as MU

G0_M_S2 = 9.80665
Directions = Literal['prograde', 'retrograde', 'radial_out', 'radial_in', 'normal', 'antinormal', 'transverse', 'antitransverse']


@dataclass(frozen=True)
class Burn:
    start_s: float
    duration_s: float
    thrust_n: float
    isp_s: float
    direction: Directions = 'prograde'
    throttle: float = 1.0
    name: str = 'Maneuver'

    @property
    def end_s(self):
        return self.start_s + self.duration_s

    @property
    def mass_flow_kg_s(self):
        return self.thrust_n * self.throttle / (self.isp_s * G0_M_S2)


def rocket_equation_budget(delta_vs_m_s: list[float], dry_mass_kg: float,
                           propellant_kg: float, isp_s: float) -> dict:
    """Ideal instantaneous-burn mass budget; no gravity/steering losses.

    Required fuel includes ALL ideal maneuvers and is based on final dry mass;
    actual individual burn usages are conditional on enough fuel for the plan.
    """
    if not (np.isfinite(dry_mass_kg) and dry_mass_kg > 0 and
            np.isfinite(propellant_kg) and propellant_kg >= 0 and
            np.isfinite(isp_s) and isp_s > 0):
        raise ValueError('Positive dry mass and Isp, and nonnegative propellant are required')
    values = np.asarray(delta_vs_m_s, dtype=float)
    if values.ndim != 1 or np.any(~np.isfinite(values)) or np.any(values < 0):
        raise ValueError('Burn magnitudes must be finite and nonnegative')
    total = float(np.sum(values))
    ve = isp_s * G0_M_S2
    mass_ratio = exp(total / ve)
    fuel_required = dry_mass_kg * (mass_ratio - 1)
    available_wet = dry_mass_kg + propellant_kg
    feasible = fuel_required <= propellant_kg + 1e-9
    usage = []
    remaining = available_wet
    for dv in values:
        next_mass = remaining * exp(-float(dv) / ve)
        usage.append({'delta_v_m_s': float(dv),
                      'ideal_propellant_used_kg': float(remaining - next_mass),
                      'mass_after_kg': float(next_mass)})
        remaining = next_mass
    achievable = ve * log(available_wet / dry_mass_kg)
    return {
        'feasible': bool(feasible), 'specific_impulse_s': float(isp_s),
        'initial_wet_mass_kg': float(available_wet), 'dry_mass_kg': float(dry_mass_kg),
        'available_propellant_kg': float(propellant_kg),
        'required_propellant_kg': float(fuel_required),
        'propellant_margin_kg': float(propellant_kg - fuel_required),
        'maximum_ideal_delta_v_m_s': float(achievable),
        'requested_ideal_delta_v_m_s': total,
        'burns_if_fully_feasible': usage if feasible else None,
        'ideal_final_mass_kg': float(remaining) if feasible else None,
        'assumptions': 'Ideal instantaneous burns; constant Isp; ignores gravity and steering losses.'
    }


def steering_unit(state: np.ndarray, direction: Directions) -> np.ndarray:
    r, v = state[:3], state[3:6]
    rn = np.linalg.norm(r)
    vn = np.linalg.norm(v)
    if rn <= 0 or vn <= 0:
        raise ValueError('Direction undefined for zero radius or velocity')
    radial = r / rn
    h = np.cross(r, v)
    hn = np.linalg.norm(h)
    if hn <= 1e-12:
        raise ValueError('Orbit normal undefined for near-radial motion')
    normal = h / hn
    transverse = np.cross(normal, radial)
    basis = {
        'prograde': v / vn, 'retrograde': -v / vn,
        'radial_out': radial, 'radial_in': -radial,
        'normal': normal, 'antinormal': -normal,
        'transverse': transverse, 'antitransverse': -transverse,
    }
    return basis[direction]


def finite_derivative(y: np.ndarray, burn: Burn | None, dry_mass_kg: float, mu: float = MU) -> np.ndarray:
    r, v, mass = y[:3], y[3:6], float(y[6])
    rn = np.linalg.norm(r)
    if rn <= 0:
        raise ValueError('Gravitational singularity')
    gravity = -mu * r / rn**3
    accel = gravity
    mdot = 0.0
    if burn is not None and mass > dry_mass_kg - 1e-10:
        accel = gravity + burn.thrust_n * burn.throttle / (1000 * mass) * steering_unit(y, burn.direction)
        mdot = -burn.mass_flow_kg_s
    return np.concatenate((v, accel, [mdot]))


def finite_rk4_step(y: np.ndarray, step_s: float, burn: Burn | None, dry_mass_kg: float) -> np.ndarray:
    def f(x):
        return finite_derivative(x, burn, dry_mass_kg)
    k1 = f(y)
    k2 = f(y + step_s * k1/2)
    k3 = f(y + step_s * k2/2)
    k4 = f(y + step_s * k3)
    next_y = y + step_s * (k1 + 2*k2 + 2*k3 + k4)/6
    # Avoid tiny residual floating-point negative mass drift at burnout.
    if burn is not None and abs(next_y[6] - dry_mass_kg) < 1e-8:
        next_y[6] = dry_mass_kg
    return next_y


@dataclass
class MissionResult:
    time_s: np.ndarray
    states: np.ndarray
    thrust_active: np.ndarray
    cumulative_ideal_delta_v_m_s: np.ndarray
    events: list[dict]
    burn_reports: list[dict]
    impact: bool
    requested_duration_s: float
    nominal_output_step_s: float


def integrate_mission(initial_state: np.ndarray, dry_mass_kg: float, duration_s: float,
                      samples: int, burns: list[Burn], radius_km: float = R_EARTH) -> MissionResult:
    """RK4 with exact sample boundaries at burn ignition and planned cutoff.

    Within every output interval, numerically step <= 5 s while thrusting or
    <= 30 s while coasting. On fuel exhaustion, split the integration at the
    analytically determined fuel-exhaustion time and coast thereafter.
    """
    y = np.asarray(initial_state, dtype=float).copy()
    if y.shape != (7,) or not np.all(np.isfinite(y)) or y[6] < dry_mass_kg or dry_mass_kg <= 0:
        raise ValueError('Seven-element finite state with wet mass >= positive dry mass required')
    if np.linalg.norm(y[:3]) <= radius_km:
        raise ValueError('Initial position intersects nominal Earth sphere')
    if not (0 < duration_s <= 100000 and 32 <= samples <= 2400):
        raise ValueError('Mission duration or samples outside bounded range')
    sorted_burns = sorted(burns, key=lambda b: b.start_s)
    if len(burns) > 12:
        raise ValueError('At most twelve burns are supported')
    for i, b in enumerate(sorted_burns):
        if not (np.isfinite(b.start_s) and np.isfinite(b.duration_s) and b.start_s >= 0
                and b.duration_s > 0 and b.end_s <= duration_s + 1e-9
                and np.isfinite(b.thrust_n) and b.thrust_n > 0
                and np.isfinite(b.isp_s) and b.isp_s > 0
                and np.isfinite(b.throttle) and 0 < b.throttle <= 1):
            raise ValueError('Invalid burn parameter or burn extends past mission duration')
        if i and b.start_s < sorted_burns[i-1].end_s - 1e-9:
            raise ValueError('Burn schedules may not overlap')
    grid = np.linspace(0, duration_s, samples + 1).tolist()
    for b in sorted_burns:
        grid.extend((b.start_s, b.end_s))
    grid = sorted(set(grid))
    times, states, activity = [0.0], [y.copy()], [False]
    cumulative_dv, total_ideal_dv = [0.0], 0.0
    events = []
    burn_reports = [{'name': b.name, 'direction': b.direction, 'start_s': b.start_s,
                     'end_s': b.end_s, 'thrust_n': b.thrust_n, 'isp_s': b.isp_s,
                     'fuel_used_kg': 0.0, 'delivered_ideal_delta_v_m_s': 0.0,
                     'propellant_depleted': False} for b in sorted_burns]
    impact = False
    for index in range(1, len(grid)):
        t, tnext = grid[index-1], grid[index]
        if tnext - t < 1e-11:
            continue
        active_idx = next((j for j, b in enumerate(sorted_burns)
                           if b.start_s <= (t+tnext)/2 < b.end_s), None)
        b = sorted_burns[active_idx] if active_idx is not None else None
        if b is not None and abs(t - b.start_s) < 1e-8:
            events.append({'type':'ignition', 'time_s':float(t), 'burn_name':b.name,
                           'position_km':y[:3].tolist()})
        total_dt = tnext-t
        internal_steps = max(1, ceil(total_dt/(5.0 if b is not None else 30.0)))
        for j in range(internal_steps):
            dt = total_dt/internal_steps
            powered = b is not None and y[6] > dry_mass_kg + 1e-10
            if powered:
                before = float(y[6])
                depletion_dt = (before-dry_mass_kg)/b.mass_flow_kg_s
                burn_time = min(dt,depletion_dt)
                if burn_time > 1e-11:
                    # RK4 with a mass-flow function held active to the exact split.
                    # The internal RK4 midpoint and endpoint masses remain physical.
                    y = finite_rk4_step(y,burn_time,b,dry_mass_kg)
                    y[6] = max(dry_mass_kg,before-b.mass_flow_kg_s*burn_time)
                    used = before - y[6]
                    br = burn_reports[active_idx]
                    br['fuel_used_kg'] += used
                    increment = b.isp_s*G0_M_S2*log(before/y[6])
                    br['delivered_ideal_delta_v_m_s'] += increment
                    total_ideal_dv += increment
                if burn_time < dt - 1e-10:
                    elapsed = t + j*dt + burn_time
                    if not burn_reports[active_idx]['propellant_depleted']:
                        burn_reports[active_idx]['propellant_depleted'] = True
                        events.append({'type':'fuel_exhaustion', 'time_s':float(elapsed),
                                       'burn_name':b.name,'position_km':y[:3].tolist()})
                    y = finite_rk4_step(y,dt-burn_time,None,dry_mass_kg)
            else:
                y = finite_rk4_step(y,dt,None,dry_mass_kg)
            if not np.all(np.isfinite(y)):
                raise ValueError('Propagation became non-finite')
            if np.linalg.norm(y[:3]) <= radius_km:
                impact = True
                break
        times.append(float(tnext if not impact else t + (j+1)*total_dt/internal_steps))
        states.append(y.copy())
        activity.append(b is not None and b.start_s <= times[-1] < b.end_s
                        and (y[6] > dry_mass_kg + 1e-10))
        cumulative_dv.append(total_ideal_dv)
        if not impact and b is not None and abs(tnext-b.end_s)<1e-7:
            events.append({'type':'cutoff','time_s':float(tnext),'burn_name':b.name,
                           'position_km':y[:3].tolist()})
        if impact:
            events.append({'type':'impact_sample','time_s':times[-1], 'position_km':y[:3].tolist()})
            break
    return MissionResult(np.asarray(times),np.asarray(states),np.asarray(activity),
                         np.asarray(cumulative_dv), events, burn_reports, impact,
                         float(duration_s),float(duration_s/samples))
