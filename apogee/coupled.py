"""APOGEE v0.4: coupled Cartesian orbit + propulsion + J2 + drag dynamics.

Coordinates are idealized Earth-centred inertial (ECI) XYZ; distances km,
velocities km/s, mass kg, times seconds. Fixed-step RK4, analytic fuel
split; impacts detected only at integration substep boundaries.

The J2 and exponential drag force laws come from perturbations.py; direction
and burn properties come from propulsion.py. Drag uses CURRENT spacecraft
wet mass, not its initial mass, so force coupling matters.
"""
from dataclasses import dataclass, replace
from math import ceil, log
import numpy as np

from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from .perturbations import ForceModel, acceleration, conserved_values
from .propulsion import Burn, G0_M_S2, steering_unit


@dataclass
class CoupledResult:
    time_s: np.ndarray
    states: np.ndarray
    thrust_active: np.ndarray
    cumulative_ideal_delta_v_m_s: np.ndarray
    events: list[dict]
    burn_reports: list[dict]
    impact: bool
    requested_duration_s: float
    nominal_output_step_s: float
    force_model: ForceModel


def derivative(y: np.ndarray, force: ForceModel, burn: Burn | None = None) -> np.ndarray:
    """7-state derivative. Only call with burn during a segment with fuel."""
    m = float(y[6]); r = y[:3]; v = y[3:6]
    # v0.3's atmospheric drag law is reused with current wet mass.
    a = acceleration(r, v, replace(force, mass_kg=m))
    mdot = 0.0
    if burn is not None:
        a = a + burn.thrust_n * burn.throttle / (1000.0*m) * steering_unit(y, burn.direction)
        mdot = -burn.mass_flow_kg_s
    return np.r_[v, a, mdot]


def rk4(y: np.ndarray, dt: float, force: ForceModel, burn: Burn | None) -> np.ndarray:
    k1 = derivative(y,force,burn)
    k2 = derivative(y+dt*k1/2,force,burn)
    k3 = derivative(y+dt*k2/2,force,burn)
    k4 = derivative(y+dt*k3,force,burn)
    return y + dt*(k1+2*k2+2*k3+k4)/6


def integrate_coupled(initial: np.ndarray, dry_mass_kg: float, duration_s: float,
                      samples: int, burns: list[Burn], force: ForceModel,
                      radius_km: float = RE) -> CoupledResult:
    """Integrate powered and unpowered segments with EXACT burn boundaries.

    A burn's local direction is recomputed continuously. Fuel depletion is
    split at analytically determined time (constant thrust and Isp), allowing
    the remainder of the step to coast with the dry mass. Burn starts and
    cutoffs are explicit grid points. No overlapping burns are permitted.
    """
    force.validate()
    y = np.asarray(initial, dtype=float).copy()
    if y.shape != (7,) or not np.all(np.isfinite(y)) or not (0 < dry_mass_kg <= y[6]):
        raise ValueError('Initial seven-state must be finite; wet mass >= positive dry mass')
    if np.linalg.norm(y[:3]) <= radius_km:
        raise ValueError('Initial spacecraft intersects the Earth')
    if not (np.isfinite(duration_s) and 0 < duration_s <= 100000 and 32 <= samples <= 2400):
        raise ValueError('Invalid mission duration or samples')
    scheduled = sorted(burns, key=lambda b:b.start_s)
    if len(scheduled)>12: raise ValueError('At most twelve burns permitted')
    for k,b in enumerate(scheduled):
        values = (b.start_s,b.duration_s,b.thrust_n,b.isp_s,b.throttle)
        if not all(np.isfinite(v) for v in values) or not (b.start_s>=0 and b.duration_s>0 and b.end_s<=duration_s+1e-8 and b.thrust_n>0 and b.isp_s>0 and 0<b.throttle<=1):
            raise ValueError('Invalid maneuver timing or engine configuration')
        if k>0 and b.start_s < scheduled[k-1].end_s - 1e-8:
            raise ValueError('Overlapping burns are not supported')
    grid = sorted(set([float(t) for t in np.linspace(0,duration_s,samples+1)] +
                      [float(t) for b in scheduled for t in (b.start_s,b.end_s)]))
    ts = [0.0]; states = [y.copy()]; active = [False]; dv = [0.0]
    cumulative = 0.0; impact = False; events=[]
    reports = [{'name':b.name,'direction':b.direction,'start_s':b.start_s,'end_s':b.end_s,
                'thrust_n':b.thrust_n,'isp_s':b.isp_s,'fuel_used_kg':0.0,
                'delivered_ideal_delta_v_m_s':0.0,'propellant_depleted':False} for b in scheduled]
    for i in range(1,len(grid)):
        t0,t1 = grid[i-1],grid[i]
        if t1-t0<1e-10: continue
        idx = next((j for j,b in enumerate(scheduled) if b.start_s <= (t0+t1)/2 < b.end_s),None)
        b = scheduled[idx] if idx is not None else None
        if b is not None and abs(t0-b.start_s)<1e-8:
            events.append({'type':'ignition_command','time_s':t0,'burn_name':b.name,'position_km':y[:3].tolist()})
        max_dt = 5.0 if b is not None else 25.0
        nsub = max(1,ceil((t1-t0)/max_dt))
        current_t = t0
        for _ in range(nsub):
            dt=(t1-t0)/nsub
            powered = b is not None and y[6]>dry_mass_kg+1e-10
            if powered:
                before=float(y[6]); t_fuel=(before-dry_mass_kg)/b.mass_flow_kg_s
                t_power=min(dt,t_fuel)
                if t_power>1e-12:
                    y=rk4(y,t_power,force,b)
                    y[6]=max(dry_mass_kg,before-b.mass_flow_kg_s*t_power)
                    used=before-y[6]
                    reports[idx]['fuel_used_kg']+=used
                    increment=b.isp_s*G0_M_S2*log(before/y[6])
                    cumulative+=increment
                    reports[idx]['delivered_ideal_delta_v_m_s']+=increment
                # Register depletion even when it occurs exactly at the end of a
                # substep (unless the burn also ends there: normal cutoff).
                if y[6]<=dry_mass_kg+1e-10 and current_t+t_power < b.end_s-1e-9:
                    if not reports[idx]['propellant_depleted']:
                        reports[idx]['propellant_depleted']=True
                        events.append({'type':'fuel_exhaustion','time_s':current_t+t_power,
                                       'burn_name':b.name,'position_km':y[:3].tolist()})
                if t_power<dt-1e-10:
                    y=rk4(y,dt-t_power,force,None)
            else:
                y=rk4(y,dt,force,None)
            current_t+=dt
            if not np.all(np.isfinite(y)):
                raise ValueError('Non-finite trajectory; reduce step duration')
            if np.linalg.norm(y[:3])<=radius_km:
                impact=True;break
        ts.append(current_t);states.append(y.copy())
        active.append(bool(b is not None and t0<=current_t<b.end_s and y[6]>dry_mass_kg+1e-10))
        dv.append(cumulative)
        if b is not None and not impact and abs(t1-b.end_s)<1e-8:
            events.append({'type':'cutoff','time_s':t1,'burn_name':b.name,'position_km':y[:3].tolist()})
        if impact:
            events.append({'type':'impact_sample','time_s':current_t,'position_km':y[:3].tolist()})
            break
    return CoupledResult(np.array(ts),np.array(states),np.array(active),np.array(dv),
                         events,reports,impact,float(duration_s),duration_s/samples,force)


def coast_segment_diagnostics(result:CoupledResult, burns:list[Burn],dry_mass_kg:float) -> dict:
    """Coasting numerical-integrity monitor; J2: energy and axial h, drag: N/A.

    Even if fuel depletes mid-step, omit that sample interval to avoid crossing
    a thrust discontinuity; do not claim any conservation law with drag.
    """
    n=len(result.states)
    if result.force_model.drag:
        return {'relative_energy_drift':[], 'relative_angular_momentum_drift':[],
                'coast_diagnostics_valid':[],
                'max_relative_energy_drift':None,'max_relative_angular_momentum_drift':None,
                'conservation_applicable':False,
                'diagnostic_scope':'not applicable: atmospheric drag is dissipative'}
    vals=np.array([conserved_values(s, result.force_model) for s in result.states])
    errors=np.zeros((n,2)); anchor=None; valid=np.zeros(n,dtype=bool)
    for i in range(1,n):
        tmid=(result.time_s[i-1]+result.time_s[i])/2
        # If any scheduled powered interval touches the sample interval,
        # reset coast monitor even if fuel may have run out in it.
        burn_crosses=any(b.start_s<result.time_s[i]-1e-8 and b.end_s>result.time_s[i-1]+1e-8
                         and result.states[i-1,6]>dry_mass_kg+1e-9 for b in burns)
        if burn_crosses:
            anchor=None;continue
        if anchor is None: anchor=i-1
        valid[i]=True
        errors[i]=np.abs(vals[i]-vals[anchor])/np.maximum(np.abs(vals[anchor]),1e-12)
    return {'coast_diagnostics_valid':valid.tolist(),
            'relative_energy_drift':errors[:,0].tolist(),
            'relative_angular_momentum_drift':errors[:,1].tolist(),
            'max_relative_energy_drift':float(np.max(errors[:,0])),
            'max_relative_angular_momentum_drift':float(np.max(errors[:,1])),
            'conservation_applicable':True,
            'diagnostic_scope':'coast arcs only; J2 potential and axial angular momentum' if result.force_model.j2 else 'coast arcs only; two-body invariants'}


def compare_trajectories(base:CoupledResult, perturbed:CoupledResult) -> dict:
    """Compare results at shared timestamps (never subtract mismatched epochs)."""
    end=min(base.time_s[-1],perturbed.time_s[-1])
    mask=base.time_s <= end+1e-9
    t=base.time_s[mask]
    if len(t)==0: raise ValueError('No overlapping samples')
    ref=np.column_stack([np.interp(t,perturbed.time_s,perturbed.states[:,k]) for k in range(7)])
    difference=ref[:,:6]-base.states[:len(t),:6]
    dr=np.linalg.norm(difference[:,:3],axis=1)
    dv=np.linalg.norm(difference[:,3:],axis=1)*1000
    return {'time_s':t.tolist(), 'separation_km':dr.tolist(),
            'relative_speed_m_s':dv.tolist(),
            'final_separation_km':float(dr[-1]),'max_separation_km':float(dr.max()),
            'final_velocity_difference_m_s':float(dv[-1]),
            'comparison_duration_s':float(t[-1]),
            'warning':'Differences are model sensitivity, not true orbit-estimation errors; a shared initial state and epoch are assumed.'}
