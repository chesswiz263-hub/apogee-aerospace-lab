"""Numerical benchmark tooling (offline; optional scipy dependency).

DOP853 supplies an independently implemented integrator for APOGEE's force
laws; it is NOT an independent measurement of orbital truth. For comparison
with external ephemerides the epoch, time-scale, reference frame, central
body, and physical model MUST all be aligned separately.
"""
import json
from dataclasses import dataclass

import numpy as np

from .coupled import integrate_coupled, derivative
from .perturbations import ForceModel
from .propulsion import Burn
from .physics import state_from_elements
from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE


def dop853_reference(initial:np.ndarray, dry_mass_kg:float, times:np.ndarray,
                     burns:list[Burn], force:ForceModel) -> np.ndarray:
    """Solve piecewise ODE with high-order adaptive SciPy DOP853.

    Break at ignition, cutoff and exact analytic propellant exhaustion.
    At each boundary reference interpolation is evaluated on the same exact
    timestamps as the APOGEE RK4 output. Does not model impact events.
    """
    try:
        from scipy.integrate import solve_ivp
    except ImportError as exc:
        raise RuntimeError('Install optional benchmark dependency: pip install scipy') from exc
    t=np.asarray(times,dtype=float)
    if t.ndim!=1 or len(t)<2 or not np.all(np.isfinite(t)) or np.any(np.diff(t)<=0) or t[0]!=0:
        raise ValueError('Reference timeline must be strictly increasing, beginning at zero')
    y0=np.asarray(initial,dtype=float)
    if y0.shape!=(7,) or y0[6]<dry_mass_kg:raise ValueError('Invalid reference spacecraft state')
    remaining=float(y0[6]-dry_mass_kg)
    ends={0.,float(t[-1])}
    for burn in sorted(burns,key=lambda b:b.start_s):
        ends.update([float(burn.start_s),float(burn.end_s)])
        powered=min(float(burn.duration_s),max(0,remaining)/burn.mass_flow_kg_s)
        if 1e-10<powered<burn.duration_s-1e-10:ends.add(float(burn.start_s+powered))
        remaining=max(0,remaining-powered*burn.mass_flow_kg_s)
    ends=sorted(ends)
    output=np.empty((len(t),7));output[0]=y0
    y=y0.copy()
    for lo,hi in zip(ends[:-1],ends[1:]):
        if hi-lo<1e-10:continue
        mid=(lo+hi)/2
        active=next((b for b in burns if b.start_s<=mid<b.end_s),None)
        if y[6]<=dry_mass_kg+1e-10:active=None
        rhs=lambda _, state:derivative(state,force,active)
        solution=solve_ivp(rhs,(lo,hi),y,method='DOP853',rtol=1e-12,atol=1e-13,dense_output=True)
        if not solution.success:raise ValueError('DOP853 failed: '+solution.message)
        selected=np.flatnonzero((t>lo+1e-9)&(t<=hi+1e-9))
        if len(selected):output[selected]=solution.sol(t[selected]).T
        y=np.asarray(solution.y[:,-1]).copy()
        y[6]=max(dry_mass_kg,y[6])
    return output


def run_benchmark() -> dict:
    """Reproducible 90-minute powered low-Earth-orbit test with J2 and drag."""
    from .constants import EARTH_EQUATORIAL_RADIUS_KM
    y=np.r_[state_from_elements(EARTH_EQUATORIAL_RADIUS_KM+400,0,51.6,30),750.]
    burns=[Burn(1200,120,850,315,'prograde',1.0,'Benchmark burn')]
    force=ForceModel(j2=True,drag=True,mass_kg=750,area_m2=5,cd=2.2)
    sample=integrate_coupled(y,600,5400,540,burns,force)
    if sample.impact:raise ValueError('Reference experiment impacted Earth')
    ref=dop853_reference(y,600,sample.time_s,burns,force)
    rerr=np.linalg.norm(sample.states[:,:3]-ref[:,:3],axis=1)
    verr=np.linalg.norm(sample.states[:,3:6]-ref[:,3:6],axis=1)*1000
    return {
        'label':'APOGEE coupled RK4 vs SciPy DOP853 integration',
        'duration_s':5400,'initial_altitude_km':400,
        'forces':['point mass','Earth J2','illustrative rotating-atmosphere drag','finite prograde burn'],
        'samples':len(sample.time_s),
        'max_position_difference_m':float(np.max(rerr)*1000),
        'final_position_difference_m':float(rerr[-1]*1000),
        'max_velocity_difference_mm_s':float(np.max(verr)*1000),
        'final_wet_mass_kg':float(sample.states[-1,6]),
        'final_reference_wet_mass_kg':float(ref[-1,6]),
        'reference_method':'SciPy solve_ivp(method="DOP853",rtol=1e-12,atol=1e-13)',
        'caveat':'Integrator agreement does not establish truth of atmosphere/J2/spacecraft model or accuracy against real ephemerides.'
    }


if __name__=='__main__':
    print(json.dumps(run_benchmark(),indent=2))
