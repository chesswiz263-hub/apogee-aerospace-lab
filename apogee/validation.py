"""Standalone deterministic RK4 convergence experiment vs circular Kepler orbit.

Run: python -m apogee.validation
Measures algorithmic error for one simple two-body truth case, not fidelity to
observations or Earth force-model uncertainty.
"""
import json
from math import cos, sin, pi, sqrt
import numpy as np
from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import integrate_orbit


def run_convergence():
    r=RE+700
    rate=sqrt(MU/r**3)
    period=2*pi/rate
    duration=1.5*period
    velocity=r*rate
    initial=np.array([r,0,0,0,velocity,0])
    expected=np.array([r*cos(rate*duration),r*sin(rate*duration),0,
                       -velocity*sin(rate*duration),velocity*cos(rate*duration),0])
    resolutions=[]
    for n in [80,160,320,640]:
        outcome=integrate_orbit(initial,duration,n)
        actual=outcome.states[-1]
        resolutions.append({'steps':n, 'step_s':duration/n,
                            'position_error_m':float(np.linalg.norm(actual[:3]-expected[:3])*1000),
                            'velocity_error_mm_s':float(np.linalg.norm(actual[3:]-expected[3:])*1e6)})
    ratios=[resolutions[i]['position_error_m']/resolutions[i+1]['position_error_m']
            for i in range(3)]
    return {'reference':'analytical two-body circular trajectory',
            'radius_km':r,'duration_s':duration,
            'rk4_refinement_expected_ratio':'~16 for 2x steps before floating-point limits',
            'resolutions':resolutions, 'position_error_ratios':ratios}


if __name__=='__main__':
    print(json.dumps(run_convergence(),indent=2))
