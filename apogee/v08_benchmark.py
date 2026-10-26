"""Reproducible v0.8 navigation sanity benchmark; not a performance guarantee."""
import json
import numpy as np
from .guidance import simulate_guidance, _flow
from .physics import state_from_elements
from .perturbations import ForceModel
from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE, EARTH_MU_KM3_S2 as MU


def _rk4_terminal(x, seconds=5400, dt=2.0):
    def derivative(y):
        r=y[:3];v=y[3:]; rad=np.linalg.norm(r)
        return np.r_[v, -MU*r/rad**3]
    y=np.asarray(x,float).copy()
    for _ in range(int(seconds/dt)):
        a=derivative(y); b=derivative(y+dt/2*a)
        c=derivative(y+dt/2*b);d=derivative(y+dt*c)
        y=y+dt/6*(a+2*b+2*c+d)
    return y


def benchmark():
    one=simulate_guidance()
    runs=[simulate_guidance(seed=i) for i in range(10)]
    errors=[x['metrics']['final_guided_position_error_m'] for x in runs]
    budgets=[x['metrics']['total_guidance_delta_v_m_s'] for x in runs]
    # Independent classical RK4 for the same two-body reference dynamics.
    start=state_from_elements(RE+400,0,51.6)
    dop853=_flow(start,(0,5400),ForceModel(j2=False,drag=False))
    rk4=_rk4_terminal(start)
    return {
        'experiment':'400 km circular orbit; 90 minutes; 30 m per-axis position measurement noise',
        'initial_radial_error_m':600,
        'initial_alongtrack_speed_error_m_s':.4,
        'reference_dynamics_integrators':'DOP853 vs standalone fixed-step RK4 with 2-second steps',
        'reference_trajectory_endpoint_difference_m':float(np.linalg.norm(dop853[:3]-rk4[:3])*1000),
        'seed42_guided_final_position_error_m':one['metrics']['final_guided_position_error_m'],
        'seed42_unguided_final_position_error_m':one['metrics']['final_unguided_position_error_m'],
        'seed42_total_delta_v_m_s':one['metrics']['total_guidance_delta_v_m_s'],
        'seed0_to9_guided_final_error_m':errors,
        'seed0_to9_median_guided_error_m':float(np.median(errors)),
        'seed0_to9_worst_guided_error_m':float(np.max(errors)),
        'seed0_to9_max_delta_v_m_s':float(np.max(budgets)),
        'scope':'Ten deterministic synthetic runs with identical initial truth deviation and varied measurement noise; does not establish operational reliability.'
    }


def main():
    out=benchmark()
    assert out['reference_trajectory_endpoint_difference_m']<.01
    assert out['seed42_guided_final_position_error_m']<out['seed42_unguided_final_position_error_m']
    assert out['seed0_to9_max_delta_v_m_s']<=2+1e-9
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
