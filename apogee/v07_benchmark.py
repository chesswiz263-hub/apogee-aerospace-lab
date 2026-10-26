"""Deterministic APOGEE v0.7 independent-integrator and uncertainty demo.
Run: python -m apogee.v07_benchmark
"""
import json
import numpy as np
from .finite_optimization import optimize_finite_transfer
from .uncertainty import propagate_uncertainty
from .coupled import integrate_coupled
from .perturbations import ForceModel
from .propulsion import Burn


def main():
    d=optimize_finite_transfer(phase_candidates=1)
    s=d['finite_optimization']['selected']
    tr=d['trajectory']
    y0=np.r_[tr['position_km'][0],tr['velocity_km_s'][0],tr['wet_mass_kg'][0]]
    burns=[Burn(0,s['departure_duration_s'],850,315,'prograde'),
           Burn(s['apsis_event_s'],s['arrival_duration_s'],850,315,'prograde')]
    comparison=integrate_coupled(y0,600,tr['time_s'][-1],350,burns,ForceModel(j2=False,drag=False))
    position_error_m=float(np.linalg.norm(comparison.states[-1,:3]-tr['position_km'][-1])*1000)
    mass_error_kg=float(comparison.states[-1,6]-tr['wet_mass_kg'][-1])
    un=propagate_uncertainty()['uncertainty']
    print(json.dumps({'apogee_version':'0.7.0',
          'independent_integrator_RK4_vs_DOP853_position_difference_m':position_error_m,
          'mass_difference_kg':mass_error_kg,
          'finite_transfer':s,
          'uncertainty_95pct_approx_position_radius_m':un['position_95pct_conservative_radius_m'][-1],
          'uncertainty_final_STM_1sigma_max_axis_m':un['final_stm_position_1sigma_max_m'],
          'disclaimer':'Same force model numerical agreement; not flight-qualified accuracy.'},indent=2))

if __name__=='__main__':main()
