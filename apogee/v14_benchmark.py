"""Reproducible v1.4 powered-covariance sensitivity, not flight validation."""
import json
from dataclasses import replace
from .integrated_nav import IntegratedNavConfig, simulate_integrated_navigation


def run():
    cfg=IntegratedNavConfig(burn_start_s=300.,burn_duration_s=120.,seed=27)
    rows=[]
    for enabled in (False,True):
        out=simulate_integrated_navigation(replace(cfg,propagate_attitude_covariance=enabled))
        nav=out['integrated_navigation']
        rows.append({'powered_covariance_injection':enabled,
            'final_error_m':nav['final_position_error_m'],
            'final_position_1sigma_rss_m':nav['final_position_1sigma_rss_m'],
            'error_over_1sigma_rss':nav['final_position_error_over_sigma_rss'],
            'final_attitude_estimation_error_deg':nav['final_attitude_estimation_error_deg'],
            'warning':'Isotropic approximation; no cross-covariance or time-correlated pointing bias'})
    return {'scenario':'400 km, 120 s burn at t=300 s, 18 min mission, seed 27',
            'comparisons':rows,'verified_absolute_frame_transform':False,
            'note':'Tests only sensitivity under synthetic model; physical flight validation is pending.'}

if __name__=='__main__':
    print(json.dumps(run(),indent=2))
