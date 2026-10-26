"""Deterministic joint-filter diagnostics, not flight validation.
Run: python -m apogee.v15_benchmark
"""
import json
import statistics
from dataclasses import replace
from .joint_navigation import JointNavConfig, simulate_joint_navigation


def run():
    default=JointNavConfig(duration_minutes=12,samples=48,navigation_period_s=60,
        tracker_period_s=60, burn_start_s=300,burn_duration_s=120,seed=27)
    cases={}
    for name,cfg in [
        ('powered',default),
        ('coast',replace(default,burn_duration_s=0)),
        ('tracker_disabled',replace(default,tracker_enabled=False))]:
        n=simulate_joint_navigation(cfg)['joint_navigation']
        cases[name]={k:n[k] for k in (
            'final_position_error_m','final_position_1sigma_rss_m',
            'final_attitude_estimation_error_deg','max_position_attitude_abs_correlation',
            'final_error_over_position_sigma_rss','received_at_finish','packets_generated')}
    seeds=[]
    for seed in (3,5,7,11,13):
        n=simulate_joint_navigation(replace(default,seed=seed))['joint_navigation']
        seeds.append({'seed':seed,'final_position_error_m':n['final_position_error_m'],
                      'final_uncertainty_m':n['final_position_1sigma_rss_m'],
                      'max_position_attitude_abs_correlation':n['max_position_attitude_abs_correlation']})
    return {'version':'1.5.0','reference_frame':'APOGEE_IDEAL_ECI',
            'covariance_dimension':12,'nominal_cases':cases,
            'seed_experiment':{'runs':seeds,
               'median_final_position_error_m':statistics.median(s['final_position_error_m'] for s in seeds),
               'worst_final_position_error_m':max(s['final_position_error_m'] for s in seeds)},
            'caveat':'Synthetic truth, sensors and noise, uncalibrated process covariance; no validated real ephemeris'}

if __name__=='__main__':
    print(json.dumps(run(),indent=2,allow_nan=False))
