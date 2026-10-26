"""APOGEE v1.8 deterministic synthetic benchmark; no real sensor validation."""
from dataclasses import replace
import json
import sys
from pathlib import Path
from .adaptive_navigation import AdaptiveNavConfig, simulate_adaptive_navigation

def build():
    base=AdaptiveNavConfig(monte_carlo_runs=2)
    scenarios={
        'nominal_with_estimation':{},
        'nominal_without_estimation':{'estimate_thrust_scale':False},
        'position_fault_gating':{'fault_channel':'position','measurement_response':'gate'},
        'position_fault_quarantine':{'fault_channel':'position','measurement_response':'quarantine'},
        'tracker_fault_gating':{'fault_channel':'tracker','measurement_response':'gate'},
        'engine_20pct_mismatch':{'actual_thrust_error_pct':20.},
    }
    result={'version':'1.8.0','study':'Synthetic 13-error-state EKF; deterministic noise seeds, fixed ADCS truth dynamics.',
      'warning':'Not real spacecraft data or calibrated fault detection probabilities.', 'scenarios':{}}
    for name,kwargs in scenarios.items():
        c=replace(base,**kwargs)
        n=simulate_adaptive_navigation(c)['adaptive_navigation']
        result['scenarios'][name]={
            'median_position_error_m':n['monte_carlo']['median_final_position_error_m'],
            'worst_position_error_m':n['monte_carlo']['worst_final_position_error_m'],
            'first_run_estimated_thrust_scale':n['estimated_thrust_scale'],
            'first_run_true_thrust_scale':n['actual_thrust_scale'],
            'first_run_alarms':n['alarms_by_channel'],
            'first_run_rejected_packets':n['rejected_packets'],
            'first_run_quarantined_packets':n['quarantined_packets'],
            'seeds':n['monte_carlo']['seeds']}
    return result

if __name__=='__main__':
    result=build()
    dst=Path(sys.argv[1] if len(sys.argv)>1 else 'docs/v18-benchmark.json')
    dst.parent.mkdir(parents=True,exist_ok=True)
    dst.write_text(json.dumps(result,indent=2)+'\n')
    for key,d in result['scenarios'].items():
        print(key,'median_m',round(d['median_position_error_m'],2),
              'estimated_scale',round(d['first_run_estimated_thrust_scale'],5))
    print('Wrote',dst)
