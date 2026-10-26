"""Run reproducible v1.6 navigation integrity scenarios.

Usage: python -m apogee.v16_benchmark [output.json]
"""
import json
import sys
from dataclasses import replace
from pathlib import Path
from .integrity import IntegrityConfig, simulate_integrity


def run_benchmark():
    default=IntegrityConfig(monte_carlo_runs=12)
    cases={
        'nominal_white_noise_no_fault':replace(default,correlation=0.,thrust_error_pct=0.,fault_magnitude_m=0.),
        'time_correlated_noise_no_fault':replace(default,thrust_error_pct=0.,fault_magnitude_m=0.),
        'sensor_fault_only':replace(default,thrust_error_pct=0.),
        'engine_uncertainty_only':replace(default,fault_magnitude_m=0.),
        'combined_stress':default,
    }
    output={'version':'1.6.0','description':'Synthetic sensor/engine mismatch and chi-square gated EKF. Not operational validation.',
            'scenarios':{}}
    for name,cfg in cases.items():
        integrity=simulate_integrity(cfg)['integrity']
        output['scenarios'][name]={
            'config':cfg.__dict__,
            'first_run_final_error_m':integrity['final_position_error_m'],
            'first_run_mean_nis':integrity['mean_nis'],
            'first_run_fault_alarms':integrity['fault_alarms'],
            'first_run_pre_fault_alarms':integrity['pre_fault_alarms'],
            'monte_carlo':{k:v for k,v in integrity['monte_carlo'].items() if k!='per_run'},
        }
    return output

if __name__=='__main__':
    path=Path(sys.argv[1] if len(sys.argv)>1 else 'docs/v16-benchmark.json')
    path.parent.mkdir(exist_ok=True,parents=True)
    result=run_benchmark()
    path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(path),'scenarios':{
        key:round(val['monte_carlo']['median_final_error_m'],2)
        for key,val in result['scenarios'].items()}},indent=2))
