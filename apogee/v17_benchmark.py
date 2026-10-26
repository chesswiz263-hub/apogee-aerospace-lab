"""Reproducible APOGEE v1.7 synthetic fault-lab benchmark.

Run: python -m apogee.v17_benchmark docs/v17-benchmark.json
"""
import json,sys
from dataclasses import replace
from pathlib import Path
from .joint_integrity import JointIntegrityConfig, simulate_joint_integrity


def run():
    base=JointIntegrityConfig(monte_carlo_runs=3)
    scenarios={
        'clean_independent': replace(base,fault_channel='none',sensor_correlation=0,
                                     engine_model_error_pct=0),
        'correlated_no_fault':replace(base,fault_channel='none'),
        'position_fault':replace(base,fault_channel='position'),
        'tracker_fault':replace(base,fault_channel='tracker'),
        'both_sensor_faults':replace(base,fault_channel='both'),
        'engine_mismatch_only':replace(base,fault_channel='none',engine_model_error_pct=20),
    }
    summary={'version':'1.7.0','note':'Synthetic shared-ADCS truth, joint EKF, seeded sensor errors; not hardware reliability.',
             'runs_per_scenario':base.monte_carlo_runs,'cases':{}}
    for name,c in scenarios.items():
        n=simulate_joint_integrity(c)['joint_integrity'];m=n['monte_carlo']
        summary['cases'][name]={
           'median_final_position_error_m':m['median_final_position_error_m'],
           'worst_final_position_error_m':m['worst_final_position_error_m'],
           'first_run_position_1sigma_rss_m':n['final_position_1sigma_rss_m'],
           'first_run_position_error_m':n['final_position_error_m'],
           'first_run_attitude_error_deg':n['final_attitude_error_deg'],
           'first_run_alarm_counts':n['alarms_by_channel'],
           'first_run_classification':n['classification'],
           'injected_measurement_alarm_fraction':m['injected_measurement_alarm_fraction'],
           'first_run_false_attribution_possible':name=='engine_mismatch_only'
        }
    return summary

if __name__=='__main__':
    output=Path(sys.argv[1]) if len(sys.argv)>1 else Path('docs/v17-benchmark.json')
    result=run();output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for k,v in result['cases'].items():
        print(f'{k:24s} median error {v["median_final_position_error_m"]:9.2f} m '
              f'alarms {v["first_run_alarm_counts"]}')
    print('wrote',output)
