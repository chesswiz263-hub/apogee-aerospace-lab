"""Reproducible v1.9 synthetic robustness and observability study."""
from dataclasses import asdict, replace
import json
import sys
from pathlib import Path
from .observability import ObservabilityConfig, simulate_observability


def main():
    base=ObservabilityConfig(monte_carlo_runs=48,seed=29)
    cases={
        'nominal_clean':replace(base,fault_bias_m=0),
        'short_outlier':base,
        'persistent_bias':replace(base,fault_duration_s=0),
        'correlated_outlier':replace(base,noise_correlation=.8),
        'no_burn':replace(base,burn_duration_s=0),
        'late_observations_only':replace(base,observation_start_s=480),
    }
    report={'version':'1.9.0','description':'Synthetic two-body local batch estimation; 48 paired seeds per case; NOT flight validation',
            'cases':{}}
    for name,c in cases.items():
        obs=simulate_observability(c)['observability']
        report['cases'][name]={'config':asdict(c),'observability':obs['effective_information'],
          'robust_methods':obs['monte_carlo'],
          'first_run_estimated_thrust_scale':obs['estimated_thrust_scale']}
        print(name, 'observable=',obs['effective_information']['observable'],
              'median errors (m)=',{m:round(v['median_final_position_error_m'],2) for m,v in obs['monte_carlo'].items()})
    dest=Path(sys.argv[1] if len(sys.argv)>1 else 'docs/v19-benchmark.json')
    dest.parent.mkdir(exist_ok=True,parents=True)
    dest.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Saved',dest)

if __name__=='__main__':main()
