"""Reproducible synthetic v0.9 single-case and small seeded sweep report.

Run: python -m apogee.v09_benchmark
"""
import json
from .robust_guidance import simulate_robust_guidance, robustness_sweep

def main():
    default=simulate_robust_guidance()
    sweep=robustness_sweep(seeds=10)
    sparse=simulate_robust_guidance(station_count=3)
    mismatch=simulate_robust_guidance(truth_j2_enabled=True,filter_j2_enabled=False)
    report={
        'version':'0.9.0',
        'scope':'synthetic orbital guidance educational demonstration',
        'default':default['metrics'],
        'sweep_10_seed_summary':sweep['summary'],
        'sweep_seed_results':sweep['runs'],
        'sparse_ground_network':sparse['metrics'],
        'truth_filter_j2_mismatch':mismatch['metrics'],
        'warning':'Synthetic assumptions and 10 seeds do not establish operational navigation performance or statistically valid safety probabilities.'
    }
    print(json.dumps(report,indent=2,allow_nan=False))
if __name__=='__main__':main()
