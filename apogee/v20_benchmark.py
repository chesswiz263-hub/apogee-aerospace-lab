"""Deterministic v2.0 ground-tracking benchmark; synthetic results only."""
from dataclasses import replace
from pathlib import Path
import json
from .ground_navigation import GroundNavConfig, simulate_ground_navigation

def run():
    root=GroundNavConfig(monte_carlo_runs=12,seed=29)
    cases={
        'nominal_finite_burn':root,
        'no_burn_scale_unobservable':replace(root,burn_duration_s=0),
        'sparse_six_stations':replace(root,station_count=6),
        'high_elevation_mask':replace(root,min_elevation_deg=30),
        'long_packet_latency':replace(root,telemetry_delay_s=120),
        'persistent_range_bias':replace(root,fault_bias_m=300),
    }
    output={}
    for name,c in cases.items():
        r=simulate_ground_navigation(c)['ground_navigation']
        output[name]={**r['summary'],**{k:v for k,v in r['monte_carlo'].items() if k!='seeds'},
                      'run_seeds':r['monte_carlo']['seeds']}
    return {'version':'2.0.0','scope':'six synthetic scenarios, 12 seeds each, not operational calibration',
            'geometry':'24 artificial stations by default, approximate GMST from UTC, spherical Earth',
            'results':output}

if __name__=='__main__':
    import sys
    dest=Path(sys.argv[1]) if len(sys.argv)>1 else Path('docs/v20-benchmark.json')
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(run(),indent=2)+'\n')
    print(f'Wrote {dest}')
