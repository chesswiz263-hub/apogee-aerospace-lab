"""Deterministic ADCS sensitivity experiments; run python -m apogee.v11_benchmark."""
from dataclasses import replace
from pathlib import Path
import json
import numpy as np
from .adcs import ADCSConfig, simulate_adcs


def run():
    base=ADCSConfig()
    cases={
        'default':base,
        'controller_off':replace(base,controller_enabled=False),
        'tracker_off':replace(base,tracker_enabled=False),
        'tracker_blackout':replace(base,tracker_dropout_start_s=120.,tracker_dropout_duration_s=900.),
        'wheel_momentum_limit_0p1_Nms':replace(base,wheel_momentum_limit_nms=.1),
        'fuel_exhaustion':replace(base,fuel_mass_kg=.4),
    }
    records={}
    for key,cfg in cases.items():
        r=simulate_adcs(cfg)
        a=r['adcs']
        records[key]={k:a[k] for k in ['final_pointing_error_deg','final_estimation_error_deg',
                                          'rms_estimation_error_deg','tracker_fixes',
                                          'wheel_limit_steps','max_wheel_momentum_nms','propellant_used_kg']}
    errors=[]
    for seed in range(30,40):
        r=simulate_adcs(replace(base,seed=seed))
        errors.append(r['adcs']['final_estimation_error_deg'])
    records['ten_seed_final_estimation_error_deg']={
        'min':float(min(errors)),'median':float(np.median(errors)),
        'max':float(max(errors)),'seeds':list(range(30,40))}
    output=Path('docs/v11-benchmark.json')
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps({'description':'Synthetic numerical sensitivities; not real spacecraft measurements',
                                  'config':vars(base),'cases':records},indent=2))
    print(json.dumps(records,indent=2))

if __name__=='__main__':run()
