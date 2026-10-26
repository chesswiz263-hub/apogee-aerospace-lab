"""Reproducible synthetic estimator/desaturation checks (no flight validation)."""
import json
from pathlib import Path
import numpy as np
from .precision_adcs import PrecisionADCSConfig,simulate_precision_adcs


def summarize(d):
    a=d['precision_adcs']
    return {key:a[key] for key in ('final_pointing_error_deg','final_estimation_error_deg',
        'final_attitude_3axis_sigma_deg','mean_nis','mean_nees','tracker_fixes',
        'tracker_blocked_sun','tracker_blocked_earth','final_wheel_momentum_norm_nms',
        'magnetorquer_active_steps','propellant_used_kg')}


def main():
    baseline=simulate_precision_adcs(PrecisionADCSConfig())
    off=simulate_precision_adcs(PrecisionADCSConfig(desaturation_enabled=False))
    tracker_off=simulate_precision_adcs(PrecisionADCSConfig(tracker_enabled=False))
    ensemble=[]
    for seed in range(25,35):
        run=simulate_precision_adcs(PrecisionADCSConfig(seed=seed))
        x=summarize(run);x['seed']=seed
        ensemble.append(x)
    report={
        'version':'1.2.0','kind':'synthetic repeatable model checks only',
        'default':summarize(baseline), 'unloading_disabled':summarize(off),
        'tracker_disabled':summarize(tracker_off),
        'ten_seed_ensemble':ensemble,
        'ensemble_summary':{
            'median_final_estimation_error_deg':float(np.median([x['final_estimation_error_deg'] for x in ensemble])),
            'max_final_estimation_error_deg':float(max(x['final_estimation_error_deg'] for x in ensemble)),
            'median_mean_nis':float(np.median([x['mean_nis'] for x in ensemble if x['mean_nis'] is not None])),
            'median_mean_nees':float(np.median([x['mean_nees'] for x in ensemble])),
            'median_final_wheel_momentum_nms':float(np.median([x['final_wheel_momentum_norm_nms'] for x in ensemble]))},
        'limitations':['NIS/NEES do not establish calibration on their own',
                       'The tracker operates on direct noisy attitude solutions, not star images',
                       'Earth field and solar direction are synthetic approximations']}
    out=Path(__file__).parents[1]/'docs/v12-benchmark.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report['ensemble_summary'],indent=2))
    print('Wrote',out)


if __name__=='__main__':main()
