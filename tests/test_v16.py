"""APOGEE v1.6 fault monitoring: physics, API, statistical limitations."""
from dataclasses import replace

import numpy as np
import pytest
from fastapi.testclient import TestClient
from scipy.stats import chi2

from apogee.api import app
from apogee.integrity import (IntegrityConfig, _acceleration, _truth, _rk4,
                              _measurement_errors, _single, simulate_integrity, FRAME)
from apogee.physics import state_from_elements
from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as RE


def cfg(**kw): return replace(IntegrityConfig(monte_carlo_runs=2), **kw)


def test_default_response_is_json_serializable():
    import json
    json.dumps(simulate_integrity(cfg()), allow_nan=False)


def test_correct_frame():
    x=simulate_integrity(cfg())
    assert x['model']['reference_frame']==FRAME


def test_independent_rk4_vs_dop853_coast():
    c=cfg(burn_duration_s=0,thrust_error_pct=0,j2_enabled=False)
    times=np.linspace(0,720,49)
    truth=_truth(times,c,1.)
    pos=np.asarray(state_from_elements(RE+c.altitude_km,0.,c.inclination_deg),float)
    for a,b in zip(times[:-1],times[1:]):pos=_rk4(pos,a,b,c)
    assert np.linalg.norm(truth[-1,:3]-pos[:3])*1000 < .02


def test_thrust_model_direction_and_scale():
    c=cfg(burn_start_s=0,burn_duration_s=120,j2_enabled=False)
    x=np.asarray(state_from_elements(RE+c.altitude_km,0.,c.inclination_deg),float)
    delta=_acceleration(60,x,c,1.1)-_acceleration(60,x,c,1.)
    assert np.dot(delta,x[3:])>0


def test_ar1_stationary_statistics():
    rng=np.random.default_rng(123)
    c=cfg(correlation=.8,measurement_sigma_m=25)
    z=_measurement_errors(rng,15000,c)*1000
    assert abs(np.std(z[:,0])-25)<1.2
    assert abs(np.corrcoef(z[:-1,0],z[1:,0])[0,1]-.8)<.03


def test_zero_correlation():
    c=cfg(correlation=0)
    rng=np.random.default_rng(12)
    z=_measurement_errors(rng,10000,c)
    assert abs(np.corrcoef(z[:-1,0],z[1:,0])[0,1]) < .05


def test_3d_chi_squared_gate():
    s=_single(cfg(),31)
    assert np.isclose(s['gate_threshold_chi2_3dof'],chi2.ppf(.9973,3))


def test_missing_fault_returns_none():
    a=simulate_integrity(cfg(fault_magnitude_m=0))['integrity']
    assert a['detected_fault'] is None
    assert a['monte_carlo']['fault_detection_rate'] is None


def test_large_fault_detected_in_default():
    a=simulate_integrity(cfg(fault_magnitude_m=1200,thrust_error_pct=0))['integrity']
    assert a['detected_fault'] is True
    assert a['fault_alarms']>0


def test_alarm_never_before_fault_for_clean_reference():
    a=simulate_integrity(cfg(thrust_error_pct=0,correlation=0))['integrity']
    assert a['pre_fault_alarms']==0


def test_disabled_gating_does_not_reject():
    x=simulate_integrity(cfg(reject_faults=False))['integrity']
    assert all(x['accepted_measurement'])


def test_rejections_consistent():
    x=simulate_integrity(cfg())['integrity']
    assert x['rejected_measurements']==sum(not a for a in x['accepted_measurement'])


def test_seeded_deterministic():
    c=cfg(seed=44)
    a=simulate_integrity(c)['integrity']
    b=simulate_integrity(c)['integrity']
    assert a['position_error_m']==b['position_error_m']
    assert a['alarm']==b['alarm']


def test_different_seeds_change_realized_calibration():
    a=simulate_integrity(cfg(seed=4))['integrity']['true_thrust_calibration_factor']
    b=simulate_integrity(cfg(seed=5))['integrity']['true_thrust_calibration_factor']
    assert a != b


def test_covariance_psd():
    a=simulate_integrity(cfg())['integrity']
    assert a['min_covariance_eigenvalue']>-1e-13


def test_all_trajectory_lengths_match():
    out=simulate_integrity(cfg());tr=out['trajectory'];n=out['integrity']
    assert len(tr['time_s'])==len(n['position_error_m'])==len(n['innovation_nis'])==49


def test_sampled_engine_bias_affects_truth():
    c=cfg(thrust_error_pct=0)
    a=simulate_integrity(c)['integrity']
    assert a['true_thrust_calibration_factor']==1.
    b=simulate_integrity(cfg(thrust_error_pct=5))['integrity']
    assert b['true_thrust_calibration_factor']!=1.


def test_monte_carlo_bounded_rates():
    r=simulate_integrity(cfg())['integrity']['monte_carlo']
    assert 0<=r['fault_detection_rate']<=1
    assert 0<=r['false_alarm_rate_pre_fault']<=1
    assert r['runs']==len(r['per_run'])==2


@pytest.mark.parametrize('kwargs',[
    {'correlation':1.0}, {'samples':15}, {'monte_carlo_runs':0},
    {'duration_minutes':0}, {'burn_start_s':700,'burn_duration_s':100},
    {'fault_magnitude_m':-1}, {'reference_frame':'TEME'},
    {'seed':-1}, {'measurement_sigma_m':float('nan')},
])
def test_validation_rejects_bad_configs(kwargs):
    with pytest.raises(ValueError):simulate_integrity(cfg(**kwargs))


def test_api_returns_200_and_json():
    r=TestClient(app).post('/api/navigation/integrity',json={'monte_carlo_runs':2})
    assert r.status_code==200
    assert r.json()['integrity']['monte_carlo']['runs']==2


def test_api_rejects_frames():
    assert TestClient(app).post('/api/navigation/integrity',json={'reference_frame':'TEME'}).status_code==422


def test_api_rejects_burn_overflow():
    assert TestClient(app).post('/api/navigation/integrity',json={'duration_minutes':4,'burn_start_s':200,'burn_duration_s':120}).status_code==422


def test_prior_joint_endpoint_unchanged():
    assert '/api/navigation/joint' in [r.path for r in app.routes]
