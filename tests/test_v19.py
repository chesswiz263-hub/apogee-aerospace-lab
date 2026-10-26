"""v1.9: local observability, paired robust batch fitting, and release API."""
from dataclasses import replace
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.observability import ObservabilityConfig, simulate_observability, _propagate, _design, _estimate, information_analysis

@pytest.fixture(scope='module')
def result():
    return simulate_observability(ObservabilityConfig(monte_carlo_runs=16))

def test_nominal_schema(result):
    assert result['version']=='1.9.0' and len(result['trajectory']['time_s'])==61
    assert 'observability' in result and result['observability']['effective_information']['observable']

def test_no_burn_is_unobservable():
    r=simulate_observability(ObservabilityConfig(burn_duration_s=0,monte_carlo_runs=1))
    o=r['observability']['effective_information']
    assert o['observable'] is False and o['effective_scale_information']<1e-14 and o['rank']==6

def test_finite_burn_sensitivity_is_nonzero():
    c=ObservabilityConfig(monte_carlo_runs=1)
    t=np.linspace(0,720,21)
    state,stm=_propagate(c,t,1,sensitivities=True)
    assert stm[0,0,0]==1 and np.linalg.norm(stm[0,:3,6])==0
    assert np.linalg.norm(stm[-1,:3,6])>.001

def test_scale_sensitivity_against_independent_finite_difference():
    c=ObservabilityConfig(monte_carlo_runs=1)
    t=np.linspace(0,720,21)
    _,s=_propagate(c,t,1,sensitivities=True)
    dp,_=_propagate(c,t,1.0001)
    dm,_=_propagate(c,t,.9999)
    approximation=(dp[-1,:3]-dm[-1,:3])/.0002
    assert np.linalg.norm(approximation-s[-1,:3,6])<.001

def test_initial_position_stm_matches_finite_difference():
    c=ObservabilityConfig(monte_carlo_runs=1)
    t=np.linspace(0,720,21)
    _,s=_propagate(c,t,sensitivities=True)
    eps=1e-4
    dp,_=_propagate(c,t,initial=np.r_[eps,0,0,0,0,0])
    dm,_=_propagate(c,t,initial=np.r_[-eps,0,0,0,0,0])
    assert np.linalg.norm((dp[-1,:3]-dm[-1,:3])/(2*eps)-s[-1,:3,0])<.001

def test_noise_statistics_reference(result):
    info=result['observability']['effective_information']
    assert info['scale_1sigma_from_data']>0 and info['rank']==7

def test_only_late_fix_cannot_reliably_identify_scale():
    c=ObservabilityConfig(observation_start_s=690,monte_carlo_runs=1)
    r=simulate_observability(c)['observability']['effective_information']
    assert not r['observable']

def test_clean_case_compares_methods():
    r=simulate_observability(ObservabilityConfig(fault_bias_m=0,monte_carlo_runs=12))['observability']
    assert set(r['monte_carlo'])=={'least_squares','huber','gate'}
    assert all(np.isfinite(v['median_final_position_error_m']) for v in r['monte_carlo'].values())

def test_paired_seed_count(result):
    assert {v['runs'] for v in result['observability']['monte_carlo'].values()}=={16}

def test_robust_method_outlier_advantage(result):
    o=result['observability']['monte_carlo']
    assert o['huber']['median_final_position_error_m']<o['least_squares']['median_final_position_error_m']

def test_reproducible_across_runs():
    c=ObservabilityConfig(monte_carlo_runs=5)
    a=simulate_observability(c)['observability'];b=simulate_observability(c)['observability']
    assert a['monte_carlo']==b['monte_carlo']

def test_seed_changes_results():
    a=simulate_observability(ObservabilityConfig(monte_carlo_runs=3,seed=1))['observability']
    b=simulate_observability(ObservabilityConfig(monte_carlo_runs=3,seed=2))['observability']
    assert a['monte_carlo']!=b['monte_carlo']

def test_coverage_bounds(result):
    for v in result['observability']['monte_carlo'].values():
        assert 0<=v['position_95pct_coverage']<=1
        lo,hi=v['position_coverage_wilson_95pct']
        assert 0<=lo<=v['position_95pct_coverage']<=hi<=1

def test_correlated_measurement_not_silently_relabelled():
    r=simulate_observability(ObservabilityConfig(noise_correlation=.9,monte_carlo_runs=8))
    assert 'independent' in r['observability']['assumptions']

def test_persistent_fault_is_not_guaranteed_recovery():
    r=simulate_observability(ObservabilityConfig(fault_duration_s=0,monte_carlo_runs=5))['observability']
    assert r['monte_carlo']['huber']['median_final_position_error_m']>100

def test_zero_fault_no_bias():
    r=simulate_observability(ObservabilityConfig(fault_bias_m=0,monte_carlo_runs=8))['observability']
    assert r['monte_carlo']['least_squares']['median_final_position_error_m']<100

def test_invalid_frame_rejected():
    with pytest.raises(ValueError):simulate_observability(ObservabilityConfig(reference_frame='GCRS'))

def test_burn_beyond_mission_rejected():
    with pytest.raises(ValueError):simulate_observability(ObservabilityConfig(burn_start_s=690,burn_duration_s=120))

def test_invalid_estimator_rejected():
    with pytest.raises(ValueError):simulate_observability(ObservabilityConfig(estimator='invented'))

def test_request_endpoint():
    c=TestClient(app)
    r=c.post('/api/navigation/observability',json={'monte_carlo_runs':8,'estimator':'gate'})
    assert r.status_code==200
    assert r.json()['observability']['method']=='gate'

def test_api_validation():
    c=TestClient(app)
    assert c.post('/api/navigation/observability',json={'samples':0}).status_code==422
    assert c.post('/api/navigation/observability',json={'reference_frame':'TEME'}).status_code==422
    assert c.post('/api/navigation/observability',json={'burn_start_s':700,'burn_duration_s':120}).status_code==422

def test_health_is_current():
    c=TestClient(app)
    assert app.version=='2.4.0' and c.get('/api/health').json()['version']=='2.4.0'
