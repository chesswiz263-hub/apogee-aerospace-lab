"""APOGEE v1.8: experimental engine-scale estimation and sensor response tests."""
from dataclasses import replace
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.adaptive_navigation import (AdaptiveNavConfig, simulate_adaptive_navigation,
    _initial_adaptive, predict_adaptive, innovation, replay_adaptive)
from apogee.physics import state_from_elements
from apogee.integrated_nav import FRAME

@pytest.fixture(scope='module')
def nominal():
    return simulate_adaptive_navigation(AdaptiveNavConfig())['adaptive_navigation']


def test_dimension_and_finite_filter():
    truth=np.r_[state_from_elements(6778.137,0.,51.6),[1.,0.,0.,0.],[0.,0.,0.]]
    c=AdaptiveNavConfig()
    state,P,scale=_initial_adaptive(c,np.array([truth]))
    assert state.shape==(13,) and P.shape==(13,13) and P[12,12]>.01
    final,Q,s=predict_adaptive(state,P,scale,195,210,np.zeros(3),c)
    assert np.isfinite(Q).all() and np.allclose(Q,Q.T,atol=1e-13)
    assert np.linalg.eigvalsh(Q).min()>-1e-9
    assert abs(Q[3:6,12]).max()>1e-10   # calibration state couples to orbit during burn


def test_no_burn_no_calibration_coupling():
    truth=np.r_[state_from_elements(6778.137,0.,51.6),[1.,0.,0.,0.],[0.,0.,0.]]
    c=AdaptiveNavConfig()
    state,P,scale=_initial_adaptive(c,np.array([truth]))
    _,Q,_=predict_adaptive(state,P,scale,15,30,np.zeros(3),c)
    assert np.max(np.abs(Q[:12,12]))<1e-14


def test_thrust_calibration_converges_in_default(nominal):
    assert nominal['actual_thrust_scale']==pytest.approx(1.05)
    assert abs(nominal['estimated_thrust_scale']-1.05)<.03
    assert nominal['final_position_error_m']<200


def test_disabling_calibration_preserves_initial_guess():
    r=simulate_adaptive_navigation(replace(AdaptiveNavConfig(),estimate_thrust_scale=False))['adaptive_navigation']
    assert r['estimated_thrust_scale']==pytest.approx(1)
    assert r['final_position_error_m']>500


def test_uneventful_coast_has_no_engine_calibration_information():
    c=replace(AdaptiveNavConfig(),burn_duration_s=0)
    r=simulate_adaptive_navigation(c)['adaptive_navigation']
    assert r['estimated_thrust_scale']==pytest.approx(1.,abs=1e-10)


def test_packet_receipt_is_causal():
    from apogee.adaptive_navigation import _initial_adaptive
    x=np.r_[state_from_elements(6778.137,0,51.6),[1.,0.,0.,0.],[0.,0.,0.]]
    c=replace(AdaptiveNavConfig(),duration_minutes=3,samples=36,burn_duration_s=0,
              navigation_period_s=30.,tracker_period_s=30.)
    state,P,scale=_initial_adaptive(c,np.array([x]))
    times=np.linspace(0,180,37);gyro=np.zeros((37,3))
    packet={'frame':FRAME,'capture_s':30.,'receipt_s':100.,'kind':'position','z':[1,2,3],
            'fault_injected':False}
    _,_,_,before,_=replay_adaptive(state,P,scale,times,gyro,[packet],95,c)
    assert not before
    _,_,_,after,_=replay_adaptive(state,P,scale,times,gyro,[packet],100,c)
    assert len(after)==1 and after[0]['capture_s']==30


def test_wrong_measurement_frame():
    from apogee.adaptive_navigation import _initial_adaptive
    x=np.r_[state_from_elements(6778.137,0,51.6),[1.,0.,0.,0.],[0.,0.,0.]]
    c=AdaptiveNavConfig();state,P,_=_initial_adaptive(c,np.array([x]))
    with pytest.raises(ValueError,match='frame'):
        innovation(state,P,{'frame':'TEME','kind':'position','z':[0,0,0]},c)


def test_monitor_policy_never_rejects():
    c=replace(AdaptiveNavConfig(),fault_channel='position',measurement_response='monitor')
    r=simulate_adaptive_navigation(c)['adaptive_navigation']
    assert r['rejected_packets']==0
    assert r['total_alarms']>0


def test_quarantine_is_channel_specific_and_bounded():
    c=replace(AdaptiveNavConfig(),fault_channel='position',measurement_response='quarantine')
    r=simulate_adaptive_navigation(c)['adaptive_navigation']
    assert r['quarantined_packets']>=1
    assert any(e['action']=='quarantine_started' for e in r['packet_events'])
    assert all(e['kind']=='position' for e in r['packet_events'] if e['action']=='quarantined')


def test_noise_multiplier_is_bounded(nominal):
    assert all(1<=v<=25 for v in nominal['final_noise_multipliers'].values())


def test_no_adaptation_keeps_unity_noise():
    r=simulate_adaptive_navigation(replace(AdaptiveNavConfig(),adaptive_noise=False))['adaptive_navigation']
    assert r['final_noise_multipliers']=={'position':1.,'tracker':1.}


def test_star_tracker_fault_separate_from_position_channel():
    r=simulate_adaptive_navigation(replace(AdaptiveNavConfig(),fault_channel='tracker'))['adaptive_navigation']
    assert r['alarms_by_channel']['tracker']>=1


def test_repeated_seed_reproduces_results(nominal):
    other=simulate_adaptive_navigation(AdaptiveNavConfig())['adaptive_navigation']
    assert other['packet_events']==nominal['packet_events']
    assert other['monte_carlo']==nominal['monte_carlo']


def test_traj_matches_times(nominal):
    assert len(nominal['time_s'])==49
    assert len(nominal['position_error_m'])==49
    assert len(nominal['estimated_thrust_scale_history'])==49


@pytest.mark.parametrize('field,value',[
    ('actual_thrust_error_pct',31.),('initial_thrust_sigma_pct',31.),
    ('measurement_response','disable_everything'),('quarantine_after_alarms',0),
    ('quarantine_duration_s',1201.),('adaptive_gain',2.),
    ('process_thrust_sigma_pct_sqrt_s',-1),('monte_carlo_runs',5),
    ('reference_frame','GCRS'),('fault_channel','invalid')])
def test_invalid_configuration(field,value):
    with pytest.raises(ValueError):replace(AdaptiveNavConfig(),**{field:value}).validate()


def test_api_status_and_legacy_compatibility():
    client=TestClient(app)
    assert app.version=='2.4.0' and client.get('/api/health').json()['version']=='2.4.0'
    result=client.post('/api/navigation/adaptive',json={'monte_carlo_runs':1})
    assert result.status_code==200,result.text[:300]
    assert 'adaptive_navigation' in result.json()
    assert result.json()['model']['reference_frame']==FRAME
    assert client.post('/api/navigation/joint-integrity',json={'monte_carlo_runs':1}).status_code==200


@pytest.mark.parametrize('payload',[
    {'reference_frame':'TEME'}, {'measurement_response':'anything'},
    {'actual_thrust_error_pct':80}, {'monte_carlo_runs':10},
    {'duration_minutes':12,'samples':31},
    {'fault_start_s':3500}, {'quarantine_duration_s':-1}])
def test_api_rejects_bad_settings(payload):
    r=TestClient(app).post('/api/navigation/adaptive',json=payload)
    assert r.status_code==422,r.text[:200]
