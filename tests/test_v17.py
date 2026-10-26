"""APOGEE v1.7 joint 12-state statistical navigation integrity regression checks."""
from dataclasses import replace
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.joint_integrity import (JointIntegrityConfig,simulate_joint_integrity,
    _ar1,score_packet,_predict_integrity)
from apogee.joint_navigation import _initial_filter
from scipy.stats import chi2

@pytest.fixture(scope='module')
def main():
    return simulate_joint_integrity(JointIntegrityConfig(monte_carlo_runs=2))

@pytest.fixture(scope='module')
def clean():
    return simulate_joint_integrity(replace(JointIntegrityConfig(),fault_channel='none',
                                     sensor_correlation=0,monte_carlo_runs=1))

def test_has_12d_joint_covariance_and_truth(main):
    assert main['joint_integrity']['min_covariance_eigenvalue']>=-1e-12
    assert len(main['trajectory']['time_s'])==49
    assert main['model']['reference_frame']=='APOGEE_IDEAL_ECI'

def test_joint_and_tracker_streams(main):
    assert {p['kind'] for p in main['joint_integrity']['packet_events']}=={'position','tracker'}
    assert all(p['receipt_s']>=p['capture_s'] for p in main['joint_integrity']['packet_events'])

def test_default_position_fault_is_detected(main):
    x=main['joint_integrity'];assert x['alarms_by_channel']['position']>=1
    assert x['fault_alarms']>=1
    assert x['classification']=='position_channel_inconsistent'

def test_default_fault_rejected(main):
    n=main['joint_integrity'];assert n['rejected_packets']>=1
    assert all(p['accepted']!=p['alarm'] for p in n['packet_events'])

def test_clean_baseline_has_no_injected_fault(clean):
    n=clean['joint_integrity'];assert n['fault_measurements_received']==0
    assert n['monte_carlo']['injected_measurement_alarm_fraction'] is None
    assert n['classification']=='no_alarms'

def test_tracker_only_bias_channel_isolation():
    c=replace(JointIntegrityConfig(),fault_channel='tracker',monte_carlo_runs=1)
    n=simulate_joint_integrity(c)['joint_integrity']
    assert n['alarms_by_channel']['tracker']>=1
    assert n['alarms_by_channel']['position']==0
    assert n['classification']=='tracker_channel_inconsistent'

def test_both_channels_become_suspect():
    c=replace(JointIntegrityConfig(),fault_channel='both',monte_carlo_runs=1)
    n=simulate_joint_integrity(c)['joint_integrity']
    assert n['alarms_by_channel']['position']>=1 and n['alarms_by_channel']['tracker']>=1
    assert n['classification']=='multiple_channels_or_model_mismatch'

def test_engine_error_alone_can_trigger_position_alarm():
    c=replace(JointIntegrityConfig(),fault_channel='none',engine_model_error_pct=20.,
              thrust_sigma_pct=0.,monte_carlo_runs=1)
    n=simulate_joint_integrity(c)['joint_integrity']
    assert n['alarms_by_channel']['position']>=1
    assert n['fault_measurements_received']==0

def test_monte_carlo_reproducible(main):
    again=simulate_joint_integrity(JointIntegrityConfig(monte_carlo_runs=2))
    assert again['joint_integrity']['monte_carlo']==main['joint_integrity']['monte_carlo']
    assert again['joint_integrity']['packet_events']==main['joint_integrity']['packet_events']

def test_inflation_reduces_false_precision():
    from apogee.joint_integrity import _predict_integrity
    from apogee.physics import state_from_elements
    from apogee.attitude import q_normalize
    x=np.r_[state_from_elements(6778.137,0.,51.6),[1.,0.,0.,0.],[0.,0.,0.]]
    P=np.eye(12)*1e-10
    a=replace(JointIntegrityConfig(),thrust_sigma_pct=0.)
    b=replace(a,thrust_sigma_pct=10.)
    _,p0=_predict_integrity(x,P,190,205,np.zeros(3),a)
    _,p1=_predict_integrity(x,P,190,205,np.zeros(3),b)
    assert np.trace(p1[:3,:3])>np.trace(p0[:3,:3])
    assert np.linalg.eigvalsh(p1).min()>=-1e-11

def test_ar1_seed_and_stationary_variance():
    a=_ar1(np.random.default_rng(9),10000,.03,.7)
    b=_ar1(np.random.default_rng(9),10000,.03,.7)
    assert np.array_equal(a,b)
    assert abs(np.std(a)-.03)<.001
    assert abs(np.corrcoef(a[:-1,0],a[1:,0])[0,1]-.7)<.04

def test_gate_is_threedof():
    assert chi2.ppf(.9973,3)==pytest.approx(14.156,abs=.05)

def test_score_rejects_frame():
    from apogee.joint_integrity import score_packet
    s=np.r_[np.ones(6),[1.,0.,0.,0.],np.zeros(3)]
    with pytest.raises(ValueError,match='frame'):
        score_packet(s,np.eye(12),{'frame':'GCRS','kind':'position','z':[0,0,0]},JointIntegrityConfig())

@pytest.mark.parametrize('field,value',[('fault_channel','unknown'),('sensor_correlation',1.),
    ('engine_model_error_pct',31.),('thrust_sigma_pct',51.),('gate_probability',1.),
    ('monte_carlo_runs',9),('reference_frame','TEME'),('samples',20),('fault_start_s',9999)])
def test_invalid_configuration(field,value):
    with pytest.raises(ValueError):replace(JointIntegrityConfig(),**{field:value}).validate()

def test_api_endpoint_and_health():
    client=TestClient(app)
    assert client.get('/api/health').json()['version']=='2.4.0'
    res=client.post('/api/navigation/joint-integrity',json={'monte_carlo_runs':1})
    assert res.status_code==200,res.text[:200]
    assert 'joint_integrity' in res.json()

def test_api_bad_frame_and_bad_time():
    client=TestClient(app)
    for payload in ({'reference_frame':'GCRS'},{'samples':31,'duration_minutes':12},
                    {'burn_start_s':700,'burn_duration_s':120}):
        assert client.post('/api/navigation/joint-integrity',json=payload).status_code==422

def test_inherited_joint_endpoint_unchanged():
    client=TestClient(app)
    out=client.post('/api/navigation/joint',json={'duration_minutes':6,'samples':36,
                    'tracker_period_s':60,'burn_start_s':300,'burn_duration_s':0})
    assert out.status_code==200,out.text[:200]
    assert out.json()['joint_navigation']['filter_error_dimension']==12
