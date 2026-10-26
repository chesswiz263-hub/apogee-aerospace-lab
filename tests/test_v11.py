"""ADCS v1.1 regression, physical invariants, sensor fusion and API boundaries."""
from dataclasses import replace
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.adcs import (ADCSConfig,simulate_adcs,rotation_vector_quaternion,
                         quaternion_rotation_vector,quaternion_distance_deg,
                         allocate_wheel_motor_torque,estimated_body_torque,
                         adcs_dynamics)
from apogee.attitude import q_mul,q_conj,rotate_BI
from apogee.api import app

@pytest.fixture(scope='module')
def baseline():return simulate_adcs(ADCSConfig())


def test_rotvec_quaternion_roundtrip():
    for v in ([0,0,0], [.1,.2,-.3],[2.1,0.,.2]):
        assert np.allclose(quaternion_rotation_vector(rotation_vector_quaternion(v)),v,atol=1e-12)

def test_sign_ambiguity_invariant():
    a=rotation_vector_quaternion([.1,.4,.2]); b=rotation_vector_quaternion([.15,-.05,0])
    assert quaternion_distance_deg(a,b)==pytest.approx(quaternion_distance_deg(-a,-b))

def test_zero_rotation_identity():assert np.allclose(rotation_vector_quaternion(np.zeros(3)),[1,0,0,0])

def test_motor_torque_sign_and_limiter():
    cfg=ADCSConfig(wheel_max_torque_nm=.2,wheel_momentum_limit_nms=1)
    assert np.allclose(allocate_wheel_motor_torque(np.array([3.,-.5,0]),np.zeros(3),2.,cfg),[-.2,.2,0])

def test_motor_momentum_limit_reachable():
    cfg=ADCSConfig(wheel_max_torque_nm=.2,wheel_momentum_limit_nms=.5)
    for h in ([.49,-.49,.5],[-.5,.5,0]):
        motor=allocate_wheel_motor_torque([10,10,-10],np.asarray(h),10.,cfg)
        assert np.max(np.abs(np.asarray(h)+10*motor))<=.5+1e-12

def test_disabled_control_command_zero():
    cfg=ADCSConfig(controller_enabled=False)
    assert not np.any(allocate_wheel_motor_torque([10,2,1],np.array([0.,0.,0.]),1,cfg))

def test_bias_not_directly_truth_state():
    from apogee.adcs import estimated_body_torque
    cfg=ADCSConfig()
    r=np.array([6800.,0,0]);v=np.array([0,6.,1.])
    a=estimated_body_torque([1,0,0,0],np.zeros(3),r,v,cfg)
    b=estimated_body_torque([1,0,0,0],np.zeros(3),r,v,cfg)
    assert np.array_equal(a,b)

def test_momentum_conservation_body_wheel_exchange():
    cfg=ADCSConfig(j2_enabled=False)
    y=np.r_[np.array([6878.,0,0]),[0,7.6,0],[1.,0,0,0],[0.,.004,.003],750.,[.3,.2,-.1]]
    hdot=np.array([.02,-.04,.01]); delta=adcs_dynamics(0,y,cfg,hdot,False)
    I=np.asarray(cfg.inertia_kg_m2)
    L=I*y[10:13]+y[14:17]
    Ldot=I*delta[10:13]+delta[14:17]
    assert np.allclose(Ldot+np.cross(y[10:13],L),np.zeros(3),atol=1e-15)

def test_baseline_output_sizes(baseline):
    assert len(baseline['trajectory']['time_s'])==110
    assert len(baseline['adcs']['wheel_momentum_body_nms'])==110
    assert len(baseline['adcs']['attitude_estimation_error_deg'])==110
    assert len(baseline['attitude']['body_x_inertial'])==110

def test_quaternions_normalized(baseline):
    assert np.max(abs(np.linalg.norm(baseline['adcs']['true_quaternion_BI_wxyz'],axis=1)-1))<1e-12
    assert np.max(abs(np.linalg.norm(baseline['adcs']['estimated_quaternion_BI_wxyz'],axis=1)-1))<1e-12

def test_wheel_bounds(baseline):
    c=ADCSConfig()
    assert np.max(abs(np.array(baseline['adcs']['wheel_momentum_body_nms'])))<=c.wheel_momentum_limit_nms+1e-9
    assert np.max(abs(np.array(baseline['adcs']['wheel_commanded_momentum_rate_nm'])))<=c.wheel_max_torque_nm+1e-9

def test_fuel_budget(baseline):
    m=baseline['metrics']
    expected=850*120/(315*9.80665)
    assert m['propellant_used_kg']==pytest.approx(expected,abs=1e-7)
    assert m['propellant_remaining_kg']==pytest.approx(150-expected,abs=1e-7)

def test_controller_improves_pointing(baseline):
    off=simulate_adcs(ADCSConfig(controller_enabled=False))
    assert baseline['adcs']['final_pointing_error_deg']<1
    assert off['adcs']['final_pointing_error_deg']>15
    assert off['adcs']['max_wheel_momentum_nms']==0

def test_tracker_assists_bias_drift(baseline):
    off=simulate_adcs(ADCSConfig(tracker_enabled=False))
    assert baseline['adcs']['final_estimation_error_deg']<1
    assert off['adcs']['final_estimation_error_deg']>5
    assert off['adcs']['tracker_fixes']==0

def test_seed_reproducibility(baseline):
    duplicate=simulate_adcs(ADCSConfig())
    assert duplicate['adcs']['attitude_estimation_error_deg']==baseline['adcs']['attitude_estimation_error_deg']
    other=simulate_adcs(ADCSConfig(seed=28))
    assert other['adcs']['attitude_estimation_error_deg']!=baseline['adcs']['attitude_estimation_error_deg']

def test_tracker_blackout_counter():
    cfg=ADCSConfig(tracker_dropout_start_s=0,tracker_dropout_duration_s=1100)
    d=simulate_adcs(cfg)
    assert d['adcs']['tracker_fixes']==0
    assert d['adcs']['tracker_outage_samples']>0

def test_wheel_saturation_is_reported():
    d=simulate_adcs(ADCSConfig(wheel_momentum_limit_nms=.1))
    assert d['adcs']['wheel_limit_steps']>0
    assert d['adcs']['max_wheel_momentum_nms']<=.1+1e-8

def test_fuel_exhaustion():
    c=ADCSConfig(fuel_mass_kg=.4,burn_start_s=20,burn_duration_s=100)
    d=simulate_adcs(c)
    assert d['metrics']['fuel_exhausted']
    assert d['metrics']['propellant_remaining_kg']==pytest.approx(0,abs=1e-8)
    assert min(d['trajectory']['wet_mass_kg'])>=c.dry_mass_kg-1e-8

def test_burn_free():
    d=simulate_adcs(ADCSConfig(burn_duration_s=0))
    assert d['metrics']['propellant_used_kg']==pytest.approx(0,abs=1e-10)

def test_bad_parameters_rejected():
    for x in [ADCSConfig(wheel_momentum_limit_nms=-1),ADCSConfig(tracker_noise_deg=-1),
              ADCSConfig(burn_start_s=1000,burn_duration_s=400),ADCSConfig(samples=10)]:
        with pytest.raises(ValueError):x.validate()

def test_adcs_api_and_validation():
    client=TestClient(app)
    r=client.post('/api/adcs/mission',json={'duration_minutes':4,'samples':35,'burn_start_s':30,'burn_duration_s':30})
    assert r.status_code==200,r.text[:400]
    assert r.json()['adcs']['tracker_fixes']>0
    assert client.post('/api/adcs/mission',json={'wheel_momentum_limit_nms':-1}).status_code==422
    assert client.post('/api/adcs/mission',json={'burn_start_s':9000}).status_code==422
    assert client.get('/api/health').json()['version']=='2.4.0'
