"""v1.0 independent analytic checks, coupled dynamics, API and regressions."""
from dataclasses import replace
from math import pi
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.attitude import (AttitudeConfig, attitude_error_deg, q_normalize, q_mul,
    q_from_dcm, q_conj, rotate_BI, rotate_IB, control_torque, dynamics,
    initial_state, simulate_attitude, gravity_gradient_torque)
from apogee.constants import EARTH_MU_KM3_S2 as MU


def test_quaternion_rotation_90deg():
    q=[np.cos(pi/4),0,0,np.sin(pi/4)]
    np.testing.assert_allclose(rotate_BI(q,[1,0,0]),[0,1,0],atol=1e-14)


def test_inverse_rotation():
    q=q_normalize([.5,.5,.3,-.7]);v=[-2.5,1.7,4]
    np.testing.assert_allclose(rotate_IB(q,rotate_BI(q,v)),v,atol=1e-13)


def test_hamilton_associative():
    a=q_normalize([.1,.3,.9,-.2]);b=q_normalize([.5,.3,.6,-.4]);c=q_normalize([.4,.4,.7,.8])
    np.testing.assert_allclose(q_mul(q_mul(a,b),c),q_mul(a,q_mul(b,c)),atol=1e-14)


def test_quaternion_matrix_round_trip():
    q=q_normalize([.1,.3,.4,.8])
    R=np.column_stack([rotate_BI(q,unit) for unit in np.eye(3)])
    q2=q_from_dcm(R)
    assert abs(np.dot(q,q2))==pytest.approx(1,abs=1e-13)


def test_dcm_rejects_reflection():
    with pytest.raises(ValueError):q_from_dcm(np.diag([1.,1.,-1.]))


def test_engine_force_follows_attitude():
    cfg=AttitudeConfig(j2_enabled=False,controller_enabled=False,
                       gravity_gradient_enabled=False)
    s=initial_state(cfg);s[10:13]=0
    coast=dynamics(20,s,cfg,False)
    thrust=dynamics(20,s,cfg,True)
    expected=cfg.thrust_n/s[-1]/1000*rotate_BI(s[6:10],[1,0,0])
    np.testing.assert_allclose(thrust[3:6]-coast[3:6],expected,atol=1e-14)
    assert thrust[-1]==pytest.approx(-cfg.thrust_n/(cfg.specific_impulse_s*9.80665))


def test_controller_saturated():
    cfg=AttitudeConfig(max_control_torque_nm=.05,initial_misalignment_deg=70)
    assert np.linalg.norm(control_torque(initial_state(cfg),cfg))<=.050000000001


def test_controller_off_torque_zero():
    cfg=AttitudeConfig(controller_enabled=False)
    np.testing.assert_allclose(control_torque(initial_state(cfg),cfg),[0,0,0],atol=0)


def test_invalid_burn_rejected():
    with pytest.raises(ValueError):simulate_attitude(AttitudeConfig(duration_minutes=2,burn_start_s=100,burn_duration_s=100))


def test_no_burn_keeps_mass():
    r=simulate_attitude(AttitudeConfig(duration_minutes=3,burn_start_s=0,burn_duration_s=0,samples=30))
    assert r['metrics']['propellant_used_kg']==pytest.approx(0,abs=1e-10)
    assert r['metrics']['ideal_thrust_delta_v_m_s']==pytest.approx(0,abs=1e-10)


def test_fuel_mass_analytic_burn():
    cfg=AttitudeConfig(duration_minutes=5,burn_start_s=60,burn_duration_s=100,samples=60)
    r=simulate_attitude(cfg)
    expected=cfg.thrust_n*cfg.burn_duration_s/(cfg.specific_impulse_s*9.80665)
    assert r['metrics']['propellant_used_kg']==pytest.approx(expected,abs=1e-6)
    m0=cfg.dry_mass_kg+cfg.fuel_mass_kg
    expected_dv=cfg.specific_impulse_s*9.80665*np.log(m0/(m0-expected))
    assert r['metrics']['ideal_thrust_delta_v_m_s']==pytest.approx(expected_dv,rel=1e-7)


def test_fuel_exhaustion_and_no_negative_mass():
    cfg=AttitudeConfig(duration_minutes=5,burn_start_s=10,burn_duration_s=250,fuel_mass_kg=.3,samples=100)
    r=simulate_attitude(cfg)
    assert r['metrics']['fuel_exhausted']
    assert r['metrics']['propellant_used_kg']==pytest.approx(.3,abs=1e-6)
    assert min(r['trajectory']['wet_mass_kg'])>=cfg.dry_mass_kg-1e-9
    assert any(e['type']=='fuel_exhaustion' for e in r['events'])


def test_unit_quaternions_in_long_run():
    cfg=AttitudeConfig(duration_minutes=30,samples=70)
    r=simulate_attitude(cfg)
    qs=np.array(r['attitude']['quaternion_BI_wxyz'])
    np.testing.assert_allclose(np.linalg.norm(qs,axis=1),1.,atol=1e-12)


def test_attitude_control_reduces_misalignment():
    cfg=AttitudeConfig(duration_minutes=8,samples=65)
    r=simulate_attitude(cfg)
    assert r['attitude']['pointing_error_deg'][0]==pytest.approx(25.,abs=1e-7)
    assert r['attitude']['final_pointing_error_deg']<1.
    assert max(r['attitude']['control_torque_nm'])<=cfg.max_control_torque_nm+1e-8


def test_controlled_burn_better_pointing_than_disabled():
    cfg=AttitudeConfig(duration_minutes=10,samples=70,burn_start_s=90,burn_duration_s=120)
    with_control=simulate_attitude(cfg)
    without=simulate_attitude(replace(cfg,controller_enabled=False))
    assert with_control['attitude']['mean_burn_pointing_error_deg'] < without['attitude']['mean_burn_pointing_error_deg']


def test_prograde_and_retrograde_burns_opposite():
    cfg=AttitudeConfig(duration_minutes=5,samples=50,burn_start_s=30,burn_duration_s=40,initial_misalignment_deg=0,controller_enabled=False,j2_enabled=False)
    p=simulate_attitude(cfg)
    m=simulate_attitude(replace(cfg,target_mode='retrograde'))
    assert p['final_orbital_elements']['semi_major_axis_km']>m['final_orbital_elements']['semi_major_axis_km']


def test_zero_torque_free_spin_conserves_rotational_energy():
    cfg=AttitudeConfig(controller_enabled=False,j2_enabled=False,gravity_gradient_enabled=False)
    y=initial_state(cfg);y[10:13]=[.05,.02,.06]
    d=dynamics(0,y,cfg,False)
    I=np.array(cfg.inertia_kg_m2)
    assert np.dot(I*y[10:13],d[10:13])==pytest.approx(0,abs=1e-12)


def test_gravity_gradient_zero_when_radial_axis_is_principal_axis():
    cfg=AttitudeConfig(target_mode='inertial_x',initial_misalignment_deg=0)
    y=initial_state(cfg)
    np.testing.assert_allclose(gravity_gradient_torque(y,np.array(cfg.inertia_kg_m2)),0,atol=1e-13)


def test_api_and_form_constraints():
    client=TestClient(app)
    r=client.post('/api/attitude/mission',json={'duration_minutes':4,'burn_start_s':60,'burn_duration_s':50,'samples':35})
    assert r.status_code==200,r.text
    a=r.json();assert len(a['trajectory']['position_km'])==36
    assert len(a['attitude']['pointing_error_deg'])==36
    assert a['model']['integrator'].startswith('DOP853')
    assert client.post('/api/attitude/mission',json={'burn_start_s':120,'burn_duration_s':300,'duration_minutes':5}).status_code==422
    assert client.post('/api/attitude/mission',json={'target_mode':'bad'}).status_code==422
    assert client.get('/api/health').json()['version']=='2.4.0'
