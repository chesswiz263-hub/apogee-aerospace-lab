"""APOGEE v1.5: coupled EKF, frame safety, causal packets, and compatibility."""
import dataclasses
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.joint_navigation import (JointNavConfig, dynamics_jacobian, predict,
    update, simulate_joint_navigation, _initial_filter, _replay)
from apogee.integrated_nav import FRAME


def config(**kw):
    args=dict(duration_minutes=6,samples=36,navigation_period_s=60,
        tracker_period_s=60,burn_start_s=90,burn_duration_s=90)
    args.update(kw)
    return JointNavConfig(**args)


def test_joint_config_validates():
    config().validate()

@pytest.mark.parametrize('field,value',[
    ('reference_frame','TEME'),('tracker_period_s',17.),('samples',20),
    ('tracker_sigma_deg',0.),('tracker_latency_s',-1),('gyro_sigma_deg_s',100),
    ('initial_attitude_offset_deg',91),('measurement_mode','ground_range_rate')])
def test_joint_config_rejects_bad_inputs(field,value):
    with pytest.raises(ValueError):dataclasses.replace(config(),**{field:value}).validate()


def test_gravity_jacobian_and_shape():
    cfg=config(burn_duration_s=0)
    x=np.array([6778.,0,0,0,7.67,0]);q=np.array([1.,0,0,0]);b=np.zeros(3)
    F=dynamics_jacobian(x,q,b,np.zeros(3),100,cfg)
    assert F.shape==(12,12)
    assert np.allclose(F[3:6,6:9],0)
    assert np.array_equal(F[6:9,9:12],-np.eye(3))


def test_burn_generates_position_attitude_jacobian():
    x=np.array([6778.,0,0,0,7.67,0]);q=np.array([1.,0,0,0])
    F=dynamics_jacobian(x,q,np.zeros(3),np.zeros(3),100,config())
    assert np.linalg.norm(F[3:6,6:9])>0
    assert np.allclose(F[3:6,6],0,atol=1e-16)  # body X rotation gives no thrust change


def test_cross_covariance_forms_during_burn():
    s=np.r_[np.array([6778.,0,0,0,7.67,0]),[1.,0,0,0],np.zeros(3)]
    P=np.diag([.1]*6+[.01]*3+[1e-8]*3)
    coast=config(burn_duration_s=0)
    _,Cp=predict(s,P,90,105,np.zeros(3),coast)
    _,Bp=predict(s,P,90,105,np.zeros(3),config())
    assert np.linalg.norm(Cp[:6,6:9])<1e-15
    assert np.linalg.norm(Bp[:6,6:9])>1e-9
    assert np.linalg.eigvalsh(Bp).min()>=-1e-15


def test_star_update_uses_joint_cross_covariance():
    s=np.r_[np.array([6778.,0,0,0,7.67,0]),[1.,0,0,0],np.zeros(3)]
    P=np.diag([.1]*6+[.01]*3+[1e-8]*3)
    s,P=predict(s,P,90,105,np.zeros(3),config())
    packet=dict(kind='tracker',frame=FRAME,z=[1.,0,0,0])
    s2,P2,nis=update(s,P,packet,config())
    assert P2[1,1]<P[1,1] and np.linalg.eigvalsh(P2).min()>-1e-12
    assert np.isfinite(nis)


def test_update_rejects_frame_mixing():
    s=np.r_[np.array([6778.,0,0,0,7.67,0]),[1.,0,0,0],np.zeros(3)]
    with pytest.raises(ValueError,match='frame'):
        update(s,np.eye(12),dict(kind='position',frame='GCRS',z=[6778.,0,0]),config())


def test_joint_coast_has_no_cross_correlation():
    out=simulate_joint_navigation(config(burn_duration_s=0))['joint_navigation']
    assert out['filter_error_dimension']==12
    assert max(out['position_attitude_max_abs_correlation'])<1e-7


def test_joint_powered_cross_correlation_and_finite_telemetry():
    out=simulate_joint_navigation(config())
    n=out['joint_navigation']
    assert n['max_position_attitude_abs_correlation']>.001
    assert n['cross_covariance_modeled']
    assert n['min_covariance_eigenvalue']>-1e-11
    assert np.isfinite(n['position_1sigma_rss_m']).all()
    assert out['model']['reference_frame']==FRAME
    assert out['trajectory']['propellant_remaining_kg'][-1]>=0


def test_latency_is_causal():
    low=simulate_joint_navigation(config(telemetry_latency_s=0))['joint_navigation']
    late=simulate_joint_navigation(config(telemetry_latency_s=600))['joint_navigation']
    assert low['received_at_finish']>late['received_at_finish']
    # The delayed stream cannot change the estimator before its first delivery.
    assert np.allclose(low['estimated_state_km_km_s'][:5],late['estimated_state_km_km_s'][:5])


def test_reproducible_seed():
    a=simulate_joint_navigation(config(seed=21))['joint_navigation']
    b=simulate_joint_navigation(config(seed=21))['joint_navigation']
    assert a['position_error_m']==b['position_error_m']
    assert a['max_position_attitude_abs_correlation']==b['max_position_attitude_abs_correlation']


def test_tracker_disabled_and_delayed_packets():
    n=simulate_joint_navigation(config(tracker_enabled=False))['joint_navigation']
    assert all(p['kind']=='position' for p in n['packet_metadata'])
    assert n['pending_at_finish']>=0
    assert n['received_at_finish']<=n['packets_generated']


def test_api_health_and_joint_response():
    c=TestClient(app)
    assert c.get('/api/health').json()['version']=='2.4.0'
    r=c.post('/api/navigation/joint',json=dict(duration_minutes=6,samples=36,
      navigation_period_s=60,tracker_period_s=60,burn_start_s=90,burn_duration_s=90))
    assert r.status_code==200,r.text[:1000]
    o=r.json()['joint_navigation'];assert o['filter_error_dimension']==12
    assert len(r.json()['trajectory']['time_s'])==37


def test_api_rejects_invalid_timeline():
    c=TestClient(app)
    r=c.post('/api/navigation/joint',json=dict(duration_minutes=6,samples=36,
      navigation_period_s=60,tracker_period_s=17))
    assert r.status_code==422
    r=c.post('/api/navigation/joint',json={'reference_frame':'ICRF'})
    assert r.status_code==422


def test_legacy_navigation_remains_available():
    c=TestClient(app)
    r=c.post('/api/navigation/integrated',json=dict(duration_minutes=6,samples=36,navigation_period_s=60))
    assert r.status_code==200
    assert 'integrated_navigation' in r.json()


def test_thrust_attitude_jacobian_matches_finite_difference():
    from apogee.attitude import q_mul, rotate_BI
    from apogee.adcs import rotation_vector_quaternion as expq
    cfg=config()
    x=np.array([6778.,0,0,0,7.67,0]);q=np.array([.95,.1,.2,.2]);q/=np.linalg.norm(q)
    F=dynamics_jacobian(x,q,np.zeros(3),np.zeros(3),100,cfg)
    mass=750.-850./(315.*9.80665)*(100-cfg.burn_start_s)
    # At t=100, mass is evaluated relative to the burn start at 90 s.
    body_axis=np.array([1.,0,0])
    def acceler(theta):return 850./mass/1000.*rotate_BI(q_mul(q,expq(theta)),body_axis)
    eps=1e-6
    numeric=np.column_stack([(acceler(np.eye(3)[j]*eps)-acceler(-np.eye(3)[j]*eps))/(2*eps)
                             for j in range(3)])
    assert np.allclose(F[3:6,6:9],numeric,atol=1e-11,rtol=1e-5)
