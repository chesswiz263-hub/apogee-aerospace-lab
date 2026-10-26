"""APOGEE v1.3: causal capture/receipt navigation, frame and API checks."""
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.integrated_nav import (
    FRAME, IntegratedNavConfig, make_packet, reconstruct_estimate, simulate_integrated_navigation,
    _predict_interval, _jacobian_r,
)
from apogee.physics import state_from_elements


def test_default_coast_navigation_stays_finite_and_improves():
    s=simulate_integrated_navigation(IntegratedNavConfig())
    nav=s['integrated_navigation']
    assert nav['final_position_error_m']<nav['position_error_m'][0]
    assert np.isfinite(nav['final_position_1sigma_rss_m'])
    assert nav['received_at_finish']<nav['packets_generated']
    assert len(nav['estimated_state_km_km_s'])==len(s['trajectory']['time_s'])
    assert nav['frame']==FRAME
    assert 'loosely coupled' in nav['fusion_architecture']


def test_packets_never_delivered_before_receipt():
    cfg=IntegratedNavConfig()
    t=np.array([0.,30.,60.,90.,120.]);qs=np.tile([1.,0.,0.,0.],(len(t),1))
    x=np.asarray(state_from_elements(6778.137,0.,51.6))
    P=np.eye(6)*.1
    delayed=make_packet(30,60,[6778.137,0,0])
    a,pa,_,n=reconstruct_estimate(x,P,[delayed],60,cfg,t,qs)
    b,pb,_,m=reconstruct_estimate(x,P,[],60,cfg,t,qs)
    assert n==m==0
    assert np.array_equal(a,b) and np.array_equal(pa,pb)
    c,_,_,n2=reconstruct_estimate(x,P,[delayed],90,cfg,t,qs)
    d,_,_,_=reconstruct_estimate(x,P,[],90,cfg,t,qs)
    assert n2==1 and np.linalg.norm(c-d)>1e-5


def test_original_capture_epoch_determines_update():
    cfg=IntegratedNavConfig()
    t=np.array([0.,30.,60.,90.,120.]);qs=np.tile([1.,0.,0.,0.],(len(t),1))
    x=np.asarray(state_from_elements(6778.137,0.,51.6))
    P=np.eye(6)*.1
    p1=make_packet(30,60,[6780.,0.,0.])
    p2=make_packet(60,30,[6780.,0.,0.])
    a,_,_,_=reconstruct_estimate(x,P,[p1],90,cfg,t,qs)
    b,_,_,_=reconstruct_estimate(x,P,[p2],90,cfg,t,qs)
    assert np.linalg.norm(a-b)>.01


def test_frame_is_rejected_by_packet_and_config():
    with pytest.raises(ValueError,match='frame|Frame'):make_packet(0,0,[1,2,3],frame='TEME')
    with pytest.raises(ValueError,match='Frame'):IntegratedNavConfig(reference_frame='ICRF').validate()


def test_bad_packet_inputs_rejected():
    with pytest.raises(ValueError):make_packet(-1,0,[0,0,0])
    with pytest.raises(ValueError):make_packet(0,-1,[0,0,0])
    with pytest.raises(ValueError):make_packet(0,0,[0,0])
    with pytest.raises(ValueError):make_packet(0,0,[0,0,float('nan')])


def test_invalid_capture_grid_rejected():
    with pytest.raises(ValueError,match='integer multiple'):
        IntegratedNavConfig(samples=35).validate()


def test_j2_jacobian_is_consistent_with_gravity_derivative():
    from apogee.perturbations import j2_acceleration
    from apogee.constants import EARTH_MU_KM3_S2 as mu
    r=np.array([6750.,300.,1200.])
    def grav(pos):return -mu*pos/np.linalg.norm(pos)**3+j2_acceleration(pos)
    h=1e-3
    fd=np.column_stack([(grav(r+np.eye(3)[k]*h)-grav(r-np.eye(3)[k]*h))/(2*h) for k in range(3)])
    assert np.allclose(_jacobian_r(r,True),fd,rtol=5e-6,atol=1e-12)


def test_precision_adcs_payload_is_preserved():
    s=simulate_integrated_navigation(IntegratedNavConfig())
    assert s['precision_adcs']['tracker_fixes']>0
    assert len(s['attitude']['quaternion_BI_wxyz'])==len(s['trajectory']['time_s'])
    assert s['metrics']['propellant_used_kg']==0


def test_burn_mismatch_is_reported_not_hidden():
    s=simulate_integrated_navigation(IntegratedNavConfig(burn_duration_s=120))
    n=s['integrated_navigation']
    assert s['metrics']['propellant_used_kg']>0
    assert n['final_position_error_m']>n['final_position_1sigma_rss_m']
    assert any('attitude covariance' in w for w in s['warnings'])


def test_zero_latency_receives_last_packet():
    s=simulate_integrated_navigation(IntegratedNavConfig(telemetry_latency_s=0))
    n=s['integrated_navigation']
    assert n['received_at_finish']==n['packets_generated']


def test_ground_packets_have_capture_aligned_station_metadata():
    s=simulate_integrated_navigation(IntegratedNavConfig(measurement_mode='ground_range_rate'))
    for p in s['integrated_navigation']['packets']:
        assert p['kind']=='ground_range_rate'
        assert len(p['station'])==2
        assert p['receipt_s']>=p['capture_s']
        assert p['frame']==FRAME


def test_api_health_and_mission():
    client=TestClient(app)
    assert client.get('/api/health').json()['version']=='2.4.0'
    response=client.post('/api/navigation/integrated',json={})
    assert response.status_code==200
    payload=response.json()
    assert payload['integrated_navigation']['received_at_finish']>0
    assert len(payload['trajectory']['position_km'])>10


def test_api_rejects_bad_epoch_or_frame_or_bounds():
    client=TestClient(app)
    assert client.post('/api/navigation/integrated',json={'reference_frame':'TEME'}).status_code==422
    assert client.post('/api/navigation/integrated',json={'telemetry_latency_s':-1}).status_code==422
    assert client.post('/api/navigation/integrated',json={'samples':35}).status_code==422
    assert client.post('/api/navigation/integrated',json={'burn_start_s':900,'burn_duration_s':900}).status_code==422


def test_reproducibility_and_seed_sensitivity():
    cfg=IntegratedNavConfig()
    a=simulate_integrated_navigation(cfg)['integrated_navigation']
    b=simulate_integrated_navigation(cfg)['integrated_navigation']
    c=simulate_integrated_navigation(IntegratedNavConfig(seed=28))['integrated_navigation']
    assert a['estimated_state_km_km_s']==b['estimated_state_km_km_s']
    assert a['estimated_state_km_km_s']!=c['estimated_state_km_km_s']
