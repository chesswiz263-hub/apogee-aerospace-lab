"""v2.0 scientific validity, timing, station geometry and API tests."""
from dataclasses import replace
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.ground_navigation import (
    FRAME, GroundNavConfig, station_state, stations, observable, measurement,
    _gmst, _initial_estimate, _predict, _jac, _flow, _single, replay_at,
    simulate_ground_navigation, compare_reference_observations)


@pytest.fixture(scope='module')
def sim():
    return simulate_ground_navigation(GroundNavConfig(monte_carlo_runs=3))


def test_api_and_version(sim):
    assert sim['version']=='2.0.0' and len(sim['trajectory']['time_s'])==49
    assert TestClient(app).get('/api/health').json()['version']=='2.4.0'


def test_measurement_jacobian_against_finite_difference():
    c=GroundNavConfig();y,_,_=_initial_estimate(c);st=stations(24)[1];theta=_gmst(c.epoch_utc)
    _,H=measurement(y,st,120,theta)
    for col in range(6):
        step=1e-5 if col<3 else 1e-7
        plus=y.copy();minus=y.copy();plus[col]+=step;minus[col]-=step
        fd=(measurement(plus,st,120,theta)[0]-measurement(minus,st,120,theta)[0])/(2*step)
        assert np.allclose(H[:,col],fd,rtol=1e-3,atol=3e-5)
    assert np.array_equal(H[:,6],np.zeros(2))


def test_range_rate_is_range_time_derivative():
    c=GroundNavConfig();y,_,_=_initial_estimate(c);st=stations(24)[0];theta=_gmst(c.epoch_utc)
    dt=.001
    xp=y.copy();xm=y.copy();xp[:3]+=xp[3:6]*dt;xm[:3]-=xm[3:6]*dt
    rp=measurement(xp,st,100+dt,theta)[0][0]
    rm=measurement(xm,st,100-dt,theta)[0][0]
    dr=(rp-rm)/(2*dt)
    assert abs(dr-measurement(y,st,100,theta)[0][1])<1e-4


def test_station_velocity_matches_finite_difference():
    c=GroundNavConfig();s=stations(24)[0];theta=_gmst(c.epoch_utc)
    r,v=station_state(s,50,theta)
    a=station_state(s,50+.1,theta)[0];b=station_state(s,50-.1,theta)[0]
    assert np.linalg.norm((a-b)/.2-v)<1e-8


def test_station_count_and_horizon():
    for n in (6,12,24):assert len(stations(n))==n
    c=GroundNavConfig();theta=_gmst(c.epoch_utc);station=stations(24)[0]
    r,v=station_state(station,0,theta)
    overhead=np.r_[r/np.linalg.norm(r)*(6378.137+400),[0,0,0]]
    away=np.r_[-overhead[:3],[0,0,0]]
    assert observable(overhead,station,0,theta)[0]
    assert not observable(away,station,0,theta)[0]


def test_no_early_packet_use(sim):
    n=sim['ground_navigation'];packets=n['packets'];times=sim['trajectory']['time_s']
    for i,t in enumerate(times):
        assert n['received_packet_count'][i]==sum(p['receipt_time_s']<=t+1e-9 for p in packets)


def test_delayed_replay_matches_only_received():
    c=GroundNavConfig(monte_carlo_runs=1)
    p={'station_id':'S01','capture_time_s':0.,'receipt_time_s':90.,'range_m':1000000.,'range_rate_m_s':0.}
    now=40.;theta=_gmst(c.epoch_utc)
    a,PA,pkt,_=replay_at(c,[p],now,theta)
    b,PB,pkt2,_=replay_at(c,[],now,theta)
    assert not pkt and not pkt2
    assert np.array_equal(a,b) and np.array_equal(PA,PB)
    _,_,arrived,_=replay_at(c,[p],100,theta)
    assert len(arrived)==1


def test_covariance_psd_and_finite():
    c=GroundNavConfig();y,P,u=_initial_estimate(c)
    y,P=_predict(y,P,0,700,c,u)
    assert np.all(np.isfinite(P)) and np.min(np.linalg.eigvalsh(P))>=-1e-12
    assert y[6]==1.


def test_thrust_sensitivity_zero_coast():
    c=GroundNavConfig(burn_duration_s=0)
    y,P,u=_initial_estimate(c)
    assert np.array_equal(_jac(210,y,c,u)[:,6],np.zeros(7))
    no_burn=simulate_ground_navigation(replace(c,monte_carlo_runs=1))
    assert no_burn['ground_navigation']['summary']['final_thrust_scale']==pytest.approx(1.)


def test_thrust_sensitivity_on_burn():
    c=GroundNavConfig();y,P,u=_initial_estimate(c)
    assert np.linalg.norm(_jac(230,y,c,u)[3:6,6])>0


def test_truth_independence_and_causal_schema(sim):
    gn=sim['ground_navigation'];q=gn['summary'];
    assert len(gn['monte_carlo']['seeds'])==3
    assert q['received_count']<=q['packet_count']
    assert np.isfinite(q['final_error_m']) and q['final_1sigma_rss_m']>0
    assert len(gn['position_error_m'])==len(sim['trajectory']['time_s'])
    assert all(p['receipt_time_s']>=p['capture_time_s'] for p in gn['packets'])


def test_no_packet_visibility():
    c=GroundNavConfig(station_count=6,min_elevation_deg=35.,monte_carlo_runs=1)
    r=simulate_ground_navigation(c)
    assert r['ground_navigation']['summary']['received_count']<=r['ground_navigation']['summary']['packet_count']


def test_reproducible():
    c=GroundNavConfig(monte_carlo_runs=2)
    r=simulate_ground_navigation(c)['ground_navigation']['monte_carlo']
    assert r==simulate_ground_navigation(c)['ground_navigation']['monte_carlo']


def test_different_seeds_change_noise():
    c=GroundNavConfig(monte_carlo_runs=1)
    a=_single(c,1,False);b=_single(c,2,False)
    assert a['final_error_m']!=b['final_error_m']


@pytest.mark.parametrize('changes',[{'reference_frame':'GCRS'}, {'epoch_utc':'2026-10-25T12:00:00'},
    {'station_count':9},{'duration_minutes':2},{'burn_start_s':700,'burn_duration_s':120},
    {'true_thrust_scale':.1},{'range_rate_sigma_m_s':0},{'seed':-1}])
def test_reject_invalid(changes):
    with pytest.raises(ValueError):simulate_ground_navigation(replace(GroundNavConfig(),**changes))


def test_post_api_valid_and_invalid():
    c=TestClient(app)
    r=c.post('/api/navigation/ground-tracking',json={'monte_carlo_runs':1})
    assert r.status_code==200 and r.json()['ground_navigation']['summary']['packet_count']>=0
    assert c.post('/api/navigation/ground-tracking',json={'reference_frame':'ITRS'}).status_code==422
    assert c.post('/api/navigation/ground-tracking',json={'station_count':5}).status_code==422


def test_reference_residuals_roundtrip():
    c=GroundNavConfig()
    from apogee.observability import ObservabilityConfig,_propagate
    t=180.;theta=_gmst(c.epoch_utc)
    o=ObservabilityConfig(altitude_km=c.altitude_km,inclination_deg=c.inclination_deg,
                         duration_minutes=c.duration_minutes,burn_start_s=c.burn_start_s,
                         burn_duration_s=c.burn_duration_s)
    state,_=_propagate(o,np.array([t]),1.)
    s=stations(c.station_count)[0];v,_=measurement(state[0],s,t,theta)
    record={'capture_time_s':t,'station_id':s['id'],'range_m':float(v[0]),'range_rate_m_s':float(v[1])}
    result=compare_reference_observations(c,[record])
    assert result['reference_metadata']['physical_validation'] is False
    assert result['rms_range_residual_m']<1e-6
    assert result['rms_range_rate_residual_m_s']<1e-8
    assert TestClient(app).post('/api/navigation/ground-reference-residuals',json={'configuration':{},'records':[record]}).status_code==200


def test_reference_rejection():
    c=GroundNavConfig()
    with pytest.raises(ValueError):compare_reference_observations(c,[{'capture_time_s':0,'station_id':'BAD','range_m':1,'range_rate_m_s':0}])
    assert TestClient(app).post('/api/navigation/ground-reference-residuals',json={'configuration':{'reference_frame':'GCRS'},'records':[]}).status_code==422
