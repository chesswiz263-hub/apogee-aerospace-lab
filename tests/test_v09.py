"""v0.9 deterministic, mechanical, estimator and API regression tests."""
from dataclasses import replace
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.robust_guidance import (
    RobustConfig, G0, OMEGA, stations, station_state, ground_observation,
    ekf_range_rate_update, process_noise_covariance, covariance_predict,
    finite_burn, finite_burn_duration, simulate_robust_guidance, robustness_sweep)
from apogee.perturbations import ForceModel

@pytest.fixture(scope='module')
def standard():
    return simulate_robust_guidance()

def test_station_velocity_is_earth_rotation():
    r,v=station_state(10,20,100)
    assert np.allclose(v,np.cross([0,0,OMEGA],r))
    assert abs(np.dot(r,v))<1e-10

def test_ground_mask_blocks_earth():
    x=np.array([6778.,0,0,0,7.6,0])
    assert ground_observation(x,0,0,0) is not None
    assert ground_observation(x,0,180,0) is None

def test_horizon_and_elevation_degrees():
    x=np.array([6778.,0,0,0,7.6,0])
    obs=ground_observation(x,0,0,0)
    assert obs[2]==pytest.approx(90,abs=1e-8)
    assert obs[0][0]==pytest.approx(6778-6378.1363,abs=.1)

def test_rate_is_time_derivative_of_range():
    x=np.array([6778.,100,40,-.12,7.6,.02])
    h=.001
    before=ground_observation(x+np.r_[x[3:]*(-h),np.zeros(3)],0,0,-h,0)[0][0]
    after=ground_observation(x+np.r_[x[3:]*h,np.zeros(3)],0,0,h,0)[0][0]
    assert (after-before)/(2*h)==pytest.approx(ground_observation(x,0,0,0,0)[0][1],abs=3e-7)

def test_range_rate_jacobian_matches_finite_difference():
    x=np.array([6800.,120,40,-.1,7.6,.02]);o,H,_=ground_observation(x,0,0,100,0)
    for i in range(6):
        eps=1e-4 if i<3 else 1e-6
        delta=np.eye(6)[i]*eps
        a=ground_observation(x+delta,0,0,100,0)[0]
        b=ground_observation(x-delta,0,0,100,0)[0]
        assert np.allclose((a-b)/(2*eps),H[:,i],atol=5e-7,rtol=1e-5)

def test_ekf_measurement_update_is_psd():
    x=np.array([6778.,0,0,0,7.6,0])
    P=np.diag([1,1,1,1e-6,1e-6,1e-6])
    obs=ground_observation(x,0,0,0)[0]+[.01,.0001]
    x1,P1,innovation,nis=ekf_range_rate_update(x,P,obs,(0,0),0,.015,.00003)
    assert nis>=0 and np.linalg.norm(innovation)>0
    assert np.linalg.eigvalsh(P1).min()>=-1e-14
    assert np.allclose(P1,P1.T)
    assert P1[0,0]<P[0,0]

def test_invisible_filter_station_is_skipped():
    x=np.array([6778.,0,0,0,7.6,0]);P=np.eye(6)
    xa,pa,inn,nis=ekf_range_rate_update(x,P,[200,0],(0,180),0,.01,.001)
    assert np.array_equal(x,xa) and np.array_equal(P,pa)
    assert nis is None and inn is None

def test_process_noise_symmetric_psd():
    q=process_noise_covariance(90,.1)
    assert np.allclose(q,q.T)
    assert np.linalg.eigvalsh(q).min()>-1e-15
    assert np.trace(q)>0
    assert np.array_equal(process_noise_covariance(90,0),np.zeros((6,6)))

def test_filter_covariance_grows_under_process_noise():
    P=np.eye(6);phi=np.eye(6)
    assert np.trace(covariance_predict(P,phi,90,.1))>np.trace(P)

def test_duration_follows_rocket_equation():
    duration,dv=finite_burn_duration(2.,750.,850.,315.,150.,100.)
    consumed=duration*850/(315*G0)
    assert dv==pytest.approx(315*G0*np.log(750/(750-consumed)),rel=1e-10)
    assert dv==pytest.approx(2.,rel=1e-10)

def test_duration_respects_fuel_and_time():
    duration,dv=finite_burn_duration(10.,750.,850.,315.,.01,.05)
    assert duration<=.05 and duration*850/(315*G0)<=.01+1e-10
    assert finite_burn_duration(0,750,850,315,150,100)==(0,0)

def test_mass_flow_integrates_exactly():
    from apogee.physics import state_from_elements
    from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM
    x=np.r_[state_from_elements(EARTH_EQUATORIAL_RADIUS_KM+400,0,51.6),750.]
    a=finite_burn(x,0,2.,ForceModel(j2=True,drag=False),850,315,[0,1,0])
    assert a[6]==pytest.approx(750-2*850/(315*G0),abs=1e-8)
    assert np.isfinite(a).all()

def test_mass_and_stm_burn_agree():
    from apogee.physics import state_from_elements
    from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM
    x=np.r_[state_from_elements(EARTH_EQUATORIAL_RADIUS_KM+400,0,51.6),750.]
    model=ForceModel(j2=False,drag=False)
    a=finite_burn(x,0,2,model,850,315,[0,1,0])
    b,phi=finite_burn(x,0,2,model,850,315,[0,1,0],with_stm=True)
    assert np.allclose(a,b,rtol=1e-10,atol=1e-8)
    assert np.linalg.det(phi)>0

def test_reject_empty_or_invalid_burn():
    with pytest.raises(ValueError):finite_burn(np.r_[np.zeros(6),1.],0,1,ForceModel(),850,315,[0,0,0])

def test_standard_guidance_improves_position(standard):
    m=standard['metrics']
    assert m['final_guided_position_error_m']<m['final_unguided_position_error_m']
    assert m['measurement_count']>0
    assert m['correction_count']>0

def test_nominal_budget_respected(standard):
    m=standard['metrics']
    assert m['total_guidance_delta_v_m_s']<=standard['input']['max_total_delta_v_m_s']+1e-7
    assert all(e['duration_s']>0 for e in standard['events'])
    assert all(e['end_s']>e['time_s'] for e in standard['events'])

def test_actual_and_nominal_reported_separately(standard):
    m=standard['metrics']
    assert abs(m['actual_integrated_delta_v_m_s']-m['total_guidance_delta_v_m_s'])>0
    assert m['fuel_consumed_kg']>0
    assert m['fuel_remaining_kg']>=0

def test_monotonic_history(standard):
    t=standard['trajectory']['time_s'];mass=standard['trajectory']['wet_mass_kg']
    assert all(b>a for a,b in zip(t,t[1:]))
    assert all(b<=a+1e-8 for a,b in zip(mass,mass[1:]))
    assert t[-1]==pytest.approx(standard['input']['duration_minutes']*60)

def test_guidance_off_matches_unguided():
    r=simulate_robust_guidance(guidance_enabled=False)
    assert r['metrics']['correction_count']==0
    assert r['metrics']['fuel_consumed_kg']==0
    assert r['metrics']['total_guidance_delta_v_m_s']==0
    assert r['metrics']['final_guided_position_error_m']==pytest.approx(r['metrics']['final_unguided_position_error_m'],abs=1e-7)

def test_reproducibility_default(standard):
    repeat=simulate_robust_guidance()
    assert repeat['metrics']==standard['metrics']
    assert repeat['events']==standard['events']

def test_seeds_change_sensor_or_engine():
    r1=simulate_robust_guidance(seed=42)
    r2=simulate_robust_guidance(seed=43)
    assert r1['metrics']['final_guided_position_error_m']!=r2['metrics']['final_guided_position_error_m']

def test_mismatched_model_does_not_crash():
    r=simulate_robust_guidance(truth_j2_enabled=True,filter_j2_enabled=False)
    assert np.isfinite(r['metrics']['final_guided_position_error_m'])
    assert r['input']['truth_j2_enabled']!=r['input']['filter_j2_enabled']

def test_sparse_station_network_makes_fewer_measurements(standard):
    r=simulate_robust_guidance(station_count=3)
    assert r['metrics']['measurement_count']<standard['metrics']['measurement_count']

def test_zero_fuel_no_burn():
    r=simulate_robust_guidance(fuel_mass_kg=0)
    assert r['metrics']['correction_count']==0
    assert r['metrics']['fuel_remaining_kg']==0

def test_low_fuel_no_dry_mass_violation():
    r=simulate_robust_guidance(fuel_mass_kg=.01,thrust_error_pct=20.)
    assert r['metrics']['fuel_remaining_kg']>=-1e-7
    assert min(r['trajectory']['wet_mass_kg'])>=600-1e-7

def test_invalid_config_rejected():
    for cfg in (RobustConfig(station_count=8),RobustConfig(range_sigma_m=0),RobustConfig(seed=-1),RobustConfig(duration_minutes=5)):
        with pytest.raises(ValueError):cfg.validate()

def test_seed_sweep_reports_all_runs():
    result=robustness_sweep(seeds=3)
    assert len(result['runs'])==3
    assert 0<=result['summary']['runs_improved_vs_baseline']<=3
    assert result['summary']['median_guided_position_error_m']>0

def test_api_endpoint_response():
    with TestClient(app) as client:
        r=client.post('/api/robust/guidance',json={'seed':42})
        assert r.status_code==200,r.text
        data=r.json()
        assert data['robust']['station_locations_lat_lon_deg']
        assert data['events'] and data['trajectory']['position_km']
        assert client.get('/api/health').json()['version']=='2.4.0'

def test_api_rejects_invalid():
    with TestClient(app) as client:
        for payload in ({'station_count':7},{'range_sigma_m':-1},{'measurement_interval_s':13},{'fuel_mass_kg':-4}):
            assert client.post('/api/robust/guidance',json=payload).status_code==422

def test_sweep_api():
    with TestClient(app) as client:
        a=client.post('/api/robust/sweep',json={'seeds':3})
        assert a.status_code==200,a.text
        assert len(a.json()['runs'])==3
        b=client.post('/api/robust/sweep',json={'seeds':100})
        assert b.status_code==422
