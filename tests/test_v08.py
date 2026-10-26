"""v0.8: navigation, EKF, and feedback controller regressions."""
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.guidance import GuidanceConfig, simulate_guidance, ekf_position_update, target_velocity_impulse, _flow
from apogee.physics import state_from_elements
from apogee.perturbations import ForceModel
from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as RE


@pytest.fixture(scope='module')
def guided():
    return simulate_guidance()


def test_default_guidance_improves_terminal_error(guided):
    m=guided['metrics']
    assert m['final_guided_position_error_m'] < m['final_unguided_position_error_m']*.05
    assert m['final_unguided_position_error_m']>1000


def test_budget_and_schedule_respected(guided):
    m=guided['metrics'];events=guided['events']
    assert 0<=m['total_guidance_delta_v_m_s']<=2.+1e-9
    assert len(events)<=3
    assert sum(e['delta_v_m_s'] for e in events)==pytest.approx(m['total_guidance_delta_v_m_s'])
    assert all(0<e['time_s']<5400 for e in events)
    assert all(e['remaining_budget_m_s']>=-1e-8 for e in events)


def test_unguided_run_no_burn_matches_baseline():
    out=simulate_guidance(guidance_enabled=False)
    assert out['events']==[]
    assert out['metrics']['total_guidance_delta_v_m_s']==0
    assert out['metrics']['final_guided_position_error_m']==pytest.approx(out['metrics']['final_unguided_position_error_m'],abs=1e-5)


def test_zero_budget_same_as_unguided():
    out=simulate_guidance(max_total_delta_v_m_s=0)
    assert not out['events']
    assert out['metrics']['final_guided_position_error_m']==pytest.approx(out['metrics']['final_unguided_position_error_m'],abs=1e-5)


def test_zero_corrections_same_as_unguided():
    out=simulate_guidance(corrections=0)
    assert not out['events']
    assert out['metrics']['final_guided_position_error_m']==pytest.approx(out['metrics']['final_unguided_position_error_m'],abs=1e-5)


def test_deterministic_measurements_and_commands(guided):
    other=simulate_guidance()
    assert other['events']==guided['events']
    assert other['guidance']['innovation_norm_m']==guided['guidance']['innovation_norm_m']
    assert other['metrics']['final_guided_position_error_m']==guided['metrics']['final_guided_position_error_m']


def test_change_seed_alters_synthetic_sensor_path():
    one=simulate_guidance(seed=1)
    two=simulate_guidance(seed=2)
    assert one['guidance']['innovation_norm_m']!=two['guidance']['innovation_norm_m']


def test_final_covariance_symmetric_positive(guided):
    p=np.asarray(guided['guidance']['final_covariance_6x6_km_units'])
    assert p.shape==(6,6)
    assert np.allclose(p,p.T,atol=1e-12)
    assert np.linalg.eigvalsh(p).min()>-1e-13
    assert guided['metrics']['final_estimated_position_1sigma_rss_m']>0


def test_ekf_measurement_update_improves_scalar_position_estimate():
    x=np.array([10.,20.,30.,1.,2.,3.])
    P=np.eye(6);z=np.array([10.1,20.,30.])
    new,P1,innovation,nis=ekf_position_update(x,P,z,.01)
    assert 10.<new[0]<10.1
    assert P1[0,0]<P[0,0]
    assert nis>0
    assert innovation.tolist()==pytest.approx([.1,0,0])


def test_linear_stm_position_sensitivity_matches_finite_difference():
    model=ForceModel(j2=False,drag=False)
    x=np.asarray(state_from_elements(RE+400,0,51.6))
    terminal,phi=_flow(x,(0,1000),model,variational=True)
    h=1e-6
    xp=x.copy();xp[4]+=h
    difference=(_flow(xp,(0,1000),model)-terminal)/h
    assert np.linalg.norm(phi[:,4]-difference)<1e-2


def test_guidance_works_with_j2():
    out=simulate_guidance(j2_enabled=True,duration_minutes=35,corrections=2)
    assert out['metrics']['final_guided_position_error_m'] < out['metrics']['final_unguided_position_error_m']
    assert 'Earth J2' in out['model']['force_terms']


def test_measurement_update_count(guided):
    g=guided['guidance']
    assert len(g['time_s'])==len(g['estimate_position_error_m'])==len(g['guided_position_error_m'])
    assert guided['metrics']['measurement_count']==60
    assert g['time_s'][-1]==pytest.approx(5400)


@pytest.mark.parametrize('bad',[
    {'altitude_km':100}, {'measurement_sigma_m':0}, {'measurement_interval_s':1},
    {'max_total_delta_v_m_s':-1}, {'corrections':6}, {'seed':-1},
    {'duration_minutes':500}, {'initial_velocity_error_m_s':7},
])
def test_bad_config_rejected(bad):
    with pytest.raises(ValueError):simulate_guidance(**bad)


def test_api_guidance_default_and_version():
    client=TestClient(app)
    response=client.post('/api/guidance',json={})
    assert response.status_code==200
    data=response.json()
    assert data['metrics']['correction_count']==3
    assert data['guidance']['time_s'][-1]==5400
    assert app.version=='2.4.0'


def test_api_validation_rejects_out_of_range():
    client=TestClient(app)
    response=client.post('/api/guidance',json={'measurement_interval_s':5})
    assert response.status_code==422


def test_api_disabled_guidance():
    response=TestClient(app).post('/api/guidance',json={'guidance_enabled':False})
    assert response.status_code==200
    assert response.json()['events']==[]


def test_idealized_frame_disclosed(guided):
    assert 'not TEME or ICRF' in guided['model']['reference_frame']
    assert 'instantaneous' in ' '.join(guided['model']['assumptions'])
    assert guided['guidance']['position_observation_model'].startswith('synthetic')
