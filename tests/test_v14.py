"""v1.4 contracts: absolute epochs, covariance injection and API boundaries."""
import importlib.util
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.flight_references import (utc_epoch, components_to_cartesian,
                                       reference_geometry, _vector3)
from apogee.integrated_nav import IntegratedNavConfig, simulate_integrated_navigation
from apogee.powered_covariance import powered_attitude_discrete_cov


def test_utc_epoch_strict():
    a=utc_epoch('2026-10-24T12:00:00Z')
    assert a.isoformat()==utc_epoch('2026-10-24T05:00:00-07:00').isoformat()
    for v in ('2026-10-24T12:00:00','garbage','2031-01-01T00:00:00Z',None):
        with pytest.raises(ValueError):utc_epoch(v)


def test_field_spherical_basis():
    # At the equator and Greenwich, radial = X, theta points -Z, phi points +Y.
    assert np.allclose(components_to_cartesian(10000,2000,500,90,0),[1e-5,5e-7,-2e-6],atol=1e-20)
    assert np.isclose(np.linalg.norm(components_to_cartesian(5,6,7,34,40)),np.linalg.norm([5,6,7])*1e-9)


def test_covariance_zero_and_psd():
    assert np.array_equal(powered_attitude_discrete_cov([0,0,0],10,20),np.zeros((6,6)))
    assert np.array_equal(powered_attitude_discrete_cov([0,1,0],10,0),np.zeros((6,6)))
    Q=powered_attitude_discrete_cov([0,0.002,0],10,4)
    assert Q.shape==(6,6) and np.allclose(Q,Q.T,atol=1e-18)
    assert np.linalg.eigvalsh(Q).min()>=-1e-18
    assert Q[1,1]<Q[0,0] # uncertainty in acceleration orthogonal to thrust
    assert np.linalg.norm(Q)>0
    for args in [([0,0,0],-1,3),([0,0,0],1,-1),([1,2],1,4),([np.nan,0,0],1,4)]:
        with pytest.raises(ValueError):powered_attitude_discrete_cov(*args)


def test_config_rejects_bad_flag():
    with pytest.raises(ValueError): IntegratedNavConfig(propagate_attitude_covariance='yes').validate()


def test_reference_rejects_frame_mix_before_optional_dependency():
    with pytest.raises(ValueError,match='Only GCRS or ITRS'):
        reference_geometry('2026-10-24T12:00:00Z',[6778,0,0],[0,7.6,0],input_frame='APOGEE_IDEAL_ECI')
    with pytest.raises(ValueError,match='Position radius'):
        reference_geometry('2026-10-24T12:00:00Z',[0,0,0],[0,0,0])
    with pytest.raises(ValueError,match='Speed'):
        reference_geometry('2026-10-24T12:00:00Z',[6778,0,0],[30,0,0])
    with pytest.raises(ValueError):_vector3([1,2,float('inf')],'position')


def test_epoch_api_error_contract():
    c=TestClient(app)
    assert c.get('/api/health').json()['version']=='2.4.0'
    req={'epoch_utc':'2026-10-24T12:00:00Z','position_km':[6778.137,0,0],
         'velocity_km_s':[0,7.6686,0]}
    bad=c.post('/api/references/epoch',json={**req,'input_frame':'TEME'})
    assert bad.status_code==422 and 'GCRS or ITRS' in bad.json()['detail']
    bad2=c.post('/api/references/epoch',json={**req,'epoch_utc':'2026-10-24'})
    assert bad2.status_code==422
    if importlib.util.find_spec('astropy') is None:
        missing=c.post('/api/references/epoch',json=req)
        assert missing.status_code==422 and 'astropy' in missing.json()['detail'].lower()


def test_integrated_no_burn_preserves_legacy_covariance():
    cfg=IntegratedNavConfig(duration_minutes=6,samples=36,navigation_period_s=60,
                            burn_start_s=0,burn_duration_s=0)
    a=simulate_integrated_navigation(cfg)
    b=simulate_integrated_navigation(IntegratedNavConfig(**{**cfg.__dict__,
                                  'propagate_attitude_covariance':True}))
    na=a['integrated_navigation'];nb=b['integrated_navigation']
    assert np.allclose(na['position_1sigma_rss_m'],nb['position_1sigma_rss_m'])
    assert nb['attitude_covariance_augmented']
    assert a['model']['reference_frame']=='APOGEE_IDEAL_ECI'


def test_integrated_powered_covariance_changes_uncertainty():
    args=dict(duration_minutes=6,samples=36,navigation_period_s=60,burn_start_s=60,
              burn_duration_s=60,propagate_attitude_covariance=True)
    augmented=simulate_integrated_navigation(IntegratedNavConfig(**args))
    baseline=simulate_integrated_navigation(IntegratedNavConfig(**{**args,'propagate_attitude_covariance':False}))
    p=np.asarray(augmented['integrated_navigation']['position_1sigma_rss_m'])
    q=np.asarray(baseline['integrated_navigation']['position_1sigma_rss_m'])
    assert np.isfinite(p).all() and (p>0).all()
    assert np.any(abs(p-q)>1e-8)
    assert augmented['integrated_navigation']['final_position_error_over_sigma_rss']>0


def test_api_includes_augmentation_flag():
    c=TestClient(app)
    response=c.post('/api/navigation/integrated',json=dict(duration_minutes=6,samples=36,
                 navigation_period_s=60,propagate_attitude_covariance=True))
    assert response.status_code==200,response.text[:1000]
    assert response.json()['integrated_navigation']['attitude_covariance_augmented']
