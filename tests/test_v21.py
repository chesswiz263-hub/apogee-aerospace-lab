"""v2.1 geodesy, strict frames, provenance, epoch, and API regression tests."""
import math
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.real_tracking import (A_WGS84_M,F_WGS84,strict_utc,wgs84_station_itrs,
 topocentric_observation,calculate_observation,compare_records)

ST={'latitude_deg':0.,'longitude_deg':0.,'height_m':0.}
R=A_WGS84_M/1000
Y=[R+400,0,0,0,7.5,0]
EPOCH='2026-10-25T12:00:00Z'

def test_wgs84_equatorial_radius():
    np.testing.assert_allclose(wgs84_station_itrs(ST),[R,0,0],atol=1e-11)

def test_wgs84_equatorial_quarter_turn():
    np.testing.assert_allclose(wgs84_station_itrs({**ST,'longitude_deg':90}),[0,R,0],atol=1e-11)

def test_wgs84_polar_radius():
    b=R*(1-F_WGS84)
    np.testing.assert_allclose(wgs84_station_itrs({**ST,'latitude_deg':90}),[0,0,b],atol=1e-11)

def test_station_height_is_ellipsoidal():
    assert np.isclose(wgs84_station_itrs({**ST,'height_m':500})[0],R+.5)

def test_geometry_zenith_eastbound():
    a=topocentric_observation(Y,ST)
    assert a['range_m']==pytest.approx(400000)
    assert a['range_rate_m_s']==pytest.approx(0,abs=1e-9)
    assert a['elevation_deg']==pytest.approx(90,abs=1e-9)
    assert a['azimuth_deg'] is None # undefined at exact zenith

def test_range_rate_radial_sign():
    assert topocentric_observation([R+500,0,0,-.02,0,0],ST)['range_rate_m_s']==pytest.approx(-20)

def test_geometric_range_rate_is_time_derivative():
    dt=1e-4
    state=np.array([R+400,200.,120.,.3,7.4,-1.2])
    p=state.copy();p[:3]+=dt*p[3:]
    m=state.copy();m[:3]-=dt*m[3:]
    num=(topocentric_observation(p,ST)['range_m']-topocentric_observation(m,ST)['range_m'])/(2*dt)
    analytic=topocentric_observation(state,ST)['range_rate_m_s']
    assert abs(num-analytic)<2e-5

def test_enu_at_equator_east_and_north():
    a=topocentric_observation([R,100,0,0,0,0],ST)
    b=topocentric_observation([R,0,100,0,0,0],ST)
    assert a['azimuth_deg']==pytest.approx(90)
    assert b['azimuth_deg']==pytest.approx(0)
    assert a['elevation_deg']==pytest.approx(0)

def test_itrs_epoch_observation_contains_scope():
    out=calculate_observation(EPOCH,ST,Y,'ITRS')
    assert out['state_frame']=='ITRS' and 'NO signal' in out['validity']
    assert out['eop_status'].startswith('not required')
    assert out['epoch_utc']==EPOCH

def test_observed_residual_sign():
    out=calculate_observation(EPOCH,ST,Y,'ITRS',observed={'range_m':400005,'range_rate_m_s':2.5})
    assert out['residuals']==pytest.approx({'range_m':5,'range_rate_m_s':2.5})

@pytest.mark.parametrize('frame',['TEME','APOGEE_SIM_ECI_EARTH_ROTATING','ICRF',''])
def test_incompatible_frames_rejected(frame):
    with pytest.raises(ValueError,match='frame'): calculate_observation(EPOCH,ST,Y,frame)

@pytest.mark.parametrize('epoch',['2026-10-25T12:00:00','2026-10-25T12:00:00+00:00','today','2041-01-01T00:00:00Z'])
def test_epoch_rejections(epoch):
    with pytest.raises(ValueError): strict_utc(epoch)

@pytest.mark.parametrize('station',[{'latitude_deg':91,'longitude_deg':0,'height_m':0},
  {'latitude_deg':0,'longitude_deg':-181,'height_m':0},
  {'latitude_deg':0,'longitude_deg':0,'height_m':15000}])
def test_invalid_station_rejected(station):
    with pytest.raises(ValueError): wgs84_station_itrs(station)

@pytest.mark.parametrize('state',[[1,2,3],[float('nan')]*6,[R,0,0,0,0,0]])
def test_invalid_state_rejected(state):
    with pytest.raises(ValueError): topocentric_observation(state,ST)

def test_invalid_external_measurement_rejected():
    with pytest.raises(ValueError): calculate_observation(EPOCH,ST,Y,'ITRS',observed={'range_m':-1,'range_rate_m_s':0})

def test_gcrs_requires_astropy_and_never_approximates():
    try:
        import astropy
    except ImportError:
        with pytest.raises(RuntimeError,match='Astropy'):calculate_observation(EPOCH,ST,Y,'GCRS')
    else:
        # Installed Astropy may or may not have sufficient EOP; both a proper
        # transform and an explicit error are acceptable, never a hand rotation.
        try: o=calculate_observation(EPOCH,ST,Y,'GCRS')
        except RuntimeError as e: assert 'IERS' in str(e)
        else: assert o['state_frame']=='GCRS'

def test_batch_matches_known_residuals():
    base={'epoch_utc':EPOCH,'station':ST,
          'state':{'position_km':Y[:3],'velocity_km_s':Y[3:]},'frame':'ITRS',
          'observed':{'range_m':400007,'range_rate_m_s':2}}
    res=compare_records([base,{**base,'observed':{'range_m':399997,'range_rate_m_s':-2}}],
                        'synthetic','unit-test-fixture')
    assert res['sample_count']==2
    assert res['statistics']['range_rms_m']==pytest.approx(math.sqrt(29))
    assert res['statistics']['range_bias_m']==pytest.approx(2)
    assert res['statistics']['range_rate_rms_m_s']==pytest.approx(2)

def test_mixed_frames_rejected_without_astropy(monkeypatch):
    import apogee.real_tracking as mod
    monkeypatch.setattr(mod,'_astropy_gcrs_to_itrs',lambda y,epoch:y)
    base={'epoch_utc':EPOCH,'station':ST,
          'state':{'position_km':Y[:3],'velocity_km_s':Y[3:]},'frame':'ITRS',
          'observed':{'range_m':400000,'range_rate_m_s':0}}
    with pytest.raises(ValueError,match='different coordinate frames'):
        compare_records([base,{**base,'frame':'GCRS'}],'synthetic','unit-test-fixture')

def test_api_health_v21():
    assert TestClient(app).get('/api/health').json()['version']=='2.4.0'

def test_api_itrs_success():
    r=TestClient(app).post('/api/navigation/epoch-ground-observation',json={
      'epoch_utc':EPOCH,'frame':'ITRS','station':ST,
      'state':{'position_km':Y[:3],'velocity_km_s':Y[3:]}})
    assert r.status_code==200,r.text
    assert r.json()['range_m']==pytest.approx(400000)

def test_api_wrong_frame_rejected():
    r=TestClient(app).post('/api/navigation/epoch-ground-observation',json={'frame':'TEME'})
    assert r.status_code==422

def test_api_bad_epoch_rejected():
    r=TestClient(app).post('/api/navigation/epoch-ground-observation',json={'epoch_utc':'not-an-epoch'})
    assert r.status_code==422

def test_api_batch_and_authentication_scope():
    record={'epoch_utc':EPOCH,'station':ST,
         'state':{'position_km':Y[:3],'velocity_km_s':Y[3:]},'frame':'ITRS',
         'observed':{'range_m':400005,'range_rate_m_s':0.12}}
    r=TestClient(app).post('/api/navigation/epoch-reference-comparison',json={
      'reference_id':'external-supplied-demo','provenance':'external_unverified','records':[record]})
    assert r.status_code==200,r.text
    assert r.json()['statistics']['range_rms_m']==pytest.approx(5)
    assert r.json()['provenance']=='external_unverified'
    assert 'NOT externally authenticated' in r.json()['validation_scope']

def test_api_batch_rejects_missing_reference_identifier():
    r=TestClient(app).post('/api/navigation/epoch-reference-comparison',json={'records':[]})
    assert r.status_code==422

def test_dashboard_wired():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    html=(root/'web/index.html').read_text()
    js=(root/'web/app.js').read_text()
    assert 'mode-geodesy' in html and 'geodesy-form' in html
    assert '/api/navigation/epoch-ground-observation' in js
    assert "submitGeodesy" in js
