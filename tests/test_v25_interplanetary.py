"""Independent pilot transfer tests, run on CI without fetching network data."""
from math import isfinite
import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from apogee.v25_interplanetary import _planet_position_km, lambert, plan_transfer, router
from apogee.v25_solar import AU_KM, parse_utc


def test_planet_center_agrees_with_existing_solar_viewer():
    from apogee.v25_solar import solar_system_state
    instant='2026-10-25T22:00:00Z'
    state=solar_system_state(instant)
    emb=next(b for b in state['bodies'] if b['name']=='Earth')
    assert np.allclose(_planet_position_km('Earth',parse_utc(instant))/AU_KM,
                       emb['position_au'], rtol=0, atol=1e-9)

@pytest.mark.parametrize('origin,dest,days',[
    ('Earth','Mars',260),('Earth','Venus',160),('Earth','Jupiter',1050),
    ('Mars','Earth',280),('Jupiter','Saturn',1200)
])
def test_transfer_reaches_model_target(origin,dest,days):
    result=plan_transfer(origin,dest,'2026-10-25T00:00:00Z',days,80)
    assert result['origin']==origin and result['destination']==dest
    assert len(result['trajectory'])==80
    assert result['arrival_miss_km']<2.0  # numerical Lambert/propagator consistency ONLY
    assert result['departure_v_infinity_km_s']>0
    assert all(isfinite(x) for point in result['trajectory'] for x in point['position_au'])
    assert result['frame'].startswith('heliocentric')

@pytest.mark.parametrize('a,b,days',[
 ('Earth','Earth',260),('Sun','Mars',260),('Earth','Pluto',260),
 ('Earth','Mars',5),('Earth','Mars',30000)
])
def test_input_validation(a,b,days):
    with pytest.raises(ValueError):plan_transfer(a,b,'2026-10-25T00:00:00Z',days)


def test_endpoint_response():
    app=FastAPI();app.include_router(router)
    resp=TestClient(app).get('/api/v25/interplanetary/transfer',params={
      'origin':'Earth','destination':'Mars','departure':'2026-10-25T00:00:00Z','flight_days':260,'samples':48})
    assert resp.status_code==200,resp.text
    body=resp.json();assert len(body['trajectory'])==48
    assert body['arrival_miss_km']<2


def test_no_solution_reports_422():
    app=FastAPI();app.include_router(router)
    resp=TestClient(app).get('/api/v25/interplanetary/transfer',params={
      'origin':'Earth','destination':'Earth'})
    assert resp.status_code==422


def test_lambert_collinearity_rejected():
    with pytest.raises(ValueError):lambert([AU_KM,0,0],[-AU_KM,0,0],200*86400)


def test_satellite_requires_astropy_or_injected_service(monkeypatch):
    # No CelesTrak fetch should be attempted before Astropy dependency check.
    from apogee.v25_interplanetary import satellite_heliocentric
    try:
        import astropy  # noqa
    except ImportError:
        from fastapi import HTTPException
        with pytest.raises(HTTPException) as err:satellite_heliocentric(25544)
        assert err.value.status_code==503
