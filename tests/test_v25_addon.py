"""Tests are offline, so can run without CelesTrak or paid APIs."""
import json
from datetime import datetime, timezone
from unittest.mock import patch
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from apogee.v25_solar import parse_utc, solar_system_state
from apogee.v25_operations import router

app=FastAPI();app.include_router(router)
client=TestClient(app)

def test_solar_system_all_planets():
    state=solar_system_state('2026-10-25T00:00:00Z')
    names=[b['name'] for b in state['bodies']]
    assert names[:10]==['Sun','Mercury','Venus','Earth','Mars','Jupiter','Saturn','Uranus','Neptune','Pluto']
    assert state['major_planet_count']==8
    assert state['illustrative_moon_count']>=12
    earth=next(b for b in state['bodies'] if b['name']=='Earth')
    assert .98<earth['distance_au']<1.02
    assert state['valid_years']==[1800,2050]
    assert 'barycenter' in ' '.join(state['limitations'])

def test_solar_known_ordering():
    st=solar_system_state('2026-10-25T00:00:00Z')
    r={b['name']:b['distance_au'] for b in st['bodies'] if b['type'] in ('planet','star')}
    assert r['Mercury']<r['Venus']<r['Earth']<r['Mars']<r['Jupiter']<r['Saturn']<r['Uranus']<r['Neptune']

def test_epoch_limits_and_utc():
    with pytest.raises(ValueError): solar_system_state('2100-01-01T00:00:00Z')
    with pytest.raises(ValueError): parse_utc('2026-10-25T00:00:00')
    assert parse_utc('2026-10-25T01:00:00+01:00')==datetime(2026,10,25,tzinfo=timezone.utc)

def test_solar_api():
    r=client.get('/api/v25/solar-system',params={'at':'2026-10-25T00:00:00Z'})
    assert r.status_code==200
    assert len(r.json()['bodies'])>=22
    assert client.get('/api/v25/solar-system',params={'at':'3000-10-25T00:00:00Z'}).status_code==422

def test_verification_missing_audit(monkeypatch, tmp_path):
    monkeypatch.setenv('APOGEE_V25_AUDIT',str(tmp_path/'missing.json'))
    assert client.get('/api/v25/verification').json()['status']=='NOT_RUN'

def test_catalog_id_validation():
    r=client.get('/api/v25/satellites/0/state')
    assert r.status_code==422

def test_reference_element_distances():
    st=solar_system_state('2000-01-01T12:00:00Z')
    body={b['name']:b for b in st['bodies']}
    for n in ['Mercury','Earth','Mars','Neptune']:
        b=body[n]
        assert b['semi_major_axis_au']*(1-b['eccentricity'])-.001 <= b['distance_au'] <= b['semi_major_axis_au']*(1+b['eccentricity'])+.001


def test_omm_cache_no_repeat_network(monkeypatch,tmp_path):
    import apogee.v25_operations as ops
    ops._CACHE.clear();ops._QUERY_TIMES.clear()
    monkeypatch.setattr(ops,'CACHE_DIR',tmp_path)
    calls=[]
    class Reply:
        status=200
        def __enter__(self):return self
        def __exit__(self,*args):return False
        def read(self,n):return json.dumps([{'NORAD_CAT_ID':25544,'EPOCH':'2026-10-25T00:00:00'}]).encode()
    def fake(url,timeout):
        calls.append(url.full_url)
        assert 'FORMAT=JSON' in url.full_url
        return Reply()
    monkeypatch.setattr(ops,'urlopen',fake)
    a, cached=ops._fetch_omm(25544)
    assert cached is False
    b, cached=ops._fetch_omm(25544)
    assert cached is True and a==b
    assert len(calls)==1


def test_omm_request_budget(monkeypatch,tmp_path):
    import apogee.v25_operations as ops
    from fastapi import HTTPException
    ops._CACHE.clear()
    monkeypatch.setattr(ops,'CACHE_DIR',tmp_path)
    import time
    monkeypatch.setattr(ops,'_QUERY_TIMES',[time.time()]*ops.MAX_QUERIES_PER_HOUR)
    with pytest.raises(HTTPException) as e:
        ops._fetch_omm(40001)
    assert e.value.status_code==429
    ops._QUERY_TIMES.clear()
