"""APOGEE v2.2 independent software-verification tests."""
import copy
import hashlib
import json
import math
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.independent_validation import (FIXTURE,analytic_elliptic_state,
    analytic_benchmark,parse_sgp4_reference,sgp4_reference_benchmark,suite_report)

@pytest.fixture
def data():return json.loads(FIXTURE.read_text())

def test_analytic_initial_radius_and_finite():
    state=analytic_elliptic_state(0)
    assert state.shape==(6,) and np.all(np.isfinite(state))
    assert 6378 < np.linalg.norm(state[:3]) < 10000

@pytest.mark.parametrize('time_s',[0.,1.,1234.,9999.])
def test_kepler_repeatability(time_s):
    assert np.array_equal(analytic_elliptic_state(time_s),analytic_elliptic_state(time_s))

def test_kepler_periodicity():
    from apogee.constants import EARTH_MU_KM3_S2
    T=2*math.pi*math.sqrt(8400**3/EARTH_MU_KM3_S2)
    assert np.linalg.norm(analytic_elliptic_state(0)-analytic_elliptic_state(T))<1e-10

def test_analytic_baseline():
    x=analytic_benchmark()
    assert x['status']=='PASS' and x['max_position_error_m']<3 and x['max_velocity_error_m_s']<0.003

def test_analytic_convergence():
    a=analytic_benchmark(steps_per_orbit=120,orbits=1)['max_position_error_m']
    b=analytic_benchmark(steps_per_orbit=240,orbits=1)['max_position_error_m']
    c=analytic_benchmark(steps_per_orbit=480,orbits=1)['max_position_error_m']
    assert 12<a/b<20 and 12<b/c<20

@pytest.mark.parametrize('bad',[(20,1),(1300,1),(240,0),(240,6)])
def test_bounded_inputs(bad):
    with pytest.raises(ValueError):analytic_benchmark(steps_per_orbit=bad[0],orbits=bad[1])

def test_builtin_reference_provenance(data):
    r=parse_sgp4_reference(data)
    assert r=={'case_count':1,'sample_count':5}
    assert data['frame']=='TEME' and data['satellites'][0]['catalog_id']==5
    assert data['satellites'][0]['samples'][0]['position_km'][0]==7022.46529266

def test_reference_sha_is_stable(data):
    r=suite_report()
    assert r['reference_bundle']['sha256']==hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
    assert r['results'][1]['status'] in ('PASS','SKIP')
    assert r['suite_status']==('PASS' if r['results'][1]['status']=='PASS' else 'PARTIAL')

@pytest.mark.parametrize('field,value',[
    ('frame','GCRS'),('time_unit','UTC'),('position_unit','m'),('velocity_unit','m/s'),('schema','something-else')])
def test_reject_incompatible_reference(data,field,value):
    data[field]=value
    with pytest.raises(ValueError):parse_sgp4_reference(data)

def test_reject_nonfinite(data):
    data['satellites'][0]['samples'][0]['position_km'][0]=float('nan')
    with pytest.raises(ValueError):parse_sgp4_reference(data)

def test_reject_duplicate_epoch(data):
    data['satellites'][0]['samples'].append(copy.deepcopy(data['satellites'][0]['samples'][0]))
    with pytest.raises(ValueError):parse_sgp4_reference(data)

def test_reject_mismatched_catalog(data):
    data['satellites'][0]['catalog_id']=12345
    with pytest.raises(ValueError):parse_sgp4_reference(data)

def test_sgp4_skip_not_fake_pass(data,monkeypatch):
    import builtins
    real_import=builtins.__import__
    def block(name,*args,**kwargs):
        if name.startswith('sgp4'):raise ImportError('forced missing optional library')
        return real_import(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',block)
    r=sgp4_reference_benchmark(data)
    assert r['status']=='SKIP' and r['max_position_error_m'] is None
    with pytest.raises(RuntimeError):sgp4_reference_benchmark(data,allow_skip=False)

def test_api_analytic():
    r=TestClient(app).post('/api/verification/independent',json={'steps_per_orbit':240,'orbits':2})
    assert r.status_code==200 and r.json()['results'][0]['status']=='PASS'
    assert TestClient(app).get('/api/health').json()['version']=='2.4.0'

def test_api_reject_excessive_workload():
    r=TestClient(app).post('/api/verification/independent',json={'steps_per_orbit':9999,'orbits':10})
    assert r.status_code==422

def test_api_reject_bad_reference(data):
    data['frame']='ITRS'
    r=TestClient(app).post('/api/verification/sgp4-reference',json=data)
    assert r.status_code==422

def test_api_reference_requires_library(data,monkeypatch):
    # When available, compare actual published data; when missing, fail clearly.
    r=TestClient(app).post('/api/verification/sgp4-reference',json=data)
    assert (r.status_code==200 and r.json()['status'] in {'PASS','FAIL'}) or (r.status_code==422 and 'sgp4' in r.text.lower())

def test_home_includes_v22():
    response=TestClient(app).get('/')
    assert response.status_code==200 and 'mode-v22' in response.text
