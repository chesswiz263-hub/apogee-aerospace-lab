"""APOGEE 2.4 software verification release-gate tests."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from fastapi.testclient import TestClient
import pytest

from apogee.api import app
from apogee.v23_validation import CURATED
from apogee.v24_validation import evaluate, MIN_CASES, MIN_VECTORS


def demo_texts():
    fixture=json.loads(CURATED.read_text())
    tle='\n'.join('v24\n'+x['tle_line1']+'\n'+x['tle_line2']+'   0.00  4320.00 360.00' for x in fixture['satellites'])+'\n'
    out='\n'.join(str(s['catalog_id'])+' xx\n'+'\n'.join(' '.join(str(z) for z in (v['minutes'], *v['position_km'], *v['velocity_km_s'])) for v in s['samples']) for s in fixture['satellites'])+'\n'
    return tle,out


@pytest.fixture(scope='module')
def report():
    return evaluate()


def test_defines_five_independent_audits(report):
    assert len(report['results'])==5
    assert len({x['case'] for x in report['results']})==5
    assert report['version']=='2.4.0'


def test_numerical_accuracy_and_convergence(report):
    analytic, convergence = report['results'][:2]
    assert analytic['status']=='PASS'
    assert convergence['status']=='PASS'
    assert convergence['error_reduction_factor']>8
    assert convergence['fine_max_position_error_m']<convergence['coarse_max_position_error_m']


def test_missing_full_corpus_fails_gate(report):
    assert report['results'][2]['status']=='SKIP'
    assert report['suite_status'] in ('PARTIAL','FAIL')
    assert report['release_gate']['real_flight_accuracy_validated'] is False


def test_dependency_is_never_silent_pass(report):
    assert report['results'][3]['status'] in ('PASS','SKIP','FAIL')
    assert report['results'][4]['status'] in ('PASS','SKIP','FAIL')
    assert report['passed']+report['failed']+report['skipped']==5


def test_source_hash_is_real(report):
    assert report['reference_bundle']['fixture_sha256']==hashlib.sha256(CURATED.read_bytes()).hexdigest()
    assert report['reference_bundle']['upstream_signature_verified'] is False


def test_reject_unmatched_pair():
    tle,out=demo_texts()
    with pytest.raises(ValueError,match='BOTH'):evaluate(tle_text=tle)
    with pytest.raises(ValueError,match='BOTH'):evaluate(vector_text=out)


def test_reject_bad_hash_format():
    with pytest.raises(ValueError,match='64 hexadecimal'):evaluate(expected_tle_sha256='a')


def test_reject_pin_without_source():
    with pytest.raises(ValueError,match='require supplied'):evaluate(expected_tle_sha256='0'*64)


def test_reject_bad_resolution():
    with pytest.raises(ValueError,match='Steps'):evaluate(steps_per_orbit=1201)


def test_external_corpus_imported_but_not_mislabelled_full():
    tle,out=demo_texts()
    r=evaluate(tle_text=tle,vector_text=out,
      expected_tle_sha256=hashlib.sha256(tle.encode()).hexdigest(),
      expected_vector_sha256=hashlib.sha256(out.encode()).hexdigest())
    assert r['import_summary']['samples']==28
    assert r['results'][2]['status']=='SKIP'
    assert r['results'][3]['sample_count']==28
    assert r['reference_bundle']['expected_sha256_match'] is True
    assert r['suite_status']!='PASS'


def test_bad_external_hash_fails_before_comparison():
    tle,out=demo_texts()
    with pytest.raises(ValueError,match='hash does not match'):
        evaluate(tle_text=tle,vector_text=out,expected_tle_sha256='0'*64)


def test_incomplete_reference_record_rejected():
    tle,out=demo_texts()
    with pytest.raises(ValueError):evaluate(tle_text='corrupted TLE',vector_text=out)


def test_expected_complete_corpus_threshold():
    assert MIN_CASES==33 and MIN_VECTORS==666


def test_api_health_version():
    c=TestClient(app)
    assert c.get('/api/health').json()['version']=='2.4.0'


def test_api_gate_incomplete_is_partial():
    r=TestClient(app).post('/api/verification/v24',json={})
    assert r.status_code==200
    assert r.json()['release_gate']['status']!='PASS'


def test_api_rejects_mismatched_source_pair():
    r=TestClient(app).post('/api/verification/v24',json={'tle_text':'abc'})
    assert r.status_code==422


def test_cli_status_partial_nonzero(tmp_path):
    target=tmp_path/'audit.json'
    completed=subprocess.run([sys.executable,'-m','apogee.v24_validation','--output',str(target)],
                             stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    assert completed.returncode==2,completed.stderr
    assert json.loads(target.read_text())['suite_status']=='PARTIAL'


def test_cli_explicit_partial_override(tmp_path):
    target=tmp_path/'audit.json'
    completed=subprocess.run([sys.executable,'-m','apogee.v24_validation','--allow-partial','--output',str(target)],
                             capture_output=True,text=True)
    assert completed.returncode==0,completed.stderr
    assert json.loads(target.read_text())['suite_status']=='PARTIAL'
