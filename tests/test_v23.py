"""APOGEE v2.3 multiregime reference and corpus import regression."""
from __future__ import annotations

import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.v23_validation import CURATED, compare_many, parse_vallado_raw, astropy_roundtrip, validation_report


def fixture():
    return json.loads(CURATED.read_text())


def source_text():
    data=fixture()
    tle='\n'.join('# published verification data\n'+x['tle_line1']+'\n'+x['tle_line2']+'      0.0     4320.0    360.0'
                 for x in data['satellites'])+'\n'
    out='\n'.join(str(x['catalog_id'])+' xx\n'+ '\n'.join(' '.join(f'{n:.11f}' for n in
                 [v['minutes'],*v['position_km'],*v['velocity_km_s']]) for v in x['samples'])
                  for x in data['satellites'])+'\n'
    return tle,out


def test_curated_catalogs_and_ranges():
    x=fixture(); assert [s['catalog_id'] for s in x['satellites']]==[5,4632,6251,8195]
    assert sum(len(s['samples']) for s in x['satellites'])==28
    assert any(v['minutes']<0 for v in x['satellites'][1]['samples'])
    assert any(v['minutes']>=4320 for v in x['satellites'][0]['samples'])


def test_optional_dependency_not_claimed_to_pass():
    r=validation_report()
    assert r['version']=='2.3.0' # legacy v23 report retains its version
    assert r['results'][0]['status']=='PASS'
    assert r['results'][1]['status'] in {'PASS','SKIP'}
    assert r['results'][2]['status'] in {'PASS','SKIP'}
    assert r['suite_status']==('PASS' if r['passed']==3 else 'PARTIAL')


def test_multi_case_summary():
    r=compare_many(fixture())
    assert r['cases']==4 and r['sample_count']==28
    assert len(r['by_satellite'])==4
    assert all(row['status'] in {'PASS','SKIP'} for row in r['by_satellite'])


def test_raw_ingestion():
    tle,out=source_text(); imported=parse_vallado_raw(tle,out)
    assert imported['samples']==28 and imported['cases_with_vectors']==4
    assert imported['error_lines_skipped']==0
    assert imported['reference']['frame']=='TEME'
    assert compare_many(imported['reference'])['sample_count']==28
    assert len(imported['reference']['provenance']['input_tle_sha256'])==64


def test_import_strips_verification_schedule_without_corrupting_tle():
    tle,out=source_text()
    data=parse_vallado_raw(tle,out)['reference']['satellites']
    for row in data:
        assert len(row['tle_line1'])==69 and len(row['tle_line2'])==69


def test_ingestion_error_only_case_not_counted_pass():
    tle,out=source_text()
    first=fixture()['satellites'][0]['catalog_id']
    out=out.replace(f'{first} xx\n',f'{first} xx\nxx error 6\n',1)
    p=parse_vallado_raw(tle,out)
    assert p['error_lines_skipped']==1 and p['samples']==28


@pytest.mark.parametrize('mutate',[lambda a,b:('garbage',b),
                                   lambda a,b:(a,'garbage'),
                                   lambda a,b:(a,b.replace('4632 xx','9999 xx',1)),
                                   lambda a,b:(a,b.replace('6251 xx\n','6251 xx\nnot-a-state\n',1))])
def test_bad_corpus_rejected(mutate):
    tle,out=source_text();a,b=mutate(tle,out)
    with pytest.raises(ValueError):parse_vallado_raw(a,b)


def test_full_corpus_import_api():
    tle,out=source_text()
    response=TestClient(app).post('/api/verification/vallado-import',json={'tle_text':tle,'output_text':out})
    assert response.status_code==200,response.text[:500]
    assert response.json()['import_summary']['samples']==28
    assert response.json()['comparison']['status'] in {'PASS','SKIP'}


def test_import_api_malformed():
    response=TestClient(app).post('/api/verification/vallado-import',json={'tle_text':'abc','output_text':'def'})
    assert response.status_code==422


def test_v23_api_health():
    c=TestClient(app)
    assert c.get('/api/health').json()['version']=='2.4.0'
    r=c.post('/api/verification/v23',json={'steps_per_orbit':240,'orbits':2})
    assert r.status_code==200
    assert r.json()['results'][1]['cases']==4
    assert r.json()['results'][1]['sample_count']==28


def test_v23_api_rejects_outside_workload():
    assert TestClient(app).post('/api/verification/v23',json={'steps_per_orbit':99999,'orbits':2}).status_code==422


def test_frame_optional_is_not_fake_pass():
    result=astropy_roundtrip()
    assert result['status'] in {'PASS','SKIP'}
    if result['status']=='SKIP': assert 'reason' in result
    else: assert result['position_roundtrip_error_m']<0.05


def test_ui_contains_v23_and_import_controls():
    html=TestClient(app).get('/').text
    assert 'Benchmarks v2.4' in html
    assert 'vb-tle-file' in html and 'vb-output-file' in html


def test_import_rejects_corrupt_tle_checksum():
    tle,out=source_text()
    tle=tle.replace('00179.78495062','00179.78495063',1)
    with pytest.raises(ValueError,match='checksum'):
        parse_vallado_raw(tle,out)
