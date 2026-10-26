from datetime import datetime
from unittest.mock import patch
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.satellites import validate_omm, fetch_catalog, propagate_omm

SAMPLE={'OBJECT_NAME':'Educational synthetic record','NORAD_CAT_ID':99999,
 'EPOCH':'2026-10-23T12:00:00.000000Z','MEAN_MOTION':15.5,
 'ECCENTRICITY':.0004,'INCLINATION':51.6,'RA_OF_ASC_NODE':32.3,
 'ARG_OF_PERICENTER':111.3,'MEAN_ANOMALY':85.2,
 'BSTAR':.0001}
CLIENT=TestClient(app)


def test_validated_omm_epoch_formatted_for_library():
    rec=validate_omm(SAMPLE)
    assert rec['EPOCH']=='2026-10-23T12:00:00.000000'
    assert rec['BSTAR']==.0001
    assert rec['MEAN_MOTION_DOT']==0


def test_invalid_omm_missing_and_bad_orbit():
    with pytest.raises(ValueError,match='Missing OMM'):
        validate_omm({'OBJECT_NAME':'Bad'})
    with pytest.raises(ValueError,match='eccentricity'):
        validate_omm({**SAMPLE,'ECCENTRICITY':1.1})
    with pytest.raises(ValueError,match='epoch'):
        validate_omm({**SAMPLE,'EPOCH':'tomorrow'})


def test_catalog_id_validation_before_external_request():
    with pytest.raises(ValueError,match='Invalid catalog'):
        fetch_catalog(0)
    assert CLIENT.get('/api/catalog/0').status_code==502


def test_missing_sgp4_returns_actionable_api_error():
    # This runtime has no SGP4 dependency, while user installer installs it.
    from apogee.satellites import sgp4_available
    if sgp4_available():pytest.skip('sgp4 installed')
    r=CLIENT.post('/api/satellite/propagate',json={'omm':SAMPLE,'duration_minutes':20,'samples':50})
    assert r.status_code==503
    assert 'sgp4' in r.json()['detail'].lower()


def test_satellite_validation_runs_before_optional_import():
    response=CLIENT.post('/api/satellite/propagate',json={'omm':{},'duration_minutes':20})
    assert response.status_code==422
