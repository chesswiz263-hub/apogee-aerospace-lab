"""Guard APOGEE browser mission requests against accidental GET-only CORS."""
from fastapi.testclient import TestClient
from apogee.api import app

ORIGIN = 'https://chesswiz263-hub.github.io'


def test_existing_mission_post_preflight_allowed():
    response = TestClient(app).options(
        '/api/navigation/integrity',
        headers={
            'Origin': ORIGIN,
            'Access-Control-Request-Method': 'POST',
            'Access-Control-Request-Headers': 'content-type',
        },
    )
    assert response.status_code == 200, response.text
    assert response.headers.get('access-control-allow-origin') == ORIGIN
    methods = response.headers.get('access-control-allow-methods', '')
    assert 'POST' in [part.strip() for part in methods.split(',')]


def test_existing_mission_get_preflight_allowed():
    response = TestClient(app).options(
        '/api/v25/capabilities',
        headers={'Origin': ORIGIN, 'Access-Control-Request-Method': 'GET'},
    )
    assert response.status_code == 200, response.text
    assert response.headers.get('access-control-allow-origin') == ORIGIN
