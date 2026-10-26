import pytest
from fastapi.testclient import TestClient
from apogee.api import app

client = TestClient(app)


def test_healthy():
    assert client.get('/api/health').json()['status'] == 'ok'
    assert client.get('/').status_code == 200


def test_propagate_bounded_response_and_metrics():
    response = client.post('/api/propagate', json={"perigee_altitude_km": 400,
                              "eccentricity": 0, "duration_orbits": 1,
                              "samples_per_orbit": 240})
    assert response.status_code == 200, response.text
    body = response.json()
    assert body['metrics']['samples'] == 241
    assert len(body['trajectory']['position_km']) == 241
    assert body['metrics']['max_relative_energy_drift'] < 1e-6
    assert body['orbital_elements']['period_s'] > 5000


def test_transfer_arrives_at_target():
    response = client.post('/api/transfer', json={"start_altitude_km": 400,
                                                   "target_altitude_km": 700})
    assert response.status_code == 200, response.text
    assert abs(response.json()['transfer']['arrival_radius_error_km']) < .01


def test_invalid_requests_produce_validation_errors():
    assert client.post('/api/propagate', json={"eccentricity": 1.2}).status_code == 422
    assert client.post('/api/transfer', json={"start_altitude_km": -1}).status_code == 422
    assert client.post('/api/propagate', json={"spacecraft_mass_kg": -1}).status_code == 422
