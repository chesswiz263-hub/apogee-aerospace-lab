"""v0.5 regression tests: numerical targeting, frames and strict reference data."""
import numpy as np
import pytest
from fastapi.testclient import TestClient

from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as RE, EARTH_MU_KM3_S2 as MU
from apogee.physics import hohmann_transfer
from apogee.frames import spin_transform, EARTH_ANGULAR_VELOCITY_RAD_S as OMEGA
from apogee.targeting import solve_apsis_target
from apogee.ephemeris import parse_horizons_vectors, format_reference_vectors, METADATA
from apogee.api import app

CLIENT=TestClient(app)

SYNTHETIC_HORIZONS='''Target body name: SYNTHETIC TEST POINT
Center body name: Earth (399)
Output units: KM-S
$$SOE
2461000.500000000, A.D. synthetic only, 6.778137D+03, 0.0D+00, 0.0D+00, 0.0D+00, 7.6686D+00, 0.0D+00,
2461000.500694444, A.D. synthetic only, 6.760000D+03, 4.596000D+02, 0.0D+00, -5.2000D-01, 7.6400D+00, 0.0D+00,
$$EOE
'''

@pytest.mark.parametrize('start,target',[(400,700),(700,400),(400,200),(400,1500)])
def test_bisection_matches_hohmann_independently(start,target):
    r=solve_apsis_target(start,target,samples=720)
    analytical=hohmann_transfer(RE+start,RE+target)
    assert r['iterations']>4
    assert abs(r['solver_residual_km'])<1e-7
    assert abs(r['total_delta_v_m_s']-analytical['total_delta_v_km_s']*1000)<1e-5
    assert abs(r['numerical_arrival_radius_error_km'])<1e-4
    assert r['post_circularization_eccentricity']<1e-6
    assert np.sign(r['departure_delta_v_m_s'])==np.sign(target-start)


def test_targeting_reports_no_fuel():
    r=solve_apsis_target(400,45000,propellant_kg=1)
    assert not r['fuel_budget']['feasible']
    assert r['fuel_budget']['propellant_margin_kg']<0


def test_targeting_rejects_invalid_inputs():
    for kwargs in [{'target_altitude_km':400},{'start_altitude_km':100},
                   {'dry_mass_kg':0},{'samples':50},{'target_altitude_km':float('nan')}]:
        with pytest.raises(ValueError):solve_apsis_target(**kwargs)


def test_spin_roundtrip_position_and_velocity():
    rng=np.random.default_rng(0)
    for angle in [0,42,-181,359]:
        s=np.r_[rng.uniform(-7000,7000,3),rng.uniform(-8,8,3)]
        ecef=spin_transform(s,angle,10230,'eci_to_ecef')
        actual=spin_transform(ecef,angle,10230,'ecef_to_eci')
        np.testing.assert_allclose(actual,s,rtol=0,atol=1e-11)


def test_spin_co_rotating_ground_point_has_zero_velocity():
    r=np.array([RE,0,0])
    inertial=np.r_[r,np.cross([0,0,OMEGA],r)]
    transformed=spin_transform(inertial,0,0,'eci_to_ecef')
    np.testing.assert_allclose(transformed[3:],0,atol=1e-13)
    np.testing.assert_allclose(transformed[:3],r,atol=1e-13)


def test_spin_rotates_axes_by_90_degrees():
    ecef=spin_transform([1,0,0,0,0,0],90,0)
    np.testing.assert_allclose(ecef[:3],[0,-1,0],atol=1e-14)


def test_horizons_import_sample_format_and_time():
    parsed=parse_horizons_vectors(SYNTHETIC_HORIZONS,METADATA)
    assert parsed.states.shape==(2,6)
    assert parsed.states[0,0]==6778.137
    assert (parsed.jd_tdb[-1]-parsed.jd_tdb[0])*86400==pytest.approx(60,abs=.001)
    result=format_reference_vectors(parsed)
    assert result['metrics']['conservation_applicable'] is False
    assert result['reference']['metadata']['frame']=='ICRF'
    assert 'provenance' in result['model']['name'] or 'unverified' in result['model']['name']


@pytest.mark.parametrize('raw,meta',[
    (SYNTHETIC_HORIZONS,{**METADATA,'plane':'ECLIPTIC'}),
    (SYNTHETIC_HORIZONS.replace('Earth (399)','Moon (301)'),METADATA),
    (SYNTHETIC_HORIZONS.replace('Output units: KM-S','Output units: AU-D'),METADATA),
    (SYNTHETIC_HORIZONS.replace('$$SOE','missing'),METADATA),
    (SYNTHETIC_HORIZONS.replace('2461000.500694444','2461000.500000000'),METADATA),
    (SYNTHETIC_HORIZONS.replace('6.760000D+03','nan'),METADATA),
])
def test_horizons_rejects_incompatible_or_corrupt(raw,meta):
    with pytest.raises(ValueError):parse_horizons_vectors(raw,meta)


def test_target_api_and_reports():
    result=CLIENT.post('/api/target',json={}).json()
    assert result['targeting']['target_apsis']=='apoapsis'
    assert len(result['events'])==2
    assert len(result['trajectory']['position_km'])==501
    assert abs(result['targeting']['numerical_arrival_radius_error_km'])<1e-4
    assert result['targeting']['fuel_budget']['feasible']


def test_target_api_input_validation():
    assert CLIENT.post('/api/target',json={'start_altitude_km':400,'target_altitude_km':400}).status_code==422


def test_spin_api():
    result=CLIENT.post('/api/frames/spin',json={'state':[RE,0,0,0,0,0],'greenwich_angle_deg_at_t0':0}).json()
    assert result['state'][4]==pytest.approx(-RE*OMEGA)
    assert 'uniform' in result['frame_model']


def test_horizons_import_api():
    result=CLIENT.post('/api/ephemeris/import',json={'text':SYNTHETIC_HORIZONS,'metadata':METADATA})
    assert result.status_code==200,result.text
    payload=result.json()
    assert payload['reference']['epoch_start_jd_tdb']==2461000.5
    assert payload['model']['integrator'].startswith('none')


def test_horizons_wrong_frame_api():
    result=CLIENT.post('/api/ephemeris/import',json={'text':SYNTHETIC_HORIZONS,
       'metadata':{**METADATA,'frame':'TEME'}})
    assert result.status_code==422


def test_optional_astropy_unavailable_graceful():
    from apogee.frames import astropy_available
    if not astropy_available():
        payload={'state_teme':[6778,0,0,0,7.6,0],'epoch_utc':'2026-10-23T12:00:00+00:00','destination':'itrs'}
        result=CLIENT.post('/api/frames/teme',json=payload)
        assert result.status_code==503
        assert 'astropy' in result.text.lower() or 'frames' in result.text.lower()


def test_health_05_and_html_controls():
    info=CLIENT.get('/api/health').json()
    assert tuple(map(int, info['version'].split('.'))) >= (0,6,0)
    page=CLIENT.get('/').text
    for selector in ['mode-target','target-form','mode-reference','reference-form','f-transform']:
        assert selector in page


def test_step_refinement_improves_analytical_orbit():
    from apogee.validation import run_convergence
    result=run_convergence()
    errors=[v['position_error_m'] for v in result['resolutions']]
    assert all(a>b for a,b in zip(errors,errors[1:]))
    assert all(v>10 for v in result['position_error_ratios'])
    assert errors[-1]<.2


def test_target_reports_refinement_diagnostic():
    result=solve_apsis_target(400,700,samples=500)
    assert result['step_refinement_position_difference_m']>=0
    assert result['step_refinement_position_difference_m']<.02
