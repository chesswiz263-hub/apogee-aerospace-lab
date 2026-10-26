"""v0.7 numerical regressions: bounded powered targeting and Gaussian uncertainty."""
import numpy as np
import pytest
from fastapi.testclient import TestClient

from apogee.api import app
from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from apogee.finite_optimization import optimize_finite_transfer
from apogee.uncertainty import propagate_uncertainty
from apogee.optimization import optimize_impulsive_transfer


@pytest.fixture(scope='module')
def raising():
    return optimize_finite_transfer(phase_candidates=1)


@pytest.fixture(scope='module')
def lowering():
    return optimize_finite_transfer(start_altitude_km=700,target_altitude_km=400,phase_candidates=1)


@pytest.fixture(scope='module')
def mc():
    return propagate_uncertainty(monte_carlo_samples=32,steps=60)


def test_finite_raises_to_requested_apsis(raising):
    s=raising['finite_optimization']['selected']
    assert abs(s['target_apsis_residual_m'])<.1
    assert s['postburn_eccentricity']<.005
    assert s['fuel_feasible']


def test_finite_lowers_to_requested_apsis(lowering):
    s=lowering['finite_optimization']['selected']
    assert abs(s['target_apsis_residual_m'])<.1
    assert s['postburn_eccentricity']<.005


def test_engine_fuel_matches_massflow(raising):
    choice=raising['finite_optimization']['selected']
    from apogee.propulsion import G0_M_S2
    expected=850*(choice['departure_duration_s']+choice['arrival_duration_s'])/(315*G0_M_S2)
    assert choice['propellant_used_kg']==pytest.approx(expected,rel=1e-10)


def test_finite_burn_final_wet_mass(raising):
    tr=raising['trajectory'];c=raising['finite_optimization']['selected']
    assert tr['wet_mass_kg'][0]==pytest.approx(750)
    assert tr['wet_mass_kg'][-1]==pytest.approx(750-c['propellant_used_kg'],abs=1e-6)
    assert min(tr['wet_mass_kg'])>=600
    assert min(tr['altitude_km'])>150


def test_finite_is_not_claimed_as_impulsive(raising):
    impulse=optimize_impulsive_transfer(phase_candidates=1,j2_enabled=False)
    used=raising['finite_optimization']['selected']['propellant_used_kg']
    ideal=impulse['optimization']['selected']['fuel_required_kg']
    assert used>ideal
    assert 'global' in raising['warnings'][0].lower()


def test_j2_finite_case():
    d=optimize_finite_transfer(phase_candidates=1,j2_enabled=True)
    assert abs(d['finite_optimization']['selected']['target_apsis_residual_m'])<.2
    assert 'Earth J2' in d['model']['force_terms']


def test_bad_finite_inputs():
    with pytest.raises(ValueError):optimize_finite_transfer(phase_candidates=6)
    with pytest.raises(ValueError):optimize_finite_transfer(start_altitude_km=400,target_altitude_km=405)
    with pytest.raises(ValueError):optimize_finite_transfer(propellant_kg=0)


def test_fuel_limited_case_rejected():
    with pytest.raises(ValueError,match='fuel|bracket|feasible|constraint|burn'):
        optimize_finite_transfer(propellant_kg=0.5,phase_candidates=1)


def test_mc_covariance_symmetric_psd(mc):
    u=mc['uncertainty']
    for name in ('final_monte_carlo_covariance_6x6_km_units',
                 'final_linear_stm_covariance_6x6_km_units'):
        p=np.asarray(u[name]);assert p.shape==(6,6)
        assert np.allclose(p,p.T,atol=1e-12)
        assert min(np.linalg.eigvalsh(p))>=-1e-13


def test_mc_is_deterministic(mc):
    other=propagate_uncertainty(monte_carlo_samples=32,steps=60)
    assert other['uncertainty']['position_95pct_conservative_radius_m']==mc['uncertainty']['position_95pct_conservative_radius_m']


def test_mc_uncertainty_grows(mc):
    a=mc['uncertainty']['position_95pct_conservative_radius_m']
    assert a[-1]>a[0]*2
    assert all(np.isfinite(a))


def test_stm_has_proper_size(mc):
    stm=np.asarray(mc['uncertainty']['final_state_transition_matrix_6x6'])
    assert stm.shape==(6,6) and np.isfinite(stm).all()
    assert np.linalg.det(stm)>0


def test_mc_3d_radius_consistency(mc):
    u=mc['uncertainty']
    from apogee.uncertainty import CHI2_95_3D
    assert u['position_95pct_conservative_radius_m'][-1]==pytest.approx(
        u['position_max_axis_sigma_m'][-1]*np.sqrt(CHI2_95_3D),rel=1e-12)


def test_different_random_seed_changes_ensemble(mc):
    b=propagate_uncertainty(monte_carlo_samples=32,steps=60,seed=43)
    assert b['uncertainty']['position_95pct_conservative_radius_m'][-1]!=mc['uncertainty']['position_95pct_conservative_radius_m'][-1]


def test_bad_uncertainty_request():
    with pytest.raises(ValueError):propagate_uncertainty(position_sigma_m=0)
    with pytest.raises(ValueError):propagate_uncertainty(monte_carlo_samples=50)


def test_finite_api():
    r=TestClient(app).post('/api/finite/optimize',json={'phase_candidates':1})
    assert r.status_code==200,r.text
    assert r.json()['finite_optimization']['selected']['postburn_eccentricity']<.005


def test_uncertainty_api():
    r=TestClient(app).post('/api/uncertainty',json={'monte_carlo_samples':16,'steps':60})
    assert r.status_code==200,r.text
    assert len(r.json()['uncertainty']['rtn_position_sigma_m'])==61


def test_api_input_validation():
    client=TestClient(app)
    assert client.post('/api/finite/optimize',json={'phase_candidates':7}).status_code==422
    assert client.post('/api/uncertainty',json={'monte_carlo_samples':5}).status_code==422


def test_finite_independent_rk4_check(raising):
    # Independent v0.4 coupled RK4 integrator must match our DOP853 power+coast solution.
    from apogee.coupled import integrate_coupled
    from apogee.perturbations import ForceModel
    from apogee.propulsion import Burn
    choice=raising['finite_optimization']['selected'];tr=raising['trajectory']
    y0=np.r_[tr['position_km'][0],tr['velocity_km_s'][0],tr['wet_mass_kg'][0]]
    burns=[Burn(0,choice['departure_duration_s'],850,315,'prograde'),
           Burn(choice['apsis_event_s'],choice['arrival_duration_s'],850,315,'prograde')]
    comparison=integrate_coupled(y0,600,tr['time_s'][-1],350,burns,ForceModel(j2=False,drag=False))
    diff_m=np.linalg.norm(comparison.states[-1,:3]-tr['position_km'][-1])*1000
    assert diff_m<0.3
    assert comparison.states[-1,6]==pytest.approx(tr['wet_mass_kg'][-1],abs=1e-7)


def test_thrust_strategy_comparison(raising):
    o=raising['finite_optimization']
    strategies=o['thrust_strategy_comparison']
    assert len(strategies)==3
    assert strategies[1]['thrust_n']==850
    assert strategies[1]['fuel_used_kg']==pytest.approx(o['selected']['propellant_used_kg'],abs=1e-8)
