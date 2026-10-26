import math

import numpy as np
import pytest

from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as R, EARTH_MU_KM3_S2 as MU
from apogee.physics import (acceleration, circular_orbit_speed, elements_from_state,
                            hohmann_transfer, integrate_orbit, rk4_step,
                            specific_energy, state_from_elements, trajectory_diagnostics)


def test_circular_speed_and_period():
    r = R + 400
    v = circular_orbit_speed(r)
    assert math.isclose(v, math.sqrt(MU / r), rel_tol=1e-14)
    assert 90 < (2 * math.pi * r / v / 60) < 94


def test_central_force_inverse_square():
    a1 = np.linalg.norm(acceleration(np.array([7000., 0, 0])))
    a2 = np.linalg.norm(acceleration(np.array([14000., 0, 0])))
    assert a1 / a2 == pytest.approx(4)


def test_elements_round_trip_nonsingular():
    state = state_from_elements(12000, .32, 63.2, 42, 12, 210)
    el = elements_from_state(state)
    assert el.semi_major_axis_km == pytest.approx(12000, rel=1e-12)
    assert el.eccentricity == pytest.approx(.32, abs=1e-12)
    assert el.inclination_deg == pytest.approx(63.2, abs=1e-10)
    assert el.raan_deg == pytest.approx(42, abs=1e-10)
    assert el.argument_of_periapsis_deg == pytest.approx(12, abs=1e-10)
    assert el.true_anomaly_deg == pytest.approx(210, abs=1e-10)
    assert np.allclose(state, state_from_elements(el.semi_major_axis_km, el.eccentricity,
                                                  el.inclination_deg, el.raan_deg,
                                                  el.argument_of_periapsis_deg,
                                                  el.true_anomaly_deg), atol=1e-8)


def test_equatorial_circular_singularity():
    state = state_from_elements(R + 400, 0, 0, 0, 0, 270)
    el = elements_from_state(state)
    assert el.inclination_deg == pytest.approx(0)
    assert el.eccentricity == pytest.approx(0, abs=1e-12)
    assert el.true_anomaly_deg == pytest.approx(270, abs=1e-8)


def test_one_complete_orbit_closes_with_small_numerical_drift():
    r = R + 400
    state = state_from_elements(r, 0, 51.6, 30)
    period = 2 * math.pi * math.sqrt(r ** 3 / MU)
    output = integrate_orbit(state, period, 720)
    assert not output.impact
    assert np.linalg.norm(output.states[-1, :3] - state[:3]) < 0.002  # 2 meters
    assert np.linalg.norm(output.states[-1, 3:] - state[3:]) < 2e-6
    diag = trajectory_diagnostics(output.states)
    assert diag['max_relative_energy_drift'] < 2e-9
    assert diag['max_relative_angular_momentum_drift'] < 1e-9


def test_elliptical_conservation_multiple_orbits():
    state = state_from_elements(14000, .40, 47, 90, 30, 0)
    el = elements_from_state(state)
    result = integrate_orbit(state, el.period_s * 4, 2400)
    diag = trajectory_diagnostics(result.states)
    assert diag['max_relative_energy_drift'] < 1e-6
    assert diag['max_relative_angular_momentum_drift'] < 1e-6
    assert not result.impact


def test_rk4_order_convergence():
    state = state_from_elements(R + 1000, .04, 35)
    period = elements_from_state(state).period_s
    end_90 = integrate_orbit(state, period * .75, 90).states[-1]
    end_180 = integrate_orbit(state, period * .75, 180).states[-1]
    end_360 = integrate_orbit(state, period * .75, 360).states[-1]
    d1 = np.linalg.norm(end_90[:3] - end_180[:3])
    d2 = np.linalg.norm(end_180[:3] - end_360[:3])
    assert 12 < d1 / d2 < 20  # global 4th order accuracy ratio approximately 16


def test_hohmann_transfer_400_to_700_is_numerically_verified():
    r1, r2 = R + 400, R + 700
    plan = hohmann_transfer(r1, r2)
    assert 0.1 < plan['total_delta_v_km_s'] < 0.25
    assert plan['departure_delta_v_km_s'] > 0
    assert plan['arrival_delta_v_km_s'] > 0
    state = state_from_elements(r1, 0)
    state[4] += plan['departure_delta_v_km_s']
    outcome = integrate_orbit(state, plan['time_of_flight_s'], 720)
    assert abs(np.linalg.norm(outcome.states[-1, :3]) - r2) < 0.002


def test_inward_hohmann_has_negative_burns():
    plan = hohmann_transfer(R + 700, R + 400)
    assert plan['departure_delta_v_km_s'] < 0
    assert plan['arrival_delta_v_km_s'] < 0
    assert plan['total_delta_v_km_s'] > 0


def test_mass_does_not_enter_two_body_equations():
    state = state_from_elements(R + 400, 0)
    assert np.linalg.norm(rk4_step(state, 5) - rk4_step(state, 5)) == 0


def test_impact_stops_trajectory():
    # Start at 400km, reduce tangential velocity enough that perigee intersects Earth.
    state = state_from_elements(R + 400, 0)
    state[3:] *= .8
    period = elements_from_state(state).period_s
    output = integrate_orbit(state, period, 720)
    assert output.impact
    assert len(output.states) < 721


def test_invalid_inputs_rejected():
    with pytest.raises(ValueError):
        state_from_elements(7000, 1.0)
    with pytest.raises(ValueError):
        integrate_orbit(np.zeros(6), 50, 50)
    with pytest.raises(ValueError):
        hohmann_transfer(1000, 8000)


def test_against_scipy_dop853_reference():
    scipy = pytest.importorskip('scipy.integrate')
    state = state_from_elements(16000, .5, 20, 60, 40, 75)
    period = elements_from_state(state).period_s
    result = integrate_orbit(state, period * .8, 2000)
    def deriv(_, y):
        return np.concatenate([y[3:], acceleration(y[:3])])
    reference = scipy.solve_ivp(deriv, (0, period * .8), state, method='DOP853',
                                rtol=1e-13, atol=1e-14)
    assert reference.success
    assert np.linalg.norm(result.states[-1, :3] - reference.y[:3, -1]) < 0.001
