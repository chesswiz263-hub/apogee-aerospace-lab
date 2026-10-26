"""Two-body orbital physics with an explicit fixed-step RK4 integrator.

No proprietary propagation library or physics shortcut is used here.
Coordinates are inertial Cartesian axes (km); velocities are in km/s.
The Earth lies at the inertial origin. No rotating-Earth transform is implied.
"""

from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin, sqrt

import numpy as np

from .constants import EARTH_EQUATORIAL_RADIUS_KM as R_EARTH
from .constants import EARTH_MU_KM3_S2 as MU


@dataclass(frozen=True)
class ClassicalElements:
    semi_major_axis_km: float
    eccentricity: float
    inclination_deg: float
    raan_deg: float
    argument_of_periapsis_deg: float
    true_anomaly_deg: float
    period_s: float | None
    periapsis_altitude_km: float
    apoapsis_altitude_km: float | None

    def to_dict(self) -> dict:
        return self.__dict__.copy()


def _norm(vector: np.ndarray) -> float:
    return float(np.linalg.norm(vector))


def _angle_deg(x: float) -> float:
    return float(np.degrees(x) % 360.0)


def _clamp(x: float) -> float:
    return min(1.0, max(-1.0, x))


def state_from_elements(
    semi_major_axis_km: float,
    eccentricity: float,
    inclination_deg: float = 0,
    raan_deg: float = 0,
    argument_of_periapsis_deg: float = 0,
    true_anomaly_deg: float = 0,
    mu: float = MU,
) -> np.ndarray:
    """Keplerian elements -> Cartesian [rx,ry,rz,vx,vy,vz].

    Angle convention: right-handed 3-1-3 Euler rotation from perifocal PQW
    into the chosen inertial XYZ axes. Only bound elliptical inputs accepted.
    """
    a, e = semi_major_axis_km, eccentricity
    if not (np.isfinite(a) and a > 0 and np.isfinite(e) and 0 <= e < 1):
        raise ValueError("Expected a positive semi-major axis and 0 <= e < 1")
    p = a * (1 - e * e)
    nu = np.radians(true_anomaly_deg)
    radius = p / (1 + e * cos(nu))
    r_pqw = np.array([radius * cos(nu), radius * sin(nu), 0.0])
    v_pqw = sqrt(mu / p) * np.array([-sin(nu), e + cos(nu), 0.0])
    inc, raan, argp = np.radians([inclination_deg, raan_deg, argument_of_periapsis_deg])
    cO, sO = cos(raan), sin(raan)
    ci, si = cos(inc), sin(inc)
    cw, sw = cos(argp), sin(argp)
    # R_z(RAAN) R_x(inclination) R_z(argument of periapsis)
    rotation = np.array([
        [cO * cw - sO * sw * ci, -cO * sw - sO * cw * ci, sO * si],
        [sO * cw + cO * sw * ci, -sO * sw + cO * cw * ci, -cO * si],
        [sw * si, cw * si, ci],
    ])
    return np.hstack([rotation @ r_pqw, rotation @ v_pqw])


def elements_from_state(state: np.ndarray, mu: float = MU) -> ClassicalElements:
    """Cartesian -> osculating elements; stable singularity conventions.

    For circular orbits: argument of periapsis=0, true anomaly is the
    argument of latitude (or true longitude for equatorial circular orbits).
    For equatorial non-circular orbits: RAAN=0, arg periapsis=longitude of
    periapsis. For unbound orbits: period/apoapsis are None.
    """
    s = np.asarray(state, dtype=float)
    r, v = s[:3], s[3:]
    rn, vn = _norm(r), _norm(v)
    if rn == 0:
        raise ValueError("A zero position vector is not a valid orbit")
    h = np.cross(r, v)
    hn = _norm(h)
    if hn < 1e-12:
        raise ValueError("A purely radial trajectory has undefined orbital plane")
    n = np.cross([0.0, 0.0, 1.0], h)
    nn = _norm(n)
    evec = np.cross(v, h) / mu - r / rn
    e = _norm(evec)
    energy = vn * vn / 2 - mu / rn
    a = -mu / (2 * energy) if abs(energy) > 1e-14 else float("inf")
    inc = acos(_clamp(h[2] / hn))
    threshold = 1e-9
    raan = atan2(n[1], n[0]) % (2 * pi) if nn > threshold else 0.0
    if e > threshold:
        if nn > threshold:
            argp = atan2(np.dot(np.cross(n, evec), h) / hn, np.dot(n, evec)) % (2 * pi)
        else:
            argp = atan2(evec[1], evec[0]) % (2 * pi)
            # Retrograde equatorial convention is not unique.
            if h[2] < 0:
                argp = (-argp) % (2 * pi)
        nu = atan2(np.dot(np.cross(evec, r), h) / hn, np.dot(evec, r)) % (2 * pi)
    else:
        argp = 0.0
        if nn > threshold:
            nu = atan2(np.dot(np.cross(n, r), h) / hn, np.dot(n, r)) % (2 * pi)
        else:
            nu = atan2(r[1], r[0]) % (2 * pi)
            if h[2] < 0:
                nu = (-nu) % (2 * pi)
    periapsis = hn * hn / (mu * (1 + e)) - R_EARTH
    bound = energy < 0 and e < 1
    apoapsis = a * (1 + e) - R_EARTH if bound else None
    period = 2 * pi * sqrt(a ** 3 / mu) if bound else None
    return ClassicalElements(float(a), float(e), _angle_deg(inc), _angle_deg(raan),
                             _angle_deg(argp), _angle_deg(nu), float(period) if period else None,
                             float(periapsis), float(apoapsis) if apoapsis is not None else None)


def acceleration(position_km: np.ndarray, mu: float = MU) -> np.ndarray:
    rn = _norm(position_km)
    if rn <= 0:
        raise ValueError("Singularity at center of central body")
    return -mu * np.asarray(position_km, dtype=float) / rn ** 3


def state_derivative(state: np.ndarray, mu: float = MU) -> np.ndarray:
    return np.concatenate((state[3:], acceleration(state[:3], mu)))


def rk4_step(state: np.ndarray, dt_s: float, mu: float = MU) -> np.ndarray:
    """Standard 4-stage Runge-Kutta, fixed time step."""
    k1 = state_derivative(state, mu)
    k2 = state_derivative(state + dt_s * k1 / 2, mu)
    k3 = state_derivative(state + dt_s * k2 / 2, mu)
    k4 = state_derivative(state + dt_s * k3, mu)
    return state + dt_s * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def specific_energy(state: np.ndarray, mu: float = MU) -> float:
    return float(np.dot(state[3:], state[3:]) / 2 - mu / _norm(state[:3]))


def specific_angular_momentum(state: np.ndarray) -> np.ndarray:
    return np.cross(state[:3], state[3:])


@dataclass
class IntegrationResult:
    time_s: np.ndarray
    states: np.ndarray
    impact: bool
    step_s: float
    requested_duration_s: float


def integrate_orbit(initial_state: np.ndarray, duration_s: float, steps: int, mu: float = MU,
                    radius_km: float = R_EARTH) -> IntegrationResult:
    """Integrate for a fixed number of steps or stop at the first sampled impact.

    Impact is detected at the first state inside Earth's nominal sphere;
    continuous-event localization is intentionally not claimed in v0.1.
    """
    if not (np.isfinite(duration_s) and duration_s > 0 and 1 <= steps <= 10000):
        raise ValueError("Duration must be positive and steps must lie in [1, 10000]")
    initial = np.asarray(initial_state, dtype=float)
    if initial.shape != (6,) or not np.all(np.isfinite(initial)):
        raise ValueError("Initial state must be six finite Cartesian components")
    if _norm(initial[:3]) <= radius_km:
        raise ValueError("Initial spacecraft state is at or below Earth's surface")
    dt = float(duration_s / steps)
    states = np.empty((steps + 1, 6), dtype=float)
    times = np.linspace(0.0, duration_s, steps + 1)
    states[0] = initial
    for i in range(steps):
        states[i + 1] = rk4_step(states[i], dt, mu)
        if not np.all(np.isfinite(states[i + 1])):
            raise ValueError("Integration became non-finite; choose smaller time steps")
        if _norm(states[i + 1, :3]) <= radius_km:
            return IntegrationResult(times[:i+2].copy(), states[:i+2].copy(), True, dt, duration_s)
    return IntegrationResult(times, states, False, dt, duration_s)


def trajectory_diagnostics(states: np.ndarray, mu: float = MU) -> dict:
    """Relative drift of invariants of the two-body problem.

    For vanishing initial specific energy, use a |mu/r| scaling so the
    diagnostics remain defined; normal bound orbits use |E(0)|.
    """
    r = np.linalg.norm(states[:, :3], axis=1)
    v2 = np.sum(states[:, 3:] ** 2, axis=1)
    energies = v2 / 2 - mu / r
    h = np.linalg.norm(np.cross(states[:, :3], states[:, 3:]), axis=1)
    energy_scale = max(abs(float(energies[0])), mu / float(r[0]) * 1e-12)
    h_scale = max(float(h[0]), 1e-15)
    e_drift = np.abs((energies - energies[0]) / energy_scale)
    h_drift = np.abs((h - h[0]) / h_scale)
    return {
        "specific_energy_km2_s2": float(energies[0]),
        "specific_angular_momentum_km2_s": float(h[0]),
        "max_relative_energy_drift": float(np.max(e_drift)),
        "max_relative_angular_momentum_drift": float(np.max(h_drift)),
        "relative_energy_drift": e_drift.tolist(),
        "relative_angular_momentum_drift": h_drift.tolist(),
    }


def circular_orbit_speed(radius_km: float, mu: float = MU) -> float:
    if radius_km <= 0:
        raise ValueError("Radius must be positive")
    return sqrt(mu / radius_km)


def hohmann_transfer(start_radius_km: float, target_radius_km: float, mu: float = MU) -> dict:
    """Coplanar two-impulse Hohmann transfer between circular orbits.

    Delta-v signs represent tangential impulses along direction of motion.
    Returns ideal impulsive values; finite burn, mass, drag not modeled.
    """
    r1, r2 = float(start_radius_km), float(target_radius_km)
    if min(r1, r2) <= R_EARTH or max(r1, r2) > 200000:
        raise ValueError("Both orbital radii must be above Earth's surface and <=200,000 km")
    at = (r1 + r2) / 2
    v1 = circular_orbit_speed(r1, mu)
    v2 = circular_orbit_speed(r2, mu)
    vt1 = sqrt(mu * (2 / r1 - 1 / at))
    vt2 = sqrt(mu * (2 / r2 - 1 / at))
    dv1 = vt1 - v1
    dv2 = v2 - vt2
    tof = pi * sqrt(at ** 3 / mu)
    return {
        "transfer_semi_major_axis_km": at,
        "departure_delta_v_km_s": dv1,
        "arrival_delta_v_km_s": dv2,
        "total_delta_v_km_s": abs(dv1) + abs(dv2),
        "time_of_flight_s": tof,
        "departure_circular_speed_km_s": v1,
        "arrival_circular_speed_km_s": v2,
    }
