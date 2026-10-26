"""APOGEE interplanetary pilot: heliocentric Lambert targeting and 2-body propagation.

All numeric positions and velocities are heliocentric J2000 MEAN ECLIPTIC, km and km/s.
Planet centers come from APOGEE's approximate JPL *visualization* elements
(Earth uses EMB). The finite-difference velocities share that same approximation.
The Sun's Newtonian point mass is used between impulsive departure and arrival.
No departure/landing physics, gravity assists, n-body perturbations, thrust
history, finite burns, or error covariance is included. Not flight certified.
"""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
from math import acos, cos, cosh, isfinite, pi, sin, sinh, sqrt

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq
from fastapi import APIRouter, HTTPException, Query
from .v25_solar import AU_KM, _jd_utc, _planet, parse_utc

MU_SUN_KM3_S2 = 1.32712440018e11
DAY_S = 86400.0
BODIES = ('Mercury', 'Venus', 'Earth', 'Mars', 'Jupiter', 'Saturn', 'Uranus', 'Neptune')
router = APIRouter(prefix='/api/v25', tags=['APOGEE v2.5 interplanetary'])


def _planet_position_km(name: str, instant: datetime) -> np.ndarray:
    if name not in BODIES:
        raise ValueError(f'Body must be one of: {", ".join(BODIES)}')
    century = (_jd_utc(instant) - 2451545.0) / 36525.0
    return np.asarray(_planet(name, century)['position_au'], dtype=float) * AU_KM


def _planet_state(name: str, instant: datetime) -> tuple[np.ndarray, np.ndarray]:
    """Central finite difference of same approximate planet position, 5-minute span."""
    h = 150.0
    position = _planet_position_km(name, instant)
    velocity = (_planet_position_km(name, instant + timedelta(seconds=h)) -
                _planet_position_km(name, instant - timedelta(seconds=h))) / (2*h)
    return position, velocity


def _stumpff(z: float) -> tuple[float, float]:
    if z > 1e-7:
        s = sqrt(z)
        return (1-cos(s))/z, (s-sin(s))/(s*s*s)
    if z < -1e-7:
        s = sqrt(-z)
        if s > 50:
            raise ValueError('Lambert trial outside stable numerical range')
        return (cosh(s)-1)/(-z), (sinh(s)-s)/(s*s*s)
    return .5 - z/24 + z*z/720, 1/6 - z/120 + z*z/5040


def lambert(r1, r2, tof_seconds: float, mu: float = MU_SUN_KM3_S2):
    """Single-revolution, prograde (positive ecliptic z) universal-variable Lambert.

    Returns initial and terminal heliocentric vectors in km/s. Rejects nearly
    collinear geometry where this solver's transfer plane is ill-conditioned.
    """
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    if r1.shape != (3,) or r2.shape != (3,) or not np.all(np.isfinite([*r1, *r2])):
        raise ValueError('Position vectors must be finite three-vectors')
    if not (isfinite(tof_seconds) and tof_seconds > 0 and mu > 0):
        raise ValueError('Time of flight and gravitational parameter must be positive')
    a, b = np.linalg.norm(r1), np.linalg.norm(r2)
    cos_d = np.clip(float(np.dot(r1, r2) / (a*b)), -1, 1)
    sin_d = sqrt(max(0.0, 1.0-cos_d*cos_d))
    if np.cross(r1, r2)[2] < 0:
        sin_d = -sin_d  # prograde long-way geometry
    A = sin_d * sqrt(a*b/(1-cos_d)) if (1-cos_d) > 1e-12 else 0
    if abs(A) < 1e-6:
        raise ValueError('Departure/arrival geometry is nearly collinear; choose a different date or flight time')

    def time_residual(z: float) -> float:
        C, S = _stumpff(z)
        if C <= 0:
            return -tof_seconds
        y = a+b+A*(z*S-1)/sqrt(C)
        if y < 0:
            return -tof_seconds
        x = sqrt(y/C)
        return (x*x*x*S + A*sqrt(y))/sqrt(mu) - tof_seconds

    # z=0 and z->4*pi^2 describe the elliptic zero/single-revolution interval;
    # negative z supports hyperbolic fast transfers.
    low, high = -100.0, (2*pi-1e-5)**2
    # Work toward a valid lower bracket, avoiding very negative overflow.
    left = None
    for z in np.linspace(low, 0, 201):
        try:
            f = time_residual(float(z))
            if isfinite(f) and f < 0:
                left = float(z)
        except (ValueError, OverflowError):
            continue
    if left is None or time_residual(high) <= 0:
        raise ValueError('No single-revolution prograde Lambert solution for these inputs')
    z = brentq(time_residual, left, high, xtol=1e-11, maxiter=200)
    C, S = _stumpff(z)
    y = a+b+A*(z*S-1)/sqrt(C)
    f = 1-y/a
    g = A*sqrt(y/mu)
    gdot = 1-y/b
    if abs(g) < 1e-10:
        raise ValueError('Degenerate Lambert velocity solution')
    return (r2-f*r1)/g, (gdot*r2-r1)/g


def plan_transfer(origin: str, destination: str, departure: str,
                  flight_days: float = 260.0, samples: int = 180) -> dict:
    if origin not in BODIES or destination not in BODIES or origin == destination:
        raise ValueError('Select two different major planets')
    if not (20 <= flight_days <= 12000 and isfinite(flight_days)):
        raise ValueError('flight_days must be between 20 and 12000')
    if not (32 <= samples <= 450):
        raise ValueError('samples must be between 32 and 450')
    dep = parse_utc(departure)
    arr = dep + timedelta(days=flight_days)
    if not (1800 <= dep.year <= 2050 and 1800 <= arr.year <= 2050):
        raise ValueError('Both dates must be within JPL approximate-elements range 1800–2050')
    r1, vp1 = _planet_state(origin, dep)
    r2, vp2 = _planet_state(destination, arr)
    tof = flight_days * DAY_S
    v1, v2 = lambert(r1, r2, tof)
    y0 = np.concatenate([r1, v1])

    def rhs(_t, state):
        pos = state[:3]
        rad = float(np.linalg.norm(pos))
        return [*state[3:], *(-MU_SUN_KM3_S2 * pos/rad**3)]

    times = np.linspace(0, tof, samples)
    result = solve_ivp(rhs, (0.0, tof), y0, method='DOP853', t_eval=times,
                       rtol=1e-11, atol=1e-10)
    if not result.success or result.y.shape != (6, samples):
        raise RuntimeError('Interplanetary trajectory integration failed')
    pos = result.y[:3, :].T
    last = pos[-1]
    miss = float(np.linalg.norm(last - r2))
    # Orbital transfer target is a planetary CENTER in a two-body approximation;
    # no launch escape, encounter capture or impact modeling is implied.
    return {
        'model': 'single-revolution prograde Lambert, heliocentric Sun-only DOP853',
        'version': '2.5-interplanetary-pilot',
        'origin': origin, 'destination': destination,
        'departure_utc': dep.isoformat().replace('+00:00','Z'),
        'arrival_utc': arr.isoformat().replace('+00:00','Z'),
        'flight_days': flight_days,
        'frame': 'heliocentric J2000 mean ecliptic, matching v25_solar approximate elements',
        'units': {'position':'AU','velocity':'km/s','time':'days'},
        'departure_v_infinity_km_s': round(float(np.linalg.norm(v1-vp1)), 6),
        'arrival_v_infinity_km_s': round(float(np.linalg.norm(v2-vp2)), 6),
        'departure_heliocentric_velocity_km_s': v1.tolist(),
        'departure_planet_velocity_km_s': vp1.tolist(),
        'arrival_miss_km': round(miss, 3),
        'start_position_au': (r1/AU_KM).tolist(),
        'destination_at_arrival_au': (r2/AU_KM).tolist(),
        'trajectory': [{'day':round(float(t/DAY_S),6),
                        'position_au':(p/AU_KM).tolist()}
                       for t,p in zip(times,pos)],
        'limitations': [
            'Planet positions are JPL approximate elements, not precision DE ephemerides',
            'Earth entry represents Earth-Moon barycenter, not Earth geocenter',
            'Sun-only gravity between ideal instantaneous departure and arrival',
            'Delta-v values are planetary hyperbolic excess speeds, NOT surface-launch delta-v',
            'No planetary capture, atmosphere, finite thrust, navigation uncertainty, or gravity assists',
            'A trajectory design study, not a flight-certified mission plan'
        ]
    }


@router.get('/interplanetary/transfer')
def transfer_route(origin: str = 'Earth', destination: str = 'Mars',
                   departure: str | None = None,
                   flight_days: float = Query(260, ge=20, le=12000),
                   samples: int = Query(180, ge=32, le=450)):
    if departure is None:
        departure = datetime.now(timezone.utc).isoformat()
    try:
        return plan_transfer(origin, destination, departure, flight_days, samples)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

@router.get('/satellites/{catalog_id}/heliocentric')
def satellite_heliocentric(catalog_id: int, at: str | None = None):
    """One live CelesTrak SGP4 state transformed from TEME to a Sun-centered frame.

    The satellite's position is Earth-near and visually overlaps Earth at AU scale.
    This returns real AU coordinates, never artificially displaces the satellite.
    """
    if not 1 <= catalog_id <= 999_999_999:
        raise HTTPException(422, 'Catalog number must be a positive nine-digit-or-less integer')
    try:
        import astropy.units as u
        from astropy.coordinates import (CartesianDifferential, CartesianRepresentation,
                                         HeliocentricMeanEcliptic, TEME)
        from astropy.time import Time
        from astropy.utils import iers
    except ImportError as exc:
        raise HTTPException(503, 'Astropy required for physically correct TEME-to-heliocentric conversion') from exc
    from .v25_operations import satellite_state
    raw = satellite_state(catalog_id, at=at)
    when = parse_utc(raw['time_utc'])
    teme_state = raw['frames']['TEME']
    pos = np.asarray(teme_state['position_km'], float)
    vel = np.asarray(teme_state['velocity_km_s'], float)
    try:
        iers.conf.auto_download = False
        t = Time(when, scale='utc')
        coord = TEME(CartesianRepresentation(pos*u.km).with_differentials(
            CartesianDifferential(vel*u.km/u.s)), obstime=t)
        target = HeliocentricMeanEcliptic(equinox=Time('J2000'))
        helio = coord.transform_to(target)
        xyz = helio.cartesian.xyz.to_value(u.km)/AU_KM
    except Exception as exc:
        raise HTTPException(503, f'Astropy coordinate conversion unavailable: {str(exc)[:160]}') from exc
    return {
        'catalog_number':catalog_id, 'name':raw['name'],
        'time_utc':raw['time_utc'], 'element_age_days':raw['element_age_days'],
        'stale_or_far_from_epoch':raw['stale_or_far_from_epoch'],
        'frame':'heliocentric mean ecliptic J2000 (Astropy TEME transform)',
        'position_au':xyz.tolist(),
        'source':'CelesTrak GP OMM + SGP4 TEME + Astropy',
        'limitations':['Visualized Earth-near at Solar System scale (not visibly separate without magnification)',
                       'OMM/SGP4 prediction, not independently observed position',
                       'IERS offline Earth orientation data might degrade frame accuracy']}
