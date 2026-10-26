"""APOGEE v2.1: epoch-tagged WGS84/ITRS geometric observations.

Earth-fixed (ITRS) state observations work without optional dependencies. GCRS
states require Astropy's full coordinate transformations and bundled IERS
orientation data. No simple Earth-spin fallback is permitted. Output is
geometric *instantaneous* one-way range/range-rate, not measured radar,
GNSS, or light-time-corrected two-way range.
"""
from __future__ import annotations
from dataclasses import dataclass
from datetime import datetime, timezone
from math import radians, sin, cos, degrees, atan2, asin, sqrt
from contextlib import ExitStack
from typing import Sequence
import numpy as np

A_WGS84_M = 6378137.0
F_WGS84 = 1 / 298.257223563
E2 = F_WGS84 * (2 - F_WGS84)
ALLOWED_FRAMES = {'ITRS', 'GCRS'}


def strict_utc(value: str) -> datetime:
    if not isinstance(value, str) or not value.endswith('Z'):
        raise ValueError('Epoch must be explicit ISO-8601 UTC with trailing Z')
    try:
        dt = datetime.fromisoformat(value[:-1] + '+00:00')
    except (TypeError, ValueError) as exc:
        raise ValueError('Invalid ISO UTC epoch') from exc
    if dt.tzinfo is None or not 1973 <= dt.year <= 2035:
        raise ValueError('Epoch outside supported interval 1973–2035')
    return dt.astimezone(timezone.utc)


def check_station(station: dict):
    try:
        lat, lon, h = [float(station[k]) for k in ('latitude_deg', 'longitude_deg', 'height_m')]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Station requires latitude_deg, longitude_deg, height_m') from exc
    if not np.all(np.isfinite([lat, lon, h])) or not -90 <= lat <= 90 or not -180 <= lon <= 180 or not -500 <= h <= 10000:
        raise ValueError('Station coordinates outside supported WGS84 limits')
    return lat, lon, h


def wgs84_station_itrs(station: dict) -> np.ndarray:
    """WGS84 geodetic (deg,deg,m) -> geocentric ITRS position in km."""
    lat, lon, h = check_station(station)
    p, l = radians(lat), radians(lon)
    n = A_WGS84_M / sqrt(1-E2*sin(p)**2)
    return np.array([(n+h)*cos(p)*cos(l), (n+h)*cos(p)*sin(l), (n*(1-E2)+h)*sin(p)])/1000.0


def topocentric_observation(itrs_state: Sequence[float], station: dict) -> dict:
    """Pure WGS84/ITRS geometric slant range, range-rate, az/el.

    Velocities are ITRS coordinate derivatives in km/s. They are NOT inertial
    velocities. Fixed station velocity in ITRS is exactly zero.
    """
    y = np.asarray(itrs_state, dtype=float)
    if y.shape != (6,) or not np.all(np.isfinite(y)):
        raise ValueError('State must be six finite ITRS Cartesian km and km/s components')
    lat,lon,_ = check_station(station)
    r = wgs84_station_itrs(station)
    rel = y[:3]-r
    rho = float(np.linalg.norm(rel))
    if rho <= 1.e-6:
        raise ValueError('Satellite and station coincide; range/rate undefined')
    u = rel/rho
    p,l = radians(lat),radians(lon)
    east=np.array([-sin(l),cos(l),0.])
    north=np.array([-sin(p)*cos(l),-sin(p)*sin(l),cos(p)])
    up=np.array([cos(p)*cos(l),cos(p)*sin(l),sin(p)])
    e, n, z = [float(rel@axis) for axis in (east,north,up)]
    elevation=degrees(asin(np.clip(z/rho,-1.,1.)))
    azimuth=(degrees(atan2(e,n)) % 360.0) if np.hypot(e,n)/rho>1e-6 else None
    rate=float(u@y[3:6])*1000.0
    return dict(range_m=rho*1000.,range_rate_m_s=rate,
                elevation_deg=elevation,azimuth_deg=azimuth,
                line_of_sight_itrs=u.tolist(),station_itrs_km=r.tolist(),
                model='WGS84 ECEF/ITRS instantaneous geometric observation')


def _astropy_gcrs_to_itrs(y: np.ndarray, epoch_utc: str) -> np.ndarray:
    """Require real Earth orientation; never approximate UTC as UT1."""
    try:
        import astropy.units as u
        from astropy.time import Time
        from astropy.coordinates import GCRS, ITRS, CartesianRepresentation, CartesianDifferential
        from astropy.utils import iers
    except ImportError as exc:
        raise RuntimeError('GCRS observation requires optional Astropy and IERS data; install requirements-reference.txt') from exc
    try:
        with ExitStack() as stack:
            # No silent degraded-EOP fallback or network-dependent automatic download.
            stack.enter_context(iers.conf.set_temp('auto_download', False))
            stack.enter_context(iers.conf.set_temp('iers_degraded_accuracy', 'error'))
            t=Time(epoch_utc.replace('Z',''),format='isot',scale='utc')
            # Forces an actual UT1-UTC lookup. Extrapolation fails under strict policy.
            _ = t.ut1.jd
            rep=CartesianRepresentation(y[:3]*u.km,
                     differentials=CartesianDifferential(y[3:6]*u.km/u.s))
            result=GCRS(rep,obstime=t).transform_to(ITRS(obstime=t))
            p=np.asarray(result.cartesian.xyz.to_value(u.km),float)
            v=np.asarray(result.cartesian.differentials['s'].d_xyz.to_value(u.km/u.s),float)
            if not np.all(np.isfinite(np.r_[p,v])):
                raise ValueError('Frame conversion returned nonfinite values')
            return np.r_[p,v]
    except (ValueError,RuntimeError,KeyError) as exc:
        raise RuntimeError('Strict GCRS/ITRS transformation failed: missing, stale, or invalid IERS Earth-orientation data: '+str(exc)) from exc


def calculate_observation(epoch_utc: str, station: dict, state: Sequence[float], frame: str,
                          *, observed: dict | None = None, provenance: str='synthetic') -> dict:
    strict_utc(epoch_utc)
    if frame not in ALLOWED_FRAMES:
        raise ValueError('State frame must be explicitly ITRS or GCRS; TEME/simulator ECI is incompatible')
    if provenance not in ('synthetic','external_unverified'):
        raise ValueError('Provenance must be synthetic or external_unverified')
    y=np.asarray(state,float)
    if y.shape!=(6,) or not np.all(np.isfinite(y)):
        raise ValueError('State must have six finite values')
    y_itrs=y if frame=='ITRS' else _astropy_gcrs_to_itrs(y,epoch_utc)
    result=topocentric_observation(y_itrs,station)
    result.update(epoch_utc=epoch_utc,state_frame=frame,measurement_frame='ITRS',
                  eop_status=('not required (state supplied in ITRS)' if frame=='ITRS' else 'Astropy IERS strict transformation'),
                  provenance=provenance,validity='GEOMETRIC ONLY; NO signal light-time, atmosphere, clocks, or ground-station calibration',
                  station=dict(station),itrs_state={'position_km':y_itrs[:3].tolist(),'velocity_km_s':y_itrs[3:].tolist()})
    if observed is not None:
        try:
            rr=float(observed['range_m']);vv=float(observed['range_rate_m_s'])
        except (KeyError,ValueError,TypeError) as exc:
            raise ValueError('Observed requires range_m and range_rate_m_s') from exc
        if not np.all(np.isfinite([rr,vv])) or rr<=0:
            raise ValueError('Observed range and range-rate must be finite with positive range')
        result['residuals']={'range_m':rr-result['range_m'], 'range_rate_m_s':vv-result['range_rate_m_s']}
    return result


def compare_records(records: list[dict], provenance: str, reference_id: str) -> dict:
    if not (1<=len(records)<=200):raise ValueError('Supply between one and 200 records')
    if not isinstance(reference_id,str) or not (3<=len(reference_id)<=150):
        raise ValueError('Explicit reference_id describing the data source is required')
    if provenance not in ('synthetic','external_unverified'):
        raise ValueError('Invalid data provenance')
    rows=[]
    for x in records:
        try:
            s=x['state']
            y=list(s['position_km'])+list(s['velocity_km_s'])
            entry=calculate_observation(x['epoch_utc'],x['station'],y,x['frame'],
                   observed=x['observed'],provenance=provenance)
        except (TypeError,KeyError) as exc:
            raise ValueError('Malformed record: epoch_utc, station, state, frame, observed required') from exc
        rows.append(entry)
    frames={p['state_frame'] for p in rows}
    if len(frames)!=1:
        raise ValueError('Cannot combine records supplied in different coordinate frames')
    range_res=np.array([p['residuals']['range_m'] for p in rows]);rate_res=np.array([p['residuals']['range_rate_m_s'] for p in rows])
    return {'version':'2.1.0','records':rows,'reference_id':reference_id,'provenance':provenance,
            'frame':frames.pop(),'sample_count':len(rows),
            'statistics':{'range_rms_m':float(np.sqrt(np.mean(range_res**2))),
               'range_bias_m':float(range_res.mean()),
               'range_rate_rms_m_s':float(np.sqrt(np.mean(rate_res**2))),
               'range_rate_bias_m_s':float(rate_res.mean())},
            'validation_scope':'User-provided instantaneous geometric reference residuals; NOT externally authenticated, time-of-flight-corrected or flight qualified'}
