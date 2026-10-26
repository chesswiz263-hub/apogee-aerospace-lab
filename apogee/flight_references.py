"""Absolute-time reference geometry, opt-in and dependency-gated.

The GCRS/ITRS six-state transform uses Astropy and IERS data. The Sun vector
uses Astropy ERFA geocentric ephemerides. IGRF-14 is evaluated by ppigrf in
ITRS geocentric spherical coordinates. This is a reference *diagnostic*;
existing APOGEE_IDEAL_ECI trajectories are NOT automatically GCRS states.
All vectors use km, km/s, and tesla as stated, with timezone-aware UTC epochs.
"""
from datetime import datetime, timezone
import numpy as np


def utc_epoch(value):
    if not isinstance(value, str):
        raise ValueError('UTC epoch must be an ISO 8601 string with an offset')
    try:
        dt=datetime.fromisoformat(value.replace('Z','+00:00'))
    except ValueError as exc:
        raise ValueError('Invalid ISO 8601 epoch') from exc
    if dt.tzinfo is None or dt.utcoffset() is None:
        raise ValueError('Epoch must include timezone; naive datetimes are not accepted')
    dt=dt.astimezone(timezone.utc)
    if not 2000 <= dt.year <= 2030:
        raise ValueError('Reference laboratory accepts UTC dates from 2000 through 2030')
    return dt


def _vector3(value, name):
    arr=np.asarray(value,dtype=float)
    if arr.shape!=(3,) or not np.isfinite(arr).all():
        raise ValueError(f'{name} must be three finite numbers')
    return arr


def components_to_cartesian(br_nT, btheta_nT, bphi_nT, theta_deg, lon_deg):
    """Convert geocentric IGRF [radial, south, east] in nT to ITRS XYZ tesla."""
    th=np.deg2rad(theta_deg); ph=np.deg2rad(lon_deg)
    er=np.array([np.sin(th)*np.cos(ph),np.sin(th)*np.sin(ph),np.cos(th)])
    et=np.array([np.cos(th)*np.cos(ph),np.cos(th)*np.sin(ph),-np.sin(th)])
    ep=np.array([-np.sin(ph),np.cos(ph),0.])
    return 1.e-9*(float(br_nT)*er+float(btheta_nT)*et+float(bphi_nT)*ep)


def reference_geometry(epoch_utc, position_km, velocity_km_s, input_frame='GCRS',
                       include_igrf=False):
    """Return absolute epoch GCRS/ITRS state and Sun vector; optional IGRF-14.

    No silent fallback is performed: missing Astropy or IGRF support raises a
    descriptive error. Model quality depends on IERS tables available offline.
    """
    epoch=utc_epoch(epoch_utc)
    r=_vector3(position_km,'Position');v=_vector3(velocity_km_s,'Velocity')
    if input_frame not in ('GCRS','ITRS'):
        raise ValueError('Only GCRS or ITRS are accepted, not TEME/ICRF/IDEAL_ECI')
    if not 6300 < np.linalg.norm(r) < 100000:
        raise ValueError('Position radius must be between 6300 and 100000 km')
    if np.linalg.norm(v) > 20:
        raise ValueError('Speed exceeds supported near-Earth range')
    try:
        from astropy import units as u
        from astropy.time import Time
        from astropy.coordinates import (GCRS, ITRS, CartesianRepresentation,
                                         CartesianDifferential, get_sun)
        from astropy.utils import iers
    except ImportError as exc:
        raise RuntimeError('Install optional reference dependencies: python -m pip install astropy ppigrf') from exc
    # Crucial: position alone does not transform spacecraft velocity correctly.
    with iers.conf.set_temp('auto_download',False):
        t=Time(epoch)
        rep=CartesianRepresentation(r*u.km, differentials=CartesianDifferential(v*u.km/u.s))
        source=GCRS(rep,obstime=t) if input_frame=='GCRS' else ITRS(rep,obstime=t)
        target=ITRS(obstime=t) if input_frame=='GCRS' else GCRS(obstime=t)
        dest=source.transform_to(target)
        def state(c):
            xyz=c.cartesian
            return {'position_km':xyz.xyz.to_value(u.km).tolist(),
                    'velocity_km_s':xyz.differentials['s'].d_xyz.to_value(u.km/u.s).tolist()}
        source_state=state(source); target_state=state(dest)
        gcrs=source if input_frame=='GCRS' else dest
        itrs=dest if input_frame=='GCRS' else source
        sun=get_sun(t).cartesian.xyz.to_value(u.km)
    sun_norm=np.linalg.norm(sun)
    result={'epoch_utc':epoch.isoformat(),'input_frame':input_frame,
            'output_frame':'ITRS' if input_frame=='GCRS' else 'GCRS',
            'source':source_state,'converted':target_state,
            'gcrs_position_km':gcrs.cartesian.xyz.to_value(u.km).tolist(),
            'itrs_position_km':itrs.cartesian.xyz.to_value(u.km).tolist(),
            'sun_direction_gcrs':(sun/sun_norm).tolist(),
            'sun_distance_km':float(sun_norm),
            'igrf14':None,
            'metadata':{'reference_frames':'Astropy GCRS/ITRS, absolute UTC epoch',
                'sun_model':'Astropy ERFA simplified VSOP2000 (geocentric)',
                'magnetic_model':'IGRF-14 ppigrf if requested',
                'iers_policy':'auto-download disabled; local IERS data may be out of date',
                'flight_qualified':False,
                'warning':'GCRS != generic APOGEE_IDEAL_ECI, ICRF, or TEME; do not subtract unaligned states.'}}
    if include_igrf:
        if epoch.year>2029 or (epoch.year==2030 and (epoch.month,epoch.day)>(1,1)):
            raise ValueError('IGRF-14 predictive interval ends at 2030.0')
        try:
            import ppigrf
        except ImportError as exc:
            raise RuntimeError('Install IGRF-14 support: python -m pip install "ppigrf>=2"') from exc
        xyz=itrs.cartesian.xyz.to_value(u.km)
        radius=float(np.linalg.norm(xyz))
        theta=float(np.rad2deg(np.arccos(np.clip(xyz[2]/radius,-1,1))))
        lon=float(np.rad2deg(np.arctan2(xyz[1],xyz[0])))
        br,bt,bp=ppigrf.igrf_gc(radius,theta,lon,epoch.replace(tzinfo=None))
        b=components_to_cartesian(np.asarray(br).ravel()[0],np.asarray(bt).ravel()[0],
                                   np.asarray(bp).ravel()[0],theta,lon)
        result['igrf14']={'field_itrs_tesla':b.tolist(),
                          'magnitude_microtesla':float(np.linalg.norm(b)*1.e6),
                          'geocentric_colatitude_deg':theta,'longitude_deg':lon,
                          'model':'IGRF-14 main field + secular variation through 2030.0'}
    return result
