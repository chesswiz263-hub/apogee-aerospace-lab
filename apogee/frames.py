"""Frame transformations: explicit uniform Earth spin, optional Astropy TEME.

The uniform spin transformation is a pedagogical ECI<->ECEF model defined
by an EXPLICIT Greenwich angle at t=0; do not imply UT1/IERS fidelity.
"""
from datetime import datetime, timezone
import numpy as np
from .perturbations import EARTH_SPIN_RAD_S as EARTH_ANGULAR_VELOCITY_RAD_S


def _six(s):
    a=np.asarray(s,dtype=float)
    if a.shape!=(6,) or not np.isfinite(a).all():
        raise ValueError('State must contain six finite components')
    return a


def spin_transform(state, theta_deg: float, elapsed_s: float=0,
                   direction: str='eci_to_ecef'):
    """Rigid constant-rate Earth rotation including the omega-cross-r velocity term."""
    s=_six(state)
    if not np.isfinite([theta_deg,elapsed_s]).all() or abs(elapsed_s)>1e8:
        raise ValueError('Invalid Earth angle or elapsed time')
    theta=np.deg2rad(theta_deg)+EARTH_ANGULAR_VELOCITY_RAD_S*elapsed_s
    c,t=np.cos(theta),np.sin(theta)
    rot=np.array([[c,-t,0],[t,c,0],[0,0,1]])
    omega=np.array([0,0,EARTH_ANGULAR_VELOCITY_RAD_S])
    if direction=='eci_to_ecef':
        return np.r_[rot.T@s[:3],rot.T@(s[3:]-np.cross(omega,s[:3]))]
    if direction=='ecef_to_eci':
        r=rot@s[:3]
        return np.r_[r, rot@s[3:]+np.cross(omega,r)]
    raise ValueError('direction must be eci_to_ecef or ecef_to_eci')


def astropy_available():
    try:
        import astropy.coordinates  # noqa: F401
        return True
    except ImportError:
        return False


def transform_teme(state, epoch_utc: str, destination: str='itrs'):
    """Transform six-state TEME at absolute UTC epoch to GCRS or ITRS via Astropy.

    Earth orientation accuracy depends on available IERS data; offline
    prediction can degrade. This is NOT an ICRF geocentric ephemeris match.
    """
    s=_six(state)
    if destination not in ('itrs','gcrs'):
        raise ValueError('destination must be itrs or gcrs')
    try:
        instant=datetime.fromisoformat(epoch_utc.replace('Z','+00:00'))
        if instant.tzinfo is None:raise ValueError('Timezone required')
    except (ValueError,AttributeError) as exc:
        raise ValueError('epoch_utc must be timezone-aware ISO 8601') from exc
    try:
        import astropy.units as u
        from astropy.time import Time
        from astropy.coordinates import TEME, GCRS, ITRS, CartesianRepresentation, CartesianDifferential
        from astropy.utils import iers
    except ImportError as exc:
        raise RuntimeError('Install frame extra: python -m pip install "apogee-orbit[frames]"') from exc
    # No implicit network side effects during local scientific transformation.
    with iers.conf.set_temp('auto_download',False):
        time=Time(instant.astimezone(timezone.utc))
        rep=CartesianRepresentation(s[:3]*u.km)
        rep=rep.with_differentials(CartesianDifferential(s[3:]*u.km/u.s))
        teme=TEME(rep,obstime=time)
        frame=ITRS(obstime=time) if destination=='itrs' else GCRS(obstime=time)
        transformed=teme.transform_to(frame)
        result=np.r_[transformed.cartesian.xyz.to_value(u.km),
                     transformed.cartesian.differentials['s'].d_xyz.to_value(u.km/u.s)]
        return result
