"""Actual integration test; skips *locally* when packages are absent.
CI and Render install dependencies and must then exercise this path.
Not an independent orbital-accuracy validation.
"""
import pytest

def test_v25_teme_to_itrs_roundtrip():
    pytest.importorskip('sgp4')
    pytest.importorskip('astropy')
    import astropy.units as u
    from astropy.coordinates import TEME, ITRS, CartesianRepresentation, CartesianDifferential
    from astropy.time import Time
    from astropy.utils import iers
    from sgp4.api import Satrec
    iers.conf.auto_download=False
    # Well-known public example from SGP4 documentation (non-operational epoch).
    a='1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753'
    b='2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667'
    sat=Satrec.twoline2rv(a,b)
    err,r,v=sat.sgp4(sat.jdsatepoch,sat.jdsatepochF)
    assert err==0
    t=Time(sat.jdsatepoch+sat.jdsatepochF,format='jd',scale='utc')
    teme=TEME(CartesianRepresentation(r*u.km).with_differentials(
              CartesianDifferential(v*u.km/u.s)),obstime=t)
    itrs=teme.transform_to(ITRS(obstime=t))
    back=itrs.transform_to(TEME(obstime=t))
    delta=(back.cartesian.xyz-teme.cartesian.xyz).to_value(u.km)
    assert max(abs(x) for x in delta)<1e-3
