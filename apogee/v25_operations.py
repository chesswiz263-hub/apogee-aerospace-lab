"""Isolated v2.5 FastAPI routes: solar model, CelesTrak OMM and SGP4/Astropy.

No network requests at import or at the solar endpoint. Fixed CelesTrak host;
positive integer catalog numbers only. Cache each catalog for two hours.
"""
from __future__ import annotations

import json
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import APIRouter, HTTPException, Query
from .v25_solar import parse_utc, solar_system_state

router = APIRouter(prefix="/api/v25", tags=["APOGEE v2.5"])
_CACHE = {}
_LOCK = threading.Lock()
_QUERY_TIMES = []
MAX_QUERIES_PER_HOUR = 24
TTL = 2*3600
CACHE_DIR = Path(os.environ.get('APOGEE_V25_CACHE_DIR', '/tmp/apogee-v25-celestrak'))
MAX_AGE_DAYS = 14   # Flag stale orbital elements; no operational predictions implied.

@router.get('/capabilities')
def capabilities():
    from importlib.util import find_spec
    return {"version":"2.5.0","sgp4_available": find_spec('sgp4') is not None,
            "astropy_available":find_spec('astropy') is not None,
            "solar_model":"JPL approximate elements (1800--2050)",
            "celestrak_format":"OMM JSON","verification":"see /api/v25/verification"}

@router.get('/solar-system')
def solar_system(at: str | None = None):
    try:
        return solar_system_state(at)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc

@router.get('/verification')
def verification():
    """Expose Render build audit summary, never imply certification without it."""
    path = Path(os.environ.get('APOGEE_V25_AUDIT', 'reports/v25-render-audit.json'))
    if not path.exists():
        return {"status":"NOT_RUN","message":"Run scripts/v25_render_gate.py in Render build command; no audit found"}
    try:
        data = json.loads(path.read_text(encoding='utf8'))
    except (OSError, ValueError):
        raise HTTPException(500, 'Cannot read v2.5 build audit')
    return {"status":data.get('release_gate', data.get('status','UNKNOWN')),
            "audit":data, "warning":"Reference software verification is not independently measured orbital accuracy"}

def _read_cache(n: int):
    now = time.time()
    entry = _CACHE.get(n)
    if entry and now-entry['fetched_at'] < TTL:
        return entry
    # Persisted cache mitigates repeated queries across local worker restarts.
    try:
        raw = json.loads((CACHE_DIR / f'{n}.json').read_text('utf8'))
        if now-raw['fetched_at'] < TTL and isinstance(raw['records'],list):
            _CACHE[n]=raw
            return raw
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None

def _fetch_omm(n: int):
    with _LOCK:
        cached = _read_cache(n)
        if cached:
            return cached['records'], True
        now = time.time()
        _QUERY_TIMES[:] = [t for t in _QUERY_TIMES if now-t < 3600]
        if len(_QUERY_TIMES) >= MAX_QUERIES_PER_HOUR:
            raise HTTPException(429, 'APOGEE hourly CelesTrak request budget reached; retry later')
        _QUERY_TIMES.append(now)
        # Modern catalog numbers > 99999 are not representable in legacy TLE.
        uri = f'https://celestrak.org/NORAD/elements/gp.php?CATNR={n}&FORMAT=JSON'
        request = Request(uri, headers={'User-Agent':'APOGEE-v2.5 research simulator (CelesTrak OMM reader)',
                                        'Accept':'application/json'})
        try:
            with urlopen(request, timeout=9) as reply:
                if reply.status != 200:
                    raise RuntimeError('Upstream returned a non-200 response')
                content = reply.read(2_000_001)
                if len(content)>2_000_000:
                    raise RuntimeError('Upstream payload too large')
                records = json.loads(content)
            if not isinstance(records,list) or not records:
                raise RuntimeError('No catalog entry returned')
            if len(records)>10 or any(not isinstance(r,dict) for r in records):
                raise RuntimeError('Unexpected OMM response shape')
            # Basic structural validation prevents caching upstream HTML errors.
            for r in records:
                if str(r.get('NORAD_CAT_ID','')) != str(n) or 'EPOCH' not in r:
                    raise RuntimeError('Invalid catalog or missing epoch in OMM response')
        except (HTTPError, URLError, OSError, TimeoutError, RuntimeError, ValueError) as exc:
            raise HTTPException(502, f'CelesTrak OMM retrieval failed: {str(exc)[:160]}') from exc
        entry = {"fetched_at":time.time(),"records":records}
        _CACHE[n]=entry
        try:
            CACHE_DIR.mkdir(parents=True,exist_ok=True)
            (CACHE_DIR / f'{n}.json').write_text(json.dumps(entry),encoding='utf8')
        except OSError:
            pass
        return records, False

def _utc(timestamp: datetime):
    return timestamp.isoformat().replace('+00:00','Z')

@router.get('/satellites/{catalog_id}/state')
def satellite_state(catalog_id: int, at: str | None = None):
    if not 1 <= catalog_id <= 999_999_999:
        raise HTTPException(422,'Catalog number must be 1 to 9 digits')
    try:
        when = parse_utc(at)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    try:
        from sgp4 import omm
        from sgp4.api import Satrec, WGS72, jday
    except ImportError as exc:
        raise HTTPException(503, 'SGP4 package unavailable; install requirements-v25.txt') from exc
    records, from_cache = _fetch_omm(catalog_id)
    fields = records[0]
    try:
        sat = Satrec()
        omm.initialize(sat, fields, WGS72)
        jd, fr = jday(when.year,when.month,when.day,when.hour,when.minute,when.second+when.microsecond/1e6)
        error, r, v = sat.sgp4(jd,fr)
        if error:
            raise HTTPException(422, f'SGP4 propagator error code {error}')
        epoch = parse_utc(str(fields['EPOCH']) + ('Z' if not str(fields['EPOCH']).endswith('Z') and '+' not in str(fields['EPOCH']) else ''))
    except HTTPException:
        raise
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise HTTPException(502, f'Invalid OMM / SGP4 state: {str(exc)[:160]}') from exc
    age_days = abs((when-epoch).total_seconds())/86400
    response = {"catalog_number":catalog_id,"name":fields.get('OBJECT_NAME','Unknown'),
                "time_utc":_utc(when),"elements_epoch_utc":_utc(epoch),
                "element_age_days":round(age_days,3),"elements_cached":from_cache,
                "stale_or_far_from_epoch":age_days>MAX_AGE_DAYS,
                "model":"SGP4/SDP4 (Vallado-compatible), WGS72",
                "frames":{"TEME":{"position_km":list(r),"velocity_km_s":list(v)}},
                "limitations":["GP element predictions, not measured states", "SGP4 TEME is not GCRS/ICRS",
                               "Uncertainty and atmospheric errors not modeled"]}
    try:
        import astropy.units as u
        from astropy.coordinates import CartesianDifferential, CartesianRepresentation, GCRS, ITRS, TEME
        from astropy.time import Time
        from astropy.utils import iers
        iers.conf.auto_download=False
        t = Time(when, scale='utc')
        state = TEME(CartesianRepresentation(r*u.km).with_differentials(
            CartesianDifferential(v*u.km/u.s)), obstime=t)
        itrs = state.transform_to(ITRS(obstime=t))
        gcrs = state.transform_to(GCRS(obstime=t))
        response['frames']['ITRS'] = {'position_km':itrs.cartesian.xyz.to_value(u.km).tolist(),
                                       'velocity_km_s':itrs.cartesian.differentials['s'].d_xyz.to_value(u.km/u.s).tolist()}
        response['frames']['GCRS'] = {'position_km':gcrs.cartesian.xyz.to_value(u.km).tolist(),
                                       'velocity_km_s':gcrs.cartesian.differentials['s'].d_xyz.to_value(u.km/u.s).tolist()}
        lon,lat,alt = itrs.earth_location.geodetic
        response['subpoint']={'longitude_deg':float(lon.to_value(u.deg)),
                              'latitude_deg':float(lat.to_value(u.deg)),
                              'height_km':float(alt.to_value(u.km))}
    except ImportError:
        response['limitations'].append('Astropy missing: ITRS/GCRS conversions unavailable')
    except Exception as exc:
        # Never fabricate a frame when EOP data or transformation fails.
        response['frame_transform_error'] = str(exc)[:200]
        response['limitations'].append('Astropy frame transformation failed; TEME only')
    return response
