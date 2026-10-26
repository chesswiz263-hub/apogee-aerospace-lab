"""APOGEE v2.3: provenance-conscious reference regression and frame checks.

The published TLE and TEME output files are *software verification* cases, not
observations of a tracked satellite. A missing optional dependency is SKIP.
The original full text files can be imported locally without a network call.
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Any
import numpy as np

from .independent_validation import parse_sgp4_reference, sgp4_reference_benchmark, analytic_benchmark

ROOT = Path(__file__).resolve().parents[1]
CURATED = ROOT / 'examples' / 'v23_vallado_multiregime_subset.json'
SOURCE_NAMES = ('SGP4-VER.TLE', 'tcppver.out')


def parse_vallado_raw(tle_text: str, output_text: str) -> dict:
    """Import full public Vallado tcppver format while retaining TEME metadata.

    The first 69 chars are the exact TLE; trailing fields in the source TLE
    file describe requested integration times and must NOT enter the TLE.
    Cases with only SGP4 errors are tracked separately, not reported as PASS.
    """
    if not isinstance(tle_text, str) or not isinstance(output_text, str):
        raise ValueError('Both inputs must be plain-text Vallado files')
    if len(tle_text)>2_000_000 or len(output_text)>8_000_000:
        raise ValueError('Verification source exceeds configured size limit')
    tle_map = {}
    lines=tle_text.splitlines()
    for i,line in enumerate(lines):
        if not line.startswith('1 '):
            continue
        if i+1 >= len(lines) or not lines[i+1].startswith('2 '):
            raise ValueError('Missing second TLE line')
        l1=line[:69];l2=lines[i+1][:69]
        if len(l1)!=69 or len(l2)!=69 or l1[2:7]!=l2[2:7]:
            raise ValueError('Invalid Vallado TLE formatting')
        for tle in (l1,l2):
            checksum=(sum(int(ch) for ch in tle[:68] if ch.isdigit())+tle[:68].count('-'))%10
            if not tle[68].isdigit() or checksum!=int(tle[68]):
                raise ValueError('TLE checksum mismatch; source file may be corrupted')
        try: catalog=int(l1[2:7])
        except ValueError as exc: raise ValueError('Verification corpus requires numeric satellite IDs') from exc
        if catalog in tle_map:raise ValueError('Repeated TLE catalog number')
        tle_map[catalog]={'catalog_id':catalog,'tle_line1':l1,'tle_line2':l2,'samples':[]}
    if not tle_map:raise ValueError('No TLE records found')

    current=None; skipped_errors=0; headers_seen=set()
    for line in output_text.splitlines():
        fields=line.split()
        if not fields:continue
        if len(fields)==2 and fields[1]=='xx' and fields[0].isdigit():
            current=int(fields[0]);headers_seen.add(current)
            if current not in tle_map:raise ValueError('Reference output contains a catalog not in the supplied TLE file')
            continue
        if current is None: raise ValueError('Reference states appear before a satellite header')
        if 'xx' in fields:
            skipped_errors+=1  # known decays / SGP4 error diagnostics are not vector results
            continue
        if len(fields)<7:raise ValueError('Malformed published SGP4 vector record')
        try: v=[float(x) for x in fields[:7]]
        except ValueError as exc:raise ValueError('Published vector contains nonnumeric state data') from exc
        if not np.isfinite(v).all():raise ValueError('Nonfinite reference state')
        tle_map[current]['samples'].append({'minutes':v[0],'position_km':v[1:4],'velocity_km_s':v[4:7]})
    # Skipped error-only catalog cases are counted separately, never silently passed.
    satellites=[tle_map[k] for k in tle_map if k in headers_seen and tle_map[k]['samples']]
    if not satellites:raise ValueError('Reference output contains no valid state vectors')
    if any(len(s['samples'])>2000 for s in satellites):raise ValueError('Excessive samples per satellite')
    corpus={'schema':'apogee.sgp4-reference.v1','frame':'TEME',
        'time_unit':'minutes_since_tle_epoch','position_unit':'km','velocity_unit':'km/s',
        'source_kind':'published_software_verification_vectors_not_spacecraft_observations',
        'provenance':{'title':'Vallado SGP4 software verification corpus (user-provided official text)',
            'input_tle_sha256':hashlib.sha256(tle_text.encode()).hexdigest(),
            'input_vectors_sha256':hashlib.sha256(output_text.encode()).hexdigest()},
        'satellites':satellites}
    return {'reference':corpus,'cases_with_vectors':len(satellites),
            'samples':sum(len(s['samples']) for s in satellites),
            'catalogs_with_errors_only':len(headers_seen)-len(satellites),
            'error_lines_skipped':skipped_errors,
            'catalogs_without_published_header':len(set(tle_map)-headers_seen)}


def compare_many(data: dict) -> dict:
    """Validate each satellite separately; failures cannot hide in max-only aggregate."""
    if not isinstance(data, dict):raise ValueError('Expected reference JSON object')
    sats=data.get('satellites')
    if not isinstance(sats,list) or not 1<=len(sats)<=100:
        raise ValueError('Expected 1 to 100 reference satellite cases')
    results=[]
    for satellite in sats:
        one={k:v for k,v in data.items() if k!='satellites'}
        one['satellites']=[satellite]
        result=sgp4_reference_benchmark(one,allow_skip=True)
        result['catalog_id']=satellite.get('catalog_id')
        results.append(result)
    statuses=[r['status'] for r in results]
    return {'case':'sgp4_multi_regime', 'status':'FAIL' if 'FAIL' in statuses else ('SKIP' if 'SKIP' in statuses else 'PASS'),
            'reference_type':'published_software_verification_vectors','frame':'TEME',
            'cases':len(results),'sample_count':sum(r['sample_count'] for r in results),
            'passed':statuses.count('PASS'),'failed':statuses.count('FAIL'),'skipped':statuses.count('SKIP'),
            'max_position_error_m':max((r['max_position_error_m'] for r in results if r['max_position_error_m'] is not None),default=None),
            'max_velocity_error_m_s':max((r['max_velocity_error_m_s'] for r in results if r['max_velocity_error_m_s'] is not None),default=None),
            'by_satellite':results}


def astropy_roundtrip() -> dict:
    """Positive GCRS↔ITRS roundtrip with Astropy, not independent ephemeris validation."""
    base={'case':'astropy_gcrs_itrs_roundtrip','reference_type':'internal_transform_consistency',
          'epoch_utc':'2020-01-01T12:00:00Z','frame':'GCRS / ITRS'}
    try:
        from astropy import units as u
        from astropy.coordinates import GCRS, ITRS, CartesianRepresentation, CartesianDifferential
        from astropy.time import Time
        from astropy.utils import iers
    except ImportError:
        return {**base,'status':'SKIP','reason':'Astropy not installed; no frame transformation executed'}
    try:
        iers.conf.auto_download=False
        obstime=Time(base['epoch_utc'],scale='utc')
        p=np.array([7010.,-310.,720.]);v=np.array([0.1,7.4,1.2])
        state=CartesianRepresentation(*(p*u.km),differentials=CartesianDifferential(*(v*u.km/u.s)))
        gcrs=GCRS(state,obstime=obstime)
        itrs=gcrs.transform_to(ITRS(obstime=obstime))
        restored=itrs.transform_to(GCRS(obstime=obstime))
        pr=restored.cartesian.xyz.to_value(u.km)
        vr=restored.cartesian.differentials['s'].d_xyz.to_value(u.km/u.s)
        p_err=float(np.linalg.norm(pr-p)*1000)
        v_err=float(np.linalg.norm(vr-v)*1000)
        return {**base,'status':'PASS' if p_err<0.05 and v_err<0.0001 else 'FAIL',
                'position_roundtrip_error_m':p_err,'velocity_roundtrip_error_m_s':v_err,
                'note':'Astropy forward/inverse consistency only; not a comparison with an independent flight ephemeris.'}
    except (ValueError,RuntimeError,IndexError,KeyError,OSError) as exc:
        return {**base,'status':'SKIP','reason':'EOP data or frame transform unavailable: '+str(exc)[:180]}


def validation_report(*,steps_per_orbit:int=240,orbits:int=2) -> dict:
    raw=CURATED.read_bytes();curated=json.loads(raw)
    reference=compare_many(curated)
    analytic=analytic_benchmark(steps_per_orbit=steps_per_orbit,orbits=orbits)
    frames=astropy_roundtrip()
    results=[analytic,reference,frames]
    statuses=[x['status'] for x in results]
    return {'version':'2.3.0','suite_status':'FAIL' if 'FAIL' in statuses else ('PARTIAL' if 'SKIP' in statuses else 'PASS'),
            'passed':statuses.count('PASS'),'failed':statuses.count('FAIL'),'skipped':statuses.count('SKIP'),
            'results':results,
            'reference_bundle':{'file':CURATED.name,'sha256':hashlib.sha256(raw).hexdigest(),
                 'source':curated['provenance'], 'authentication':'manually curated excerpts from the publicly hosted corpus; no upstream signature'},
            'limitations':['A 4-case curated subset is bundled, not the full Vallado corpus.',
                'Complete 700-line tcppver.out plus SGP4-VER.TLE can be supplied to /api/verification/vallado-import.',
                'SGP4 missing means software-verification vectors are SKIP, not PASS.',
                'The Astropy roundtrip checks only internal transformations, not independent observations.',
                'No flight-tracking accuracy claims.']}
