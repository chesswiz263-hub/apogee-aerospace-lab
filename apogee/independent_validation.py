"""APOGEE v2.2 — independent software verification, not flight validation.

Two-body states are compared with a mathematically independent closed-form
Kepler solution. Optional python-sgp4 is checked against externally published
Vallado TEME verification samples, with explicit provenance, strict units/frame,
and an unambiguous SKIP state if the dependency is missing.
"""
from __future__ import annotations

import hashlib
import json
from math import cos, sin, sqrt, pi, atan2
from pathlib import Path

import numpy as np
from .constants import EARTH_MU_KM3_S2 as MU
from .physics import integrate_orbit

FIXTURE = Path(__file__).resolve().parents[1] / 'examples' / 'v22_vallado_public_teme_subset.json'


def _kepler_eccentric_anomaly(mean_anomaly: float, e: float) -> float:
    m = (mean_anomaly + pi) % (2*pi) - pi
    eccentric = m + e*sin(m)
    for _ in range(24):
        delta = (eccentric - e*sin(eccentric) - m)/(1-e*cos(eccentric))
        eccentric -= delta
        if abs(delta)<1e-13:break
    if abs(eccentric-e*sin(eccentric)-m)>1e-10:
        raise ArithmeticError('Kepler equation failed to converge')
    return eccentric


def analytic_elliptic_state(t_s: float, *, a_km:float=8400.0, eccentricity:float=0.14,
                            inclination_deg:float=51.6, raan_deg:float=23,
                            argument_periapsis_deg:float=17, initial_mean_anomaly_deg:float=25) -> np.ndarray:
    """Closed-form two-body ellipse in fixed Cartesian axes, km and km/s.

    No numerical ODE solver or APOGEE orbit-element conversion is called.
    """
    values=np.asarray([t_s,a_km,eccentricity,inclination_deg,raan_deg,argument_periapsis_deg,initial_mean_anomaly_deg],float)
    if not np.all(np.isfinite(values)) or a_km <= 6378.137 or not 0 <= eccentricity < 0.8 or a_km*(1-eccentricity) <= 6378.137:
        raise ValueError('Expected a bound non-impacting elliptical Earth orbit')
    inc, O, w=np.deg2rad([inclination_deg,raan_deg,argument_periapsis_deg])
    mean= np.deg2rad(initial_mean_anomaly_deg)+sqrt(MU/a_km**3)*t_s
    E=_kepler_eccentric_anomaly(mean,eccentricity)
    b=sqrt(1-eccentricity**2)
    d=1-eccentricity*cos(E)
    pqw_r=np.array([a_km*(cos(E)-eccentricity),a_km*b*sin(E),0.])
    n=sqrt(MU/a_km**3)
    pqw_v=np.array([-a_km*n*sin(E)/d,a_km*n*b*cos(E)/d,0.])
    cO,sO,cw,sw,ci,si=cos(O),sin(O),cos(w),sin(w),cos(inc),sin(inc)
    transform=np.array([[cO*cw-sO*sw*ci,-cO*sw-sO*cw*ci,sO*si],
                        [sO*cw+cO*sw*ci,-sO*sw+cO*cw*ci,-cO*si],
                        [sw*si,cw*si,ci]])
    return np.r_[transform@pqw_r,transform@pqw_v]


def analytic_benchmark(*, steps_per_orbit:int=240, orbits:int=2) -> dict:
    if not 40 <= steps_per_orbit <= 1200 or not 1 <= orbits <= 5:
        raise ValueError('steps_per_orbit must be 40..1200 and orbits 1..5')
    period=2*pi*sqrt(8400**3/MU)
    interval=period*orbits
    steps=steps_per_orbit*orbits
    initial=analytic_elliptic_state(0.0)
    numerical=integrate_orbit(initial,interval,steps)
    expected=np.stack([analytic_elliptic_state(float(t)) for t in numerical.time_s])
    pos=np.linalg.norm(numerical.states[:,:3]-expected[:,:3],axis=1)*1000
    vel=np.linalg.norm(numerical.states[:,3:]-expected[:,3:],axis=1)*1000
    tol_pos_m=15.0   # fixed RK4 at 240 samples/orbit; tune with resolution
    tol_velocity_m_s=0.02
    return {
        'case':'independent_kepler_ellipse', 'status':'PASS' if max(pos)<=tol_pos_m and max(vel)<=tol_velocity_m_s else 'FAIL',
        'reference_type':'analytic_solution_not_external_observations',
        'frame':'idealized_Earth_centered_inertial_axes',
        'time_scale':'elapsed_simulation_seconds',
        'samples':len(pos), 'elapsed_s':float(interval),
        'max_position_error_m':float(max(pos)), 'max_velocity_error_m_s':float(max(vel)),
        'final_position_error_m':float(pos[-1]), 'position_tolerance_m':tol_pos_m,
        'velocity_tolerance_m_s':tol_velocity_m_s,
        'trajectory':{'time_s':numerical.time_s[::max(1,steps//120)].tolist(),
                      'position_error_m':pos[::max(1,steps//120)].tolist(),
                      'velocity_error_m_s':vel[::max(1,steps//120)].tolist()},
        'note':'Independent Kepler equation vs APOGEE fixed-step RK4. Does not test J2, drag, sensor measurements or real ephemerides.'
    }


def parse_sgp4_reference(data:dict) -> dict:
    """Validate explicit TEME/elapsed-time reference data. No frame coercion."""
    if not isinstance(data,dict) or data.get('schema')!='apogee.sgp4-reference.v1':
        raise ValueError('Unsupported SGP4 reference schema')
    if data.get('frame')!='TEME' or data.get('time_unit')!='minutes_since_tle_epoch' or data.get('position_unit')!='km' or data.get('velocity_unit')!='km/s':
        raise ValueError('SGP4 reference requires TEME, minutes since TLE epoch, km and km/s')
    if not isinstance(data.get('satellites'),list) or not 1<=len(data['satellites'])<=10:
        raise ValueError('Expected 1 to 10 satellite cases')
    samples_total=0
    for sat in data['satellites']:
        if not isinstance(sat,dict) or not isinstance(sat.get('tle_line1'),str) or not isinstance(sat.get('tle_line2'),str):
            raise ValueError('Each case needs two TLE lines')
        l1,l2=sat['tle_line1'],sat['tle_line2']
        if not l1.startswith('1 ') or not l2.startswith('2 ') or len(l1)<69 or len(l2)<69:
            raise ValueError('Malformed two-line element set')
        if l1[2:7]!=l2[2:7] or int(l1[2:7]) != sat.get('catalog_id'):
            raise ValueError('TLE catalog numbers inconsistent')
        rows=sat.get('samples')
        if not isinstance(rows,list) or not 1<=len(rows)<=250:
            raise ValueError('Each case needs 1..250 state vectors')
        samples_total+=len(rows)
        seen=set()
        for row in rows:
            if not isinstance(row,dict):raise ValueError('Reference sample must be object')
            try:
                t=float(row['minutes']);r=np.asarray(row['position_km'],float);v=np.asarray(row['velocity_km_s'],float)
            except (KeyError,ValueError,TypeError) as exc:raise ValueError('Invalid state vector') from exc
            if not np.isfinite(t) or abs(t)>100000 or r.shape!=(3,) or v.shape!=(3,) or not np.all(np.isfinite(np.r_[r,v])):
                raise ValueError('Reference vectors must contain three finite components')
            if t in seen:raise ValueError('Duplicate propagation epoch')
            seen.add(t)
    return {'case_count':len(data['satellites']),'sample_count':samples_total}


def sgp4_reference_benchmark(data:dict, *, allow_skip:bool=True) ->dict:
    summary=parse_sgp4_reference(data)
    try:
        from sgp4.api import Satrec, WGS72
    except ImportError as exc:
        if not allow_skip:raise RuntimeError('Install optional sgp4 package to run the Vallado verification check') from exc
        return {'case':'vallado_published_teme_subset','status':'SKIP',**summary,
                'reason':'python-sgp4 not installed; published reference vectors are bundled but NOT compared',
                'frame':'TEME','reference_type':'published_software_verification_vectors',
                'max_position_error_m':None,'max_velocity_error_m_s':None}
    residual_r=[];residual_v=[]
    for sat in data['satellites']:
        # The community verification vectors use the WGS72 constants.
        satrec=Satrec.twoline2rv(sat['tle_line1'],sat['tle_line2'],WGS72)
        for sample in sat['samples']:
            error,r,v=satrec.sgp4_tsince(float(sample['minutes']))
            if error:
                return {'case':'vallado_published_teme_subset','status':'FAIL',**summary,
                        'reason':f'SGP4 propagator error code {error}',
                        'max_position_error_m':None,'max_velocity_error_m_s':None}
            residual_r.append(float(np.linalg.norm(np.asarray(r)-np.asarray(sample['position_km']))*1000))
            residual_v.append(float(np.linalg.norm(np.asarray(v)-np.asarray(sample['velocity_km_s']))*1000))
    pos_max=max(residual_r);vel_max=max(residual_v)
    pos_tol=.05 # 5 cm, conservative for printed public reference vector precision
    vel_tol=.00001 # 10 micrometres/second
    return {'case':'vallado_published_teme_subset',**summary,
            'status':'PASS' if pos_max<pos_tol and vel_max<vel_tol else 'FAIL',
            'max_position_error_m':pos_max,'max_velocity_error_m_s':vel_max,
            'position_tolerance_m':pos_tol,'velocity_tolerance_m_s':vel_tol,
            'frame':'TEME','reference_type':'published_software_verification_vectors',
            'reason':'Published software output comparison only—not tracking validation'}


def suite_report(*, steps_per_orbit:int=240, orbits:int=2, source:dict|None=None) -> dict:
    fixture_bytes=FIXTURE.read_bytes()
    reference=json.loads(fixture_bytes) if source is None else source
    analytic=analytic_benchmark(steps_per_orbit=steps_per_orbit,orbits=orbits)
    sgp=sgp4_reference_benchmark(reference)
    statuses=[analytic['status'],sgp['status']]
    return {'version':'2.2.0', 'suite_status':'FAIL' if 'FAIL' in statuses else ('PARTIAL' if 'SKIP' in statuses else 'PASS'),
            'passed':statuses.count('PASS'),'failed':statuses.count('FAIL'),'skipped':statuses.count('SKIP'),
            'results':[analytic,sgp],
            'reference_bundle':{'file':FIXTURE.name,'sha256':hashlib.sha256(fixture_bytes).hexdigest(),
               'source':reference.get('provenance',{}),'authentication':'source URLs cited; no cryptographic upstream signature'},
            'limitations':['Software verification is not agreement with actual spacecraft tracking',
                           'SGP4 TEME and idealized two-body XYZ states are never subtracted',
                           'Only the fixed public subset is tested; this is not the full Vallado test corpus',
                           'Astropy/IERS positive path and flight-data validation are separate unfinished checks']}
