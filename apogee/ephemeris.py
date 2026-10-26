"""Offline, bounded import of explicit JPL Horizons geometric vectors CSV.

Do not confuse ICRF Earth-centered reference vectors with TEME SGP4 outputs
or the idealized model axes. Metadata is provided/attested by the user;
format validation alone cannot verify provenance or physical accuracy.
"""
import csv
import io
from dataclasses import dataclass
import numpy as np
from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import elements_from_state

METADATA = {'center':'500@399','frame':'ICRF','plane':'FRAME','units':'KM-S',
            'corrections':'NONE','time_scale':'TDB'}

@dataclass
class HorizonsVectors:
    jd_tdb: np.ndarray
    states: np.ndarray


def parse_horizons_vectors(text: str, metadata: dict) -> HorizonsVectors:
    """CSV_FORMAT=YES, VEC_TABLE=2: JD, calendar, X,Y,Z,VX,VY,VZ, [trailing]."""
    if not isinstance(text,str) or len(text)>260000:
        raise ValueError('Reference text must be a string under 260 KB')
    if not isinstance(metadata,dict) or any(metadata.get(k)!=v for k,v in METADATA.items()):
        raise ValueError('Required Horizons metadata: CENTER=500@399, ICRF, FRAME, KM-S, VEC_CORR=NONE, TIME_TYPE=TDB')
    if text.count('$$SOE')!=1 or text.count('$$EOE')!=1:
        raise ValueError('Expected exactly one Horizons $$SOE/$$EOE vector block')
    header,tail=text.split('$$SOE',1)
    middle,_=tail.split('$$EOE',1)
    if len(middle)>200000:
        raise ValueError('Ephemeris sample block too large')
    # Fail on explicit, detectable header disagreements; other headers may vary.
    import re
    center=re.search(r'^\s*Center body name:\s*(.*)$',header,re.I|re.M)
    if center and not re.search(r'\bEarth\b|\(399\)',center.group(1),re.I):
        raise ValueError('The Horizons vector center is not Earth')
    unit=re.search(r'^\s*Output units:\s*(.*)$',header,re.I|re.M)
    if unit and not re.search(r'\bKM-S\b',unit.group(1),re.I):
        raise ValueError('Horizons header does not specify KM-S units')
    time_type=re.search(r'^\s*Time format\s*:\s*(.*)$',header,re.I|re.M)
    if time_type and 'TDB' not in time_type.group(1).upper():
        raise ValueError('Horizons time format is not TDB')
    rows=list(csv.reader(io.StringIO(middle.strip())))
    if not 2<=len(rows)<=1000:
        raise ValueError('Expected between 2 and 1000 vector records')
    data=[]
    for i,row in enumerate(rows):
        if len(row)<8:
            raise ValueError(f'Row {i+1}: expected Julian date, date label and six Cartesian components')
        try:
            vals=[float(row[0])]+[float(v.replace('D','E').replace('d','e')) for v in row[2:8]]
        except ValueError as exc:
            raise ValueError(f'Row {i+1}: expected finite numeric Cartesian components') from exc
        if not np.isfinite(vals).all():
            raise ValueError(f'Row {i+1}: non-finite state')
        data.append(vals)
    data=np.array(data,float)
    if not np.all(np.diff(data[:,0])>1e-11):
        raise ValueError('Reference Julian dates must be strictly increasing')
    if data[-1,0]-data[0,0]>370:
        raise ValueError('Maximum reference time span is 370 days')
    if np.any(np.linalg.norm(data[:,1:4],axis=1)<RE):
        raise ValueError('Reference states are below the nominal Earth radius')
    return HorizonsVectors(data[:,0],data[:,1:])


def format_reference_vectors(parsed: HorizonsVectors) -> dict:
    s=parsed.states
    elapsed=(parsed.jd_tdb-parsed.jd_tdb[0])*86400
    radii=np.linalg.norm(s[:,:3],axis=1)
    speed=np.linalg.norm(s[:,3:],axis=1)
    initial=elements_from_state(s[0])
    return {
        'name':'Imported Horizons Earth-centered vectors',
        'model':{'name':'JPL Horizons vectors (imported text; provenance unverified)',
                 'reference_frame':'Earth-centered ICRF; TDB Julian date; user-attested configuration',
                 'integrator':'none — imported reference positions and velocities',
                 'step_s':None,'force_terms':[],
                 'assumptions':['Metadata is user-attested; no provenance authentication',
                                'not TEME and not directly matched to the idealized ECI solver',
                                'Julian dates are TDB, not UTC']},
        'reference':{'kind':'JPL Horizons text import','origin':'user-supplied local text',
                     'metadata':METADATA.copy(), 'epoch_start_jd_tdb':float(parsed.jd_tdb[0]),
                     'epoch_end_jd_tdb':float(parsed.jd_tdb[-1]),
                     'jd_tdb':parsed.jd_tdb.tolist()},
        'initial_state':{'r_km':s[0,:3].tolist(),'v_km_s':s[0,3:].tolist()},
        'orbital_elements':initial.to_dict(),
        'metrics':{'samples':len(s),'step_s':None,'elapsed_s':float(elapsed[-1]),
                    'requested_duration_s':float(elapsed[-1]),
                    'min_altitude_km':float(radii.min()-RE),'max_altitude_km':float(radii.max()-RE),
                    'max_relative_energy_drift':None,'max_relative_angular_momentum_drift':None,
                    'conservation_applicable':False,'diagnostic_scope':'external reference data, no numerical propagation'},
        'trajectory':{'time_s':elapsed.tolist(),'position_km':s[:,:3].tolist(),
                      'velocity_km_s':s[:,3:].tolist(),
                      'altitude_km':(radii-RE).tolist(),'speed_km_s':speed.tolist(),
                      'relative_energy_drift':[],'relative_angular_momentum_drift':[]},
        'warnings':['User-supplied text; source authenticity is not verified.',
                    'Earth-centered ICRF vectors in TDB cannot be directly subtracted from SGP4 TEME outputs or idealized ECI without matching frames/epochs.',
                    'Raw reference trajectory only; no observational-model error comparison was attempted.']}
