"""Bounded CelesTrak OMM/JSON ingestion and SGP4 propagation in TEME.

Requires python-sgp4. Never present mean elements as osculating Keplerian
states or compare TEME directly to inertial ECI without a frame transform.
"""
import json
from datetime import datetime, timezone
from functools import lru_cache
from urllib.error import URLError, HTTPError
from urllib.request import urlopen
from urllib.parse import urlencode
import numpy as np

from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import elements_from_state


REQUIRED = ('EPOCH','MEAN_MOTION','ECCENTRICITY','INCLINATION','RA_OF_ASC_NODE',
            'ARG_OF_PERICENTER','MEAN_ANOMALY','NORAD_CAT_ID')


def validate_omm(raw: dict) -> dict:
    if not isinstance(raw,dict):raise ValueError('OMM must be a JSON object')
    record={k:v for k,v in raw.items() if isinstance(k,str) and len(k)<80}
    missing=[k for k in REQUIRED if k not in record]
    if missing: raise ValueError('Missing OMM fields: '+', '.join(missing))
    for key in ('MEAN_MOTION','ECCENTRICITY','INCLINATION','RA_OF_ASC_NODE','ARG_OF_PERICENTER','MEAN_ANOMALY'):
        try: val=float(record[key])
        except (ValueError,TypeError):raise ValueError(f'Invalid OMM {key}') from None
        if not np.isfinite(val):raise ValueError(f'Invalid OMM {key}')
    n,e=float(record['MEAN_MOTION']),float(record['ECCENTRICITY'])
    if not 0<n<20 or not 0<=e<1:raise ValueError('Invalid mean motion / eccentricity')
    try: epoch=datetime.fromisoformat(str(record['EPOCH']).replace('Z','+00:00'))
    except ValueError:raise ValueError('Invalid OMM epoch ISO timestamp') from None
    if epoch.tzinfo is None:epoch=epoch.replace(tzinfo=timezone.utc)
    if not 1<=int(record['NORAD_CAT_ID'])<=999999999:raise ValueError('Invalid NORAD catalog number')
    # Fix conventional OMM fields missing from truncated user records.
    record.setdefault('CLASSIFICATION_TYPE','U')
    record.setdefault('OBJECT_ID','')
    record.setdefault('EPHEMERIS_TYPE',0)
    record.setdefault('ELEMENT_SET_NO',1)
    record.setdefault('REV_AT_EPOCH',1)
    record.setdefault('BSTAR',0.0)
    record.setdefault('MEAN_MOTION_DOT',0.0)
    record.setdefault('MEAN_MOTION_DDOT',0.0)
    record['EPOCH']=epoch.astimezone(timezone.utc).replace(tzinfo=None).isoformat(timespec='microseconds')
    return record


def sgp4_available() -> bool:
    try:
        import sgp4  # noqa: F401
        return True
    except ImportError:return False


def propagate_omm(omm_record: dict, minutes: int=180, samples: int=360) -> dict:
    if not 1<=minutes<=1440 or not 10<=samples<=1000:
        raise ValueError('Satellite duration/samples out of bounds')
    record=validate_omm(omm_record)
    try:
        from sgp4.api import Satrec
        from sgp4 import omm
    except ImportError as exc:
        raise RuntimeError('Install optional SGP4 dependency: python -m pip install sgp4') from exc
    sat=Satrec()
    try:omm.initialize(sat,record)
    except Exception as exc:raise ValueError('SGP4 initialization failed: '+str(exc)) from exc
    times=np.linspace(0.,float(minutes)*60,samples+1)
    states=[]
    for t in times:
        code,r,v=sat.sgp4_tsince(float(t)/60)
        if code!=0: raise ValueError(f'SGP4 error {code} at +{t/60:.1f} min')
        states.append([*r,*v])
    s=np.asarray(states,float)
    radii=np.linalg.norm(s[:,:3],axis=1)
    speeds=np.linalg.norm(s[:,3:],axis=1)
    return {
      'name':str(record.get('OBJECT_NAME') or 'Catalog object')[:100],
      'model':{'name':'SGP4 GP mean-elements propagation','reference_frame':'TEME (true equator, mean equinox), NOT J2000 ECI',
        'integrator':'SGP4 reference library', 'impact_detected_at_sample':bool(np.any(radii<=RE)),
        'step_s':float(times[1]),'force_terms':['SGP4 general perturbations (via independent library)'],
        'assumptions':['GP data are fitted mean elements, not instantaneous osculating states',
                       'Earth is illustrated schematically; ground tracks/georeferencing are not provided',
                       'prediction uncertainty grows with age; not certified for conjunction risk']},
      'satellite':{'catalog_number':int(record['NORAD_CAT_ID']),'epoch_utc':record['EPOCH'],
                   'name':str(record.get('OBJECT_NAME','Unnamed')),'source':'CelesTrak OMM/JSON or user-provided OMM'},
      'orbital_elements':elements_from_state(s[0]).to_dict(),
      'initial_state':{'r_km':s[0,:3].tolist(),'v_km_s':s[0,3:].tolist()},
      'metrics':{'samples':len(s),'step_s':float(times[1]),'elapsed_s':float(times[-1]),
                 'requested_duration_s':float(times[-1]),'min_altitude_km':float(radii.min()-RE),
                 'max_altitude_km':float(radii.max()-RE), 'max_relative_energy_drift':None,
                 'max_relative_angular_momentum_drift':None, 'diagnostic_scope':'not applicable to SGP4',
                 'conservation_applicable':False},
      'trajectory':{'time_s':times.tolist(),'position_km':s[:,:3].tolist(),
                    'velocity_km_s':s[:,3:].tolist(),'altitude_km':(radii-RE).tolist(),
                    'speed_km_s':speeds.tolist(),'relative_energy_drift':[],
                    'relative_angular_momentum_drift':[]},
      'warnings':['TEME frame, not strict ECI/J2000. Globe texture is illustrative and not Earth-fixed.',
                  'Catalog elements are observational fits. Forecasts require epoch age and covariance assessment.']
    }


@lru_cache(maxsize=24)
def _fetch_catalog_once(catalog_number: int, cache_bucket: int) -> dict:
    url='https://celestrak.org/NORAD/elements/gp.php?'+urlencode({'CATNR':catalog_number,'FORMAT':'JSON'})
    from urllib.request import Request
    request=Request(url,headers={'User-Agent':'APOGEE educational orbital analysis/0.3'})
    try:
        with urlopen(request,timeout=8) as response:
            payload=response.read(65537)
    except (HTTPError,URLError,TimeoutError) as exc:
        raise ValueError('CelesTrak unavailable or no current GP record for this catalog ID') from exc
    if len(payload)>65536:raise ValueError('CelesTrak payload too large')
    try: data=json.loads(payload)
    except (ValueError,UnicodeDecodeError) as exc:raise ValueError('CelesTrak did not return JSON') from exc
    if not isinstance(data,list) or len(data)!=1: raise ValueError('Expected one catalog record')
    return validate_omm(data[0])


def fetch_catalog(catalog_number: int) -> dict:
    import time
    if not 1<=catalog_number<=999999999:raise ValueError('Invalid catalog number')
    # 10-min cache bucket; no arbitrary URLs or group queries allowed.
    return _fetch_catalog_once(int(catalog_number),int(time.time()//600))
