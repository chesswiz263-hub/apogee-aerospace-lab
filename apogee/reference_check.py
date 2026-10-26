"""v0.6: epoch/frame-aligned comparison with user-attested Horizons vectors.

Use the first imported Earth-centered ICRF state as the integration initial state
and compare at precisely the imported TDB epochs. Point-mass only: no uncertain
Earth pole orientation, TEME and UT/UTC conversions are involved.
"""
import numpy as np
from scipy.integrate import solve_ivp
from .ephemeris import parse_horizons_vectors
from .constants import EARTH_MU_KM3_S2 as MU,EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import elements_from_state


def compare_imported_reference(text:str,metadata:dict):
    parsed=parse_horizons_vectors(text,metadata)
    times=(parsed.jd_tdb-parsed.jd_tdb[0])*86400
    values=parsed.states
    initial=values[0]
    radii=np.linalg.norm(values[:,:3],axis=1)
    if len(times)>250 or times[-1]>36*3600 or np.max(radii)>RE+60000:
        raise ValueError('Comparison supports at most 250 samples, 36 hours, and Earth satellites below 60,000 km altitude')
    if np.linalg.norm(initial[3:])**2/2-MU/np.linalg.norm(initial[:3])>=0:
        raise ValueError('Only initially bound Earth satellite orbits are supported')
    def f(t,y):
        r=y[:3];return np.r_[y[3:], -MU*r/np.linalg.norm(r)**3]
    result=solve_ivp(f,(0,float(times[-1])),initial,t_eval=times,method='DOP853',
                     rtol=2e-11,atol=2e-12,max_step=100)
    if not result.success:
        raise ValueError('Model integration failed')
    calculated=result.y.T
    dr=calculated[:,:3]-values[:,:3];dv=calculated[:,3:]-values[:,3:]
    pd=np.linalg.norm(dr,axis=1);vd=np.linalg.norm(dv,axis=1)*1000
    rc=np.linalg.norm(calculated[:,:3],axis=1)
    warnings=['This comparison uses the first imported state as exact initial conditions (zero fitted initial residual).',
              'No provenance authentication: external data may not be actual JPL Horizons data.',
              'Reference is assumed Earth-centered ICRF, TDB, KM-S, geometric. A different frame or epoch invalidates the result. Float Julian dates can also introduce sub-meter timing quantization at orbital speeds.',
              'Residuals measure differences from a point-mass Earth model, not numerical accuracy or observational uncertainty.']
    return {
       'name':'Reference-aligned point-mass comparison',
       'validation':{'reference_origin':'user-attested Horizons-style text','frame':'Earth-centered ICRF',
          'time_scale':'JD TDB','epoch_start_jd_tdb':float(parsed.jd_tdb[0]),
          'alignment':'identical first state and exact matched TDB epochs; no fitted free parameters',
          'samples':int(len(times)),'position_rms_km':float(np.sqrt(np.mean(pd**2))),
          'position_max_km':float(pd.max()),'position_final_km':float(pd[-1]),
          'velocity_rms_m_s':float(np.sqrt(np.mean(vd**2))),
          'velocity_final_m_s':float(vd[-1])},
       'model':{'name':'Earth-centered ICRF point-mass gravity from imported initial state',
          'reference_frame':'Earth-centered ICRF (as attested by reference metadata)',
          'integrator':'SciPy DOP853 adaptive; sampled at reference TDB epochs',
          'force_terms':['Earth Newtonian central gravitational acceleration'],
          'assumptions':['the axes are held constant in ICRF','no J2, atmosphere, third-body gravity, thrust, or relativistic terms']},
       'orbital_elements':elements_from_state(initial).to_dict(),
       'initial_state':{'r_km':initial[:3].tolist(),'v_km_s':initial[3:].tolist()},
       'spacecraft_mass_kg':None,
       'metrics':{'samples':int(len(times)),'step_s':None,'elapsed_s':float(times[-1]),
           'min_altitude_km':float(rc.min()-RE),'max_altitude_km':float(rc.max()-RE),
           'max_relative_energy_drift':None,'max_relative_angular_momentum_drift':None,
           'conservation_applicable':False,'diagnostic_scope':'residuals, not invariant diagnostics'},
       'trajectory':{'time_s':times.tolist(),'position_km':calculated[:,:3].tolist(),
           'velocity_km_s':calculated[:,3:].tolist(),'altitude_km':(rc-RE).tolist(),
           'speed_km_s':np.linalg.norm(calculated[:,3:],axis=1).tolist(),
           'relative_energy_drift':[],'relative_angular_momentum_drift':[]},
       'comparison':{'time_s':times.tolist(),'separation_km':pd.tolist(),
           'relative_speed_m_s':vd.tolist(),'final_separation_km':float(pd[-1]),
           'max_separation_km':float(pd.max()),'final_velocity_difference_m_s':float(vd[-1]),
           'baseline_trajectory':{'time_s':times.tolist(),'position_km':values[:,:3].tolist(),
               'velocity_km_s':values[:,3:].tolist()},
           'baseline_model':'Imported Earth-centered ICRF vectors (provenance unverified)',
           'target_model':'Independent two-body propagation from matched initial state',
           'comparison_duration_s':float(times[-1]),
           'warning':'Residuals compare a simplified model to unverified reference data. Initial sample is matched by construction.'},
       'warnings':warnings
    }
