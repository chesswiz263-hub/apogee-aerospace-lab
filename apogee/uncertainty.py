"""v0.7: bounded Cartesian-state Monte Carlo and linear STM sensitivity.

A Gaussian initial-state uncertainty in an idealized Earth-centered inertial
frame is propagated through a known dynamical force model (two body + optional
J2). The reported intervals are *conditional uncertainty estimates*, not
operational ephemeris error, encounter probability, or calibrated covariance.
"""
from math import sqrt
import numpy as np
from scipy.integrate import solve_ivp

from .physics import state_from_elements, elements_from_state
from .perturbations import ForceModel, acceleration
from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE

CHI2_95_3D = 7.814727903251179  # chi2.ppf(0.95,df=3)


def propagate_uncertainty(altitude_km=400.,inclination_deg=51.6,
                          duration_minutes=90.,position_sigma_m=100.,
                          velocity_sigma_mm_s=10.,monte_carlo_samples=64,
                          seed=42,j2_enabled=False,steps=120):
    values=np.array([altitude_km,inclination_deg,duration_minutes,
                     position_sigma_m,velocity_sigma_mm_s],float)
    if (not np.isfinite(values).all() or not (200<=altitude_km<=5000 and
         0<=inclination_deg<=180 and 1<=duration_minutes<=480 and
         0<position_sigma_m<=10000 and 0<velocity_sigma_mm_s<=1000 and
         monte_carlo_samples in (16,32,64,128) and isinstance(seed,int)
         and 0<=seed<=2147483647 and steps in (60,120,240))):
        raise ValueError('Uncertainty request outside validated bounded prototype limits')

    initial=np.asarray(state_from_elements(RE+altitude_km,0,inclination_deg),float)
    duration=60.*duration_minutes
    times=np.linspace(0,duration,steps+1)
    model=ForceModel(j2=j2_enabled,drag=False)

    def ode(t,y):
        return np.r_[y[3:],acceleration(y[:3],y[3:],model)]

    def propagate(y, t_eval):
        out=solve_ivp(ode,(0,duration),y,t_eval=t_eval,
                       method='DOP853',rtol=8e-11,atol=5e-12,max_step=100)
        if not out.success: raise ValueError('Uncertainty propagation failed')
        return out.y.T

    nominal=propagate(initial,times)
    pos_std_km=position_sigma_m/1000.
    speed_std_km_s=velocity_sigma_mm_s*1e-6
    sigma0=np.array([pos_std_km]*3+[speed_std_km_s]*3)
    rng=np.random.default_rng(seed)
    perturb=rng.normal(size=(monte_carlo_samples,6))*sigma0
    draws=[]
    for offset in perturb:
        trial=propagate(initial+offset,times)
        if np.any(np.linalg.norm(trial[:,:3],axis=1)<=RE):
            raise ValueError('An uncertainty realization entered Earth')
        draws.append(trial)
    stack=np.asarray(draws)
    # Pairwise sampling of the model ensemble around its sample mean,
    # rather than centering covariance on the nominal trajectory.
    mean=stack.mean(axis=0)
    centered=stack-mean
    covariances=np.einsum('nti,ntj->tij',centered,centered)/(monte_carlo_samples-1)
    local_sigmas=[]; pos95=[]; sigmajor=[]
    for t,state in enumerate(nominal):
        r=state[:3];v=state[3:]; radial=r/np.linalg.norm(r)
        normal=np.cross(r,v);normal/=np.linalg.norm(normal)
        along=np.cross(normal,radial)
        E=np.stack([radial,along,normal],axis=0)
        poscov=covariances[t,:3,:3]
        variances=np.diag(E@poscov@E.T)
        local_sigmas.append(np.sqrt(np.maximum(variances,0))*1000)
        eig=np.linalg.eigvalsh(poscov)
        major=sqrt(max(0,float(eig[-1])))*1000
        sigmajor.append(major)
        pos95.append(sqrt(CHI2_95_3D)*major)

    # Central-difference Cartesian STM to the final time, independent of MC.
    # Units: position km and velocity km/s; perturbations scaled to input sigma.
    stm=np.zeros((6,6))
    for axis in range(6):
        h=max(1e-7 if axis<3 else 1e-10, sigma0[axis]*0.1)
        direction=np.zeros(6);direction[axis]=h
        up=propagate(initial+direction,[duration])[-1]
        down=propagate(initial-direction,[duration])[-1]
        stm[:,axis]=(up-down)/(2*h)
    p0=np.diag(sigma0**2)
    linear_final=stm@p0@stm.T
    linear_final=(linear_final+linear_final.T)/2
    mc_final=covariances[-1]
    deviation=stack[:,-1,:3]-nominal[-1,:3]
    empirical95=float(np.mean(np.linalg.norm(deviation,axis=1)*1000<=pos95[-1]))
    radius=np.linalg.norm(nominal[:,:3],axis=1)
    speed=np.linalg.norm(nominal[:,3:],axis=1)
    return {
        'name':'Navigation uncertainty · Monte Carlo and Cartesian STM',
        'spacecraft_mass_kg':750,
        'orbital_elements':elements_from_state(initial).to_dict(),
        'uncertainty':{
            'input_position_sigma_m':position_sigma_m,
            'input_velocity_sigma_mm_s':velocity_sigma_mm_s,
            'assumption':'Independent isotropic Gaussian Cartesian position and velocity errors, same model and initial epoch',
            'monte_carlo_samples':monte_carlo_samples,'random_seed':seed,
            'local_frame':'instantaneous nominal radial / along-track / cross-track (RTN) basis',
            'time_s':times.tolist(),
            'rtn_position_sigma_m':np.asarray(local_sigmas).tolist(),
            'position_max_axis_sigma_m':sigmajor,
            'position_95pct_conservative_radius_m':pos95,
            'initial_covariance_6x6_km_units':p0.tolist(),
            'final_monte_carlo_covariance_6x6_km_units':mc_final.tolist(),
            'final_linear_stm_covariance_6x6_km_units':linear_final.tolist(),
            'final_state_transition_matrix_6x6':stm.tolist(),
            'final_mc_position_rms_m':float(np.sqrt(np.mean(np.sum(deviation**2,axis=1)))*1000),
            'final_stm_position_1sigma_max_m':float(sqrt(max(np.linalg.eigvalsh(linear_final[:3,:3])[-1],0))*1000),
            'empirical_fraction_within_final_95pct_radius':empirical95,
            'interpretation':'The radius uses sqrt(chi2_3,0.95) times largest MC 1-sigma positional axis. It is a conservative 3D Gaussian linear approximation, not an observed 95% coverage guarantee.'},
        'model':{'name':'Sample covariance + central finite-difference STM of Cartesian initial state',
                 'reference_frame':'ideal Earth-centered inertial Cartesian axes, not TEME or ICRF',
                 'integrator':'SciPy DOP853 applied independently to Monte Carlo samples',
                 'force_terms':['Newtonian Earth gravity']+(['Earth J2'] if j2_enabled else []),
                 'assumptions':['No drag, thrust, third bodies, epoch error or process noise',
                                'Only initial-state uncertainty; parameters and dynamics exact within selected model']},
        'metrics':{'samples':len(times),'elapsed_s':duration,
                   'step_s':float(times[1]-times[0]),
                   'min_altitude_km':float(radius.min()-RE),
                   'max_altitude_km':float(radius.max()-RE),
                   'conservation_applicable':False,'max_relative_energy_drift':None,
                   'max_relative_angular_momentum_drift':None},
        'trajectory':{'time_s':times.tolist(),'position_km':nominal[:,:3].tolist(),
                      'velocity_km_s':nominal[:,3:].tolist(),
                      'altitude_km':(radius-RE).tolist(),
                      'speed_km_s':speed.tolist(),
                      'relative_energy_drift':[],'relative_angular_momentum_drift':[]},
        'events':[],
        'warnings':['Uncertainty envelopes represent specified initial Gaussian errors under this chosen force model, NOT measured navigation accuracy.',
                    'No authenticated spacecraft covariance, sensor model, process noise or conjunction collision probability is included.']}
