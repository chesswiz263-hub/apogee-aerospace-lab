"""APOGEE v0.9: educational finite-burn closed-loop orbital guidance.

All station locations are hypothetical and rotate with an *idealized* spherical
Earth; no ITRF/UT1 coordinate rigor. Range/range-rate visibility uses a geometric
horizon, no atmosphere/refraction. The EKF never sees hidden truth.

This is a deliberately bounded research demonstration, NEVER flight software.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import acos, cos, sin, pi, exp, log, sqrt
import numpy as np
from scipy.integrate import solve_ivp

from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state
from .perturbations import ForceModel, acceleration
from .guidance import _flow, _dynamics, _variational, target_velocity_impulse

OMEGA = 7.2921150e-5  # rad/s; fixed idealized Earth rotation
G0 = 9.80665          # m/s^2


@dataclass(frozen=True)
class RobustConfig:
    altitude_km: float = 400.
    inclination_deg: float = 51.6
    duration_minutes: float = 90.
    measurement_interval_s: int = 90
    range_sigma_m: float = 15.
    range_rate_sigma_m_s: float = .03
    initial_position_error_m: float = 600.
    initial_velocity_error_m_s: float = .4
    dry_mass_kg: float = 600.
    fuel_mass_kg: float = 150.
    thrust_n: float = 850.
    specific_impulse_s: float = 315.
    max_total_delta_v_m_s: float = 2.
    corrections: int = 3
    guidance_enabled: bool = True
    truth_j2_enabled: bool = True
    filter_j2_enabled: bool = True
    process_acceleration_sigma_mm_s2: float = .001
    thrust_error_pct: float = 2.
    pointing_error_deg: float = .25
    station_count: int = 18
    min_elevation_deg: float = 5.
    seed: int = 42

    def validate(self):
        vals = np.array([v for v in self.__dict__.values() if isinstance(v,(int,float)) and not isinstance(v,bool)],float)
        if not np.isfinite(vals).all():raise ValueError('All numeric inputs must be finite')
        if not (200<=self.altitude_km<=2000 and 0<=self.inclination_deg<=180
                and 30<=self.duration_minutes<=180 and self.measurement_interval_s in (30,60,90,120,180,300)
                and .01<=self.range_sigma_m<=1000 and .001<=self.range_rate_sigma_m_s<=5
                and 0<=self.initial_position_error_m<=5000 and 0<=self.initial_velocity_error_m_s<=5
                and 50<=self.dry_mass_kg<=100000 and 0<=self.fuel_mass_kg<=50000
                and 100<=self.thrust_n<=100000 and 50<=self.specific_impulse_s<=5000
                and 0<=self.max_total_delta_v_m_s<=15 and self.corrections in (0,1,2,3,4)
                and self.station_count in (3,9,18) and 0<=self.min_elevation_deg<=20
                and 0<=self.process_acceleration_sigma_mm_s2<=1
                and 0<=self.thrust_error_pct<=20 and 0<=self.pointing_error_deg<=10
                and isinstance(self.seed,int) and 0<=self.seed<=2147483647):
            raise ValueError('Robust GNC input outside bounded demonstration limits')
        if not all(isinstance(v,bool) for v in (self.guidance_enabled,self.truth_j2_enabled,self.filter_j2_enabled)):
            raise ValueError('Mode switches must be boolean')
        if self.dry_mass_kg+self.fuel_mass_kg<=0:raise ValueError('Invalid spacecraft mass')


def stations(count: int):
    """Fixed *fictional* station network, latitude/longitude in degrees.

    Networks are deterministic and include a station at the t=0 subpoint.
    """
    if count==3:
        return [(0.,0.),(45.,75.),(-45.,-110.)]
    if count==9:
        return [(0.,float(k*45)) for k in range(8)]+[(45.,45.)]
    if count==18:
        return [(0.,float(k*30)) for k in range(12)]+[(45.,float(k*60+30)) for k in range(6)]
    raise ValueError('Unsupported station network')


def station_state(lat_deg, lon_deg, elapsed_s):
    lat=np.deg2rad(lat_deg);lon=np.deg2rad(lon_deg)+OMEGA*elapsed_s
    r=RE*np.array([cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat)])
    v=np.cross([0.,0.,OMEGA],r)
    return r,v


def ground_observation(state, lat, lon, time_s, min_elevation_deg=5.):
    """Return [range_km, range_rate_km_s], H(2x6), elevation deg.

    A negative/elevation-too-small angle indicates no viable measurement.
    """
    r,v=station_state(lat,lon,time_s)
    rho_vec=np.asarray(state[:3],float)-r
    vrel=np.asarray(state[3:6],float)-v
    rho=float(np.linalg.norm(rho_vec))
    u=rho_vec/rho
    radial=float(u@vrel)
    elev=float(np.rad2deg(np.arcsin(np.clip(float(u@r/RE),-1,1))))
    if elev < min_elevation_deg:return None
    H=np.zeros((2,6))
    H[0,:3]=u
    H[1,:3]=(vrel-u*radial)/rho
    H[1,3:]=u
    return np.array([rho,radial]),H,elev


def ekf_range_rate_update(x,P,observation,station,time_s,sigma_range_km,sigma_rate_km_s,min_elev_deg=0.):
    predicted=ground_observation(x,*station,time_s,min_elev_deg)
    if predicted is None:return x,P,None,None
    hx,H,_=predicted
    innovation=np.asarray(observation,float)-hx
    R=np.diag([sigma_range_km**2,sigma_rate_km_s**2])
    S=H@P@H.T+R
    K=np.linalg.solve(S,(P@H.T).T).T
    x_new=x+K@innovation
    J=np.eye(6)-K@H
    p_new=J@P@J.T+K@R@K.T
    p_new=(p_new+p_new.T)/2
    nis=float(innovation@np.linalg.solve(S,innovation))
    return x_new,p_new,innovation,nis


def process_noise_covariance(dt,acceleration_sigma_mm_s2):
    """Discrete white acceleration interval approximation (PSD inflation only).

    Treats unknown constant acceleration *per interval* with independent axes;
    this does not inject physical acceleration noise into the hidden truth.
    """
    q=(acceleration_sigma_mm_s2*1e-6)**2  # km/s^2^2
    I=np.eye(3)
    return q*np.block([[I*dt**4/4,I*dt**3/2],[I*dt**3/2,I*dt**2]])


def covariance_predict(P,phi,dt,sigma):
    result=phi@P@phi.T+process_noise_covariance(dt,sigma)
    return (result+result.T)/2


def finite_burn(state7,start_s,duration_s,model,thrust_n,isp_s,direction,with_stm=False):
    """Propagate a time-steered inertial thrust vector with tank mass depletion.

    Direction held constant in inertial coordinates for each short burn. Mass is
    bounded by precomputed propellant-feasible duration; no zero-mass division.
    """
    x=np.asarray(state7,float)
    if x.size!=7 or x[6]<=0 or not (duration_s>0 and thrust_n>0 and isp_s>0):
        raise ValueError('Invalid burn inputs')
    direction=np.asarray(direction,float);norm=float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm<1e-14:raise ValueError('Invalid thrust direction')
    direction=direction/norm
    dm=thrust_n/(isp_s*G0)
    if duration_s*dm >= x[6]:raise ValueError('Burn would exhaust spacecraft total mass')
    initial=np.r_[x,np.eye(6).ravel()] if with_stm else x
    def rhs(t,y):
        dydt=np.r_[_dynamics(t,y[:6],model),-dm]
        dydt[3:6]+=(thrust_n/(y[6]*1000))*direction
        if with_stm:
            # External held-inertial thrust and mass flow do not depend on x6.
            Aphi=_variational(t,np.r_[y[:6],y[7:]],model)[6:]
            return np.r_[dydt,Aphi]
        return dydt
    sol=solve_ivp(rhs,(start_s,start_s+duration_s),initial,method='DOP853',rtol=2e-10,atol=1e-12,max_step=4.)
    if not sol.success or not np.isfinite(sol.y).all():raise ValueError('Finite burn integration failed')
    last=sol.y[:,-1]
    if np.linalg.norm(last[:3])<=RE:raise ValueError('Burn intersects Earth')
    return (last[:7],last[7:].reshape(6,6)) if with_stm else last


def finite_burn_duration(command_dv_m_s,mass_kg,thrust_n,isp_s,fuel_kg,max_duration_s):
    """Tsiolkovsky mapping to burn duration, capped by fuel and timing.

    Returns nominal duration and corresponding model delivered ∆v.
    """
    if command_dv_m_s<=0 or fuel_kg<=0:return 0.,0.
    prop_req=mass_kg*(-np.expm1(-command_dv_m_s/(isp_s*G0)))
    dm=thrust_n/(isp_s*G0)
    duration=max(0.,min(prop_req/dm,fuel_kg/dm,max_duration_s))
    dv=isp_s*G0*(-np.log1p(-duration*dm/mass_kg))
    return float(duration),float(dv)


def _rotate_direction(direction,angle_rad,rng):
    # A uniformly chosen axis perpendicular to the commanded inertial vector.
    if angle_rad<=0:return direction.copy()
    arbitrary=rng.normal(size=3)
    tangent=arbitrary-direction*np.dot(arbitrary,direction)
    n=np.linalg.norm(tangent)
    if n<1e-12:return direction.copy()
    return cos(angle_rad)*direction+sin(angle_rad)*tangent/n


def simulate_robust_guidance(config: RobustConfig|None=None,**kwargs):
    if config is None:config=RobustConfig(**kwargs)
    config.validate()
    truth_model=ForceModel(j2=config.truth_j2_enabled,drag=False)
    filter_model=ForceModel(j2=config.filter_j2_enabled,drag=False)
    t_end=config.duration_minutes*60.
    t_grid=np.unique(np.r_[np.arange(0.,t_end,config.measurement_interval_s),t_end]);t_grid.sort()
    ref0=np.array(state_from_elements(RE+config.altitude_km,0.,config.inclination_deg),float)
    radial=ref0[:3]/np.linalg.norm(ref0[:3])
    n=np.cross(ref0[:3],ref0[3:]);n/=np.linalg.norm(n)
    along=np.cross(n,radial)
    truth=np.r_[ref0+np.r_[radial*config.initial_position_error_m/1000,
                              along*config.initial_velocity_error_m_s/1000],
                 config.dry_mass_kg+config.fuel_mass_kg]
    estimate=ref0.copy();base=truth[:6].copy();ref=ref0.copy()
    model_mass=float(truth[6]);fuel_used=0.
    ppos=max(100,config.initial_position_error_m*2,config.range_sigma_m*4)/1000
    pvel=max(.2,config.initial_velocity_error_m_s*2)/1000
    P=np.diag([ppos**2]*3+[pvel**2]*3)
    rng=np.random.default_rng(config.seed)
    sites=stations(config.station_count)
    sigma_r=config.range_sigma_m/1000; sigma_v=config.range_rate_sigma_m_s/1000
    chosen=[]
    choices=np.flatnonzero((t_grid>=.2*t_end)&(t_grid<=.7*t_end))
    if config.corrections and len(choices):
        chosen=sorted(set(int(choices[int(round(k))]) for k in np.linspace(0,len(choices)-1,config.corrections)))
    target=_flow(ref0,(0.,t_end),truth_model)
    current=0.
    hist_t=[0.];hist_true=[truth.copy()];hist_ref=[ref.copy()];hist_base=[base.copy()];hist_est=[estimate.copy()]
    sigmas=[float(np.sqrt(np.trace(P[:3,:3]))*1000)];nis_values=[None]
    count=0;visible_epochs=0;events=[];budget_used=0.
    actual_integrated_dv=0.;measurement_records=[]

    def record(t,nis=None):
        hist_t.append(float(t));hist_true.append(truth.copy());hist_ref.append(ref.copy())
        hist_base.append(base.copy());hist_est.append(estimate.copy())
        sigmas.append(float(np.sqrt(max(0,np.trace(P[:3,:3])))*1000));nis_values.append(nis)

    for k in range(1,len(t_grid)):
        t=float(t_grid[k]);dt=t-current
        if dt>1e-9:
            truth[:6]=_flow(truth[:6],(current,t),truth_model)
            base=_flow(base,(current,t),truth_model)
            ref=_flow(ref,(current,t),truth_model)
            estimate,phi=_flow(estimate,(current,t),filter_model,variational=True)
            P=covariance_predict(P,phi,dt,config.process_acceleration_sigma_mm_s2)
        current=t
        epoch_nis=[];visible=[]
        if k<len(t_grid)-1 or abs(t/config.measurement_interval_s-round(t/config.measurement_interval_s))<1e-9:
            for idx,station in enumerate(sites):
                measured=ground_observation(truth[:6],*station,t,config.min_elevation_deg)
                if measured is None:continue
                visible.append(idx)
                noisy=measured[0]+rng.normal(size=2)*[sigma_r,sigma_v]
                estimate,P,innovation,nis=ekf_range_rate_update(
                    estimate,P,noisy,station,t,sigma_r,sigma_v,config.min_elevation_deg)
                # If estimate sees horizon mismatch, skip; do not leak truth into its estimator.
                if innovation is not None:
                    epoch_nis.append(nis);count+=1
                    measurement_records.append({'time_s':t,'station_index':idx,
                        'innovation_range_m':float(innovation[0]*1000),
                        'innovation_rate_m_s':float(innovation[1]*1000),'nis':nis})
        if visible:visible_epochs+=1
        record(t,float(np.mean(epoch_nis)) if epoch_nis else None)
        if config.guidance_enabled and k in chosen and budget_used<config.max_total_delta_v_m_s-1e-8 and truth[6]>config.dry_mass_kg+1e-9:
            remaining=config.max_total_delta_v_m_s-budget_used
            command,asked,before,_=target_velocity_impulse(estimate,target,t,t_end,filter_model,remaining)
            command_mag=float(np.linalg.norm(command)*1000)
            if command_mag>1e-5:
                duration,actual_nominal_dv=finite_burn_duration(command_mag,model_mass,
                    config.thrust_n,config.specific_impulse_s,truth[6]-config.dry_mass_kg,
                    min(.4*config.measurement_interval_s,.4*(float(t_grid[min(k+1,len(t_grid)-1)])-t)))
                if duration>1e-7:
                    unit=command/np.linalg.norm(command)
                    thrust_actual=config.thrust_n*(1+rng.normal(0,config.thrust_error_pct/100))
                    thrust_actual=max(1.,thrust_actual)
                    angular=float(rng.normal(0,np.deg2rad(config.pointing_error_deg)))
                    actual_direction=_rotate_direction(unit,angular,rng)
                    # Actual engine dispersion can consume fuel faster than the
                    # nominal command. Respect dry mass even in this case.
                    actual_fuel_remaining=truth[6]-config.dry_mass_kg
                    max_actual_duration=(.999999*actual_fuel_remaining*config.specific_impulse_s*G0/thrust_actual)
                    duration=min(duration,max_actual_duration)
                    nominal_dm=config.thrust_n/(config.specific_impulse_s*G0)
                    actual_nominal_dv=config.specific_impulse_s*G0*(-np.log1p(-duration*nominal_dm/model_mass))
                    m_start=truth[6]
                    truth=finite_burn(truth,t,duration,truth_model,thrust_actual,config.specific_impulse_s,actual_direction)
                    # Filter predicts nominal engine, not hidden execution error.
                    nominal,phi_b=finite_burn(np.r_[estimate,model_mass],t,duration,filter_model,
                        config.thrust_n,config.specific_impulse_s,unit,with_stm=True)
                    estimate=nominal[:6];model_mass=float(nominal[6])
                    P=covariance_predict(P,phi_b,duration,config.process_acceleration_sigma_mm_s2)
                    base=_flow(base,(t,t+duration),truth_model)
                    ref=_flow(ref,(t,t+duration),truth_model)
                    actual_dv=config.specific_impulse_s*G0*log(m_start/truth[6])
                    actual_integrated_dv+=actual_dv
                    budget_used+=actual_nominal_dv
                    fuel_used=(config.dry_mass_kg+config.fuel_mass_kg)-truth[6]
                    current=t+duration
                    events.append({'type':'guidance_burn','name':f'Finite correction {len(events)+1}',
                      'time_s':t,'end_s':current,'duration_s':duration,'delta_v_m_s':actual_nominal_dv,
                      'commanded_delta_v_m_s':command_mag,'actual_thrust_integral_delta_v_m_s':actual_dv,
                      'estimated_command_vector_m_s':(unit*actual_nominal_dv).tolist(),
                      'direction_command_inertial':unit.tolist(), 'actual_thrust_n':thrust_actual,
                      'fuel_used_kg':float(fuel_used),'position_km':truth[:3].tolist(),
                      'remaining_budget_m_s':max(0.,config.max_total_delta_v_m_s-budget_used),
                      'terminal_weighted_residual_before':before})
                    record(current)
        if not (np.isfinite(truth).all() and np.isfinite(estimate).all() and np.isfinite(P).all()):
            raise ValueError('Nonfinite robust guidance state')
        if np.linalg.eigvalsh(P).min() < -1e-8:
            raise ValueError('Robust guidance covariance invalid')
    h=np.array(hist_true);r=np.array(hist_ref);b=np.array(hist_base);e=np.array(hist_est)
    times=np.asarray(hist_t)
    poserror=np.linalg.norm(h[:,:3]-r[:,:3],axis=1)*1000
    baseline=np.linalg.norm(b[:,:3]-r[:,:3],axis=1)*1000
    estimation=np.linalg.norm(h[:,:3]-e[:,:3],axis=1)*1000
    alt=np.linalg.norm(h[:,:3],axis=1)-RE
    v=np.linalg.norm(h[:,3:6],axis=1)
    innov=np.array([x['nis'] for x in measurement_records],float)
    engine_accumulated=np.array([sum(ev['actual_thrust_integral_delta_v_m_s'] for ev in events if ev['end_s']<=q+1e-8) for q in times],float)
    metrics={'final_guided_position_error_m':float(poserror[-1]),
             'final_unguided_position_error_m':float(baseline[-1]),
             'improvement_pct':float(100*(1-poserror[-1]/max(baseline[-1],1e-12))),
             'final_estimation_position_error_m':float(estimation[-1]),
             'final_estimated_position_1sigma_rss_m':sigmas[-1],
             'final_guided_velocity_error_m_s':float(np.linalg.norm(h[-1,3:6]-r[-1,3:6])*1000),
             'total_guidance_delta_v_m_s':float(budget_used),
             'actual_integrated_delta_v_m_s':float(actual_integrated_dv),
             'fuel_consumed_kg':float(fuel_used),
             'initial_propellant_kg':float(config.fuel_mass_kg),
             'fuel_remaining_kg':float(h[-1,6]-config.dry_mass_kg),
             'correction_count':len(events),'measurement_count':count,
             'visible_measurement_epochs':visible_epochs,
             'mean_nis':float(innov.mean()) if innov.size else None,
             'final_nees':float((h[-1,:6]-e[-1])@np.linalg.solve(P,(h[-1,:6]-e[-1]))) if np.linalg.eigvalsh(P).min()>1e-18 else None,
             'samples':len(times),'step_s':float(np.median(np.diff(times))),
             'elapsed_s':t_end,'min_altitude_km':float(alt.min()),'max_altitude_km':float(alt.max()),
             'conservation_applicable':False,'max_relative_energy_drift':None,
             'max_relative_angular_momentum_drift':None}
    return {'name':'Robust GNC · finite feedback and hypothetical ground station network',
            'input':dict(config.__dict__),
            'model':{'name':'Range/range-rate EKF + bounded finite-burn guidance',
               'reference_frame':'idealized inertial frame at arbitrarily defined Greenwich angle 0, not ITRF/ICRF',
               'integrator':'DOP853 including RK dynamics + STM',
               'force_terms':['Earth Newtonian gravity']+(['Truth Earth J2'] if config.truth_j2_enabled else []),
               'assumptions':['Fictional spherical-Earth station network with fixed ideal Earth spin','Geometric elevation mask with no Earth orientation corrections or atmospheric delays',
                 'Two-way timing, measurement biases, light time and station clock errors not modeled',
                 'Engine thrust fixed in inertial coordinates during each finite burn; no attitude dynamics',
                 'Filter may mismatch truth J2; process acceleration noise inflates covariance but is not injected into truth']},
            'spacecraft_mass_kg':float(h[0,6]),'orbital_elements':elements_from_state(ref0).to_dict(),
            'robust':{'station_locations_lat_lon_deg':sites,'measurement_records':measurement_records,
                'range_observation_model':'ground station slant range plus line-of-sight range rate, subject to geometric elevation mask',
                'filter_covariance_6x6_km_units':P.tolist(),
                'maneuver_model':'mass-depleting finite duration inertial thrust with seeded thrust magnitude and pointing error',
                'command_budget_type':'nominal rocket-equation delta v; actual thrust-integrated delta v reported separately',
                'covariance_statistics':'NIS each visible 2D range/rate update; final NEES from simulated truth (diagnostic only)'},
            'guidance':{'time_s':times.tolist(), 'guided_position_error_m':poserror.tolist(),
                'unguided_position_error_m':baseline.tolist(),
                'estimate_position_error_m':estimation.tolist(),
                'estimate_position_1sigma_rss_m':sigmas,
                'normalized_innovation_squared':nis_values,
                'reference_position_km':r[:,:3].tolist(),
                'baseline_position_km':b[:,:3].tolist(),
                'final_estimate_6_state_km_units':e[-1].tolist()},
            'trajectory':{'time_s':times.tolist(),'position_km':h[:,:3].tolist(),
                'velocity_km_s':h[:,3:6].tolist(), 'altitude_km':alt.tolist(),
                'speed_km_s':v.tolist(),'wet_mass_kg':h[:,6].tolist(),
                'propellant_remaining_kg':(h[:,6]-config.dry_mass_kg).tolist(),
                'thrust_active':[any(ev['time_s']<=float(q)<ev['end_s'] for ev in events) for q in times],
                'cumulative_ideal_delta_v_m_s':engine_accumulated.tolist(),
                'relative_energy_drift':[], 'relative_angular_momentum_drift':[]},
            'metrics':metrics,'events':events,
            'warnings':['Research/education only: do not use for operational spacecraft navigation or maneuvers.',
                 'Ground stations and Earth orientation are hypothetical; geometry is not ITRF precise.',
                 'Finite burns use a held inertial thrust axis and no attitude or gravity-gradient control.',
                 'One-step STM targeting is not guaranteed to improve every seed, measurement geometry or dynamics mismatch.',
                 'Filter consistency diagnostics do not constitute calibrated flight navigation accuracy.']}


def robustness_sweep(config: RobustConfig|None=None, seeds=5,**kwargs):
    """Bounded seeded ensemble. Each run uses the same fixed initial errors.

    No inference of operational reliability from a small synthetic ensemble.
    """
    if config is None:config=RobustConfig(**kwargs)
    config.validate()
    if not isinstance(seeds,int) or not 2<=seeds<=10:raise ValueError('Use 2 through 10 seeds')
    results=[]
    for i in range(seeds):
        from dataclasses import replace
        r=simulate_robust_guidance(replace(config,seed=config.seed+i))
        m=r['metrics']
        results.append({'seed':config.seed+i,'guided_final_error_m':m['final_guided_position_error_m'],
                        'unguided_final_error_m':m['final_unguided_position_error_m'],
                        'nominal_total_delta_v_m_s':m['total_guidance_delta_v_m_s'],
                        'fuel_used_kg':m['fuel_consumed_kg'],'measurements':m['measurement_count'],
                        'final_nees':m['final_nees'],'mean_nis':m['mean_nis']})
    errors=np.array([r['guided_final_error_m'] for r in results])
    return {'input':dict(config.__dict__),'seed_count':seeds,'runs':results,
            'summary':{'median_guided_position_error_m':float(np.median(errors)),
                       'max_guided_position_error_m':float(errors.max()),
                       'min_guided_position_error_m':float(errors.min()),
                       'median_fuel_consumed_kg':float(np.median([r['fuel_used_kg'] for r in results])),
                       'runs_improved_vs_baseline':sum(r['guided_final_error_m']<r['unguided_final_error_m'] for r in results)},
            'warning':'Small fixed-initial-error synthetic ensemble, not statistically calibrated mission reliability or success probability.'}
