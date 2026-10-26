"""APOGEE v1.6 navigation integrity demonstration.

Independent DOP853 truth, nominal RK4 EKF, a first-order correlated sensor
error, a sustained biased-sensor fault, and chi-squared innovation detection.
All states use synthetic APOGEE_IDEAL_ECI; not an operational fault monitor.
"""
from dataclasses import dataclass, replace
from math import sqrt

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.stats import chi2

from .constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from .physics import state_from_elements, elements_from_state
from .perturbations import j2_acceleration
from .integrated_nav import _jacobian_r

G0 = 9.80665
FRAME = 'APOGEE_IDEAL_ECI'


@dataclass(frozen=True)
class IntegrityConfig:
    altitude_km: float = 400.0
    inclination_deg: float = 51.6
    duration_minutes: float = 12.
    samples: int = 48
    measurement_sigma_m: float = 30.
    correlation: float = .65
    thrust_error_pct: float = 3.
    fault_start_s: float = 360.
    fault_magnitude_m: float = 220.
    reject_faults: bool = True
    burn_start_s: float = 180.
    burn_duration_s: float = 120.
    j2_enabled: bool = True
    seed: int = 29
    monte_carlo_runs: int = 12
    reference_frame: str = FRAME

    def validate(self):
        vals = [self.altitude_km, self.inclination_deg, self.duration_minutes,
                self.measurement_sigma_m, self.correlation, self.thrust_error_pct,
                self.fault_start_s, self.fault_magnitude_m, self.burn_start_s,
                self.burn_duration_s]
        if not np.isfinite(vals).all():
            raise ValueError('Nonfinite navigation integrity parameter')
        dur = self.duration_minutes * 60.
        if not (200 <= self.altitude_km <= 5000 and 0 <= self.inclination_deg <= 180
                and 3 <= self.duration_minutes <= 60 and 24 <= self.samples <= 180
                and 0.01 <= self.measurement_sigma_m <= 1000
                and 0 <= self.correlation < .99
                and -30 <= self.thrust_error_pct <= 30
                and 0 <= self.fault_start_s <= dur
                and 0 <= self.fault_magnitude_m <= 5000
                and 0 <= self.burn_start_s <= dur
                and 0 <= self.burn_duration_s <= dur - self.burn_start_s + 1e-7
                and 1 <= self.monte_carlo_runs <= 32
                and 0 <= self.seed <= 2147483647
                and self.reference_frame == FRAME):
            raise ValueError('Invalid integrity configuration or incompatible coordinate frame')


def _acceleration(t, x, c, thrust_scale=1.):
    r = x[:3]; v = x[3:]
    norm = np.linalg.norm(r)
    a = -MU * r / norm ** 3
    if c.j2_enabled:
        a += j2_acceleration(r)
    if c.burn_start_s <= t < c.burn_start_s + c.burn_duration_s and np.linalg.norm(v) > 1e-12:
        mass = 750. - 850. * thrust_scale * (t - c.burn_start_s) / (315. * G0)
        a += (850. * thrust_scale / mass / 1000.) * (v / np.linalg.norm(v))
    return a


def _rhs(t, y, c, factor):
    return np.r_[y[3:], _acceleration(t, y, c, factor)]


def _rk4(x, t0, t1, c):
    """Nominal estimate RK4, splitting precisely at thrust switching times."""
    bounds = sorted(set([float(t0), float(t1)] +
                        [float(t) for t in (c.burn_start_s, c.burn_start_s+c.burn_duration_s)
                         if t0+1e-9 < t < t1-1e-9]))
    y = np.asarray(x, dtype=float).copy()
    for a,b in zip(bounds[:-1],bounds[1:]):
        n = max(1,int(np.ceil((b-a)/5.)))
        h = (b-a)/n
        for j in range(n):
            t=a+j*h
            k1=_rhs(t, y, c, 1.)
            k2=_rhs(t+h/2,y+h*k1/2,c,1.)
            k3=_rhs(t+h/2,y+h*k2/2,c,1.)
            k4=_rhs(t+h-1e-10,y+h*k3,c,1.)
            y=y+h*(k1+2*k2+2*k3+k4)/6
    return y


def _truth(times, c, factor):
    """DOP853 integration independent of estimator RK4; split burn discontinuities."""
    y=np.array(state_from_elements(RE+c.altitude_km,0.,c.inclination_deg),float)
    result=[y.copy()]
    for t0,t1 in zip(times[:-1],times[1:]):
        edges=sorted(set([float(t0),float(t1)] +
                         [float(t) for t in (c.burn_start_s,c.burn_start_s+c.burn_duration_s)
                          if t0+1e-9<t<t1-1e-9]))
        for a,b in zip(edges[:-1],edges[1:]):
            # Endpoint stays inside the same engine on/off segment.
            def fun(t,z):
                return _rhs(min(t,b-1e-10),z,c,factor)
            sol=solve_ivp(fun,(a,b),y,method='DOP853',rtol=2e-11,atol=1e-12)
            if not sol.success:raise RuntimeError('Reference integration failed: '+sol.message)
            y=sol.y[:,-1]
        result.append(y.copy())
    return np.array(result)


def _transition(x, t0, t1, c):
    dt=t1-t0
    F=np.zeros((6,6));F[:3,3:]=np.eye(3)
    F[3:,:3]=_jacobian_r(x[:3],c.j2_enabled)
    if c.burn_start_s <= .5*(t0+t1) < c.burn_start_s+c.burn_duration_s:
        v=x[3:]; speed=np.linalg.norm(v)
        if speed>1e-10:
            m=750.-850.*(.5*(t0+t1)-c.burn_start_s)/(315.*G0)
            F[3:,3:]=(850./m/1000./speed)*(np.eye(3)-np.outer(v,v)/speed**2)
    Phi=expm(F*dt)
    Q=np.zeros((6,6))
    acc=1e-7
    Q[:3,:3]=np.eye(3)*acc**2*dt**3/3
    Q[:3,3:]=np.eye(3)*acc**2*dt**2/2
    Q[3:,:3]=Q[:3,3:]
    Q[3:,3:]=np.eye(3)*acc**2*dt
    return Phi,Q


def _measurement_errors(rng, n, c):
    """Stationary AR(1) sensor errors: marginal sigma fixed at the configured value."""
    sigma=c.measurement_sigma_m/1000
    z=np.zeros((n,3));z[0]=rng.normal(0,sigma,3)
    for i in range(1,n):
        z[i]=c.correlation*z[i-1]+sqrt(1-c.correlation**2)*rng.normal(0,sigma,3)
    return z


def _single(c, seed, include_series=False):
    rng=np.random.default_rng(seed)
    times=np.linspace(0.,c.duration_minutes*60.,c.samples+1)
    # Truth thrust factor is an uncertain calibration parameter; estimator holds 1.0.
    calibration=1+c.thrust_error_pct/100 * rng.normal(0,1)
    calibration=max(.1,calibration)
    truth=_truth(times,c,calibration)
    errors=_measurement_errors(rng,len(times),c)
    faulty=np.zeros(len(times),dtype=bool)
    if c.fault_magnitude_m>0:
        faulty=(times >= c.fault_start_s-1e-9)
        errors[faulty,0] += c.fault_magnitude_m/1000
    state=truth[0].copy()
    state[:3] += np.array([.45,-.3,.25]) # km initial estimate error
    state[3:] += [0.00015,-0.00011,0.0001]
    P=np.diag([.6**2]*3+[.0003**2]*3)
    R=np.eye(3)*(c.measurement_sigma_m/1000)**2
    H=np.zeros((3,6));H[:3,:3]=np.eye(3)
    threshold=float(chi2.ppf(.9973,3))
    position_error=[];sigma_rss=[];nis=[];alarms=[];accepted=[]
    min_eigen=float('inf')
    for i,t in enumerate(times):
        if i:
            phi,Q=_transition(state,times[i-1],t,c)
            state=_rk4(state,times[i-1],t,c)
            P=phi@P@phi.T+Q
        # Do not peek at truth here; truth is only used for synthetic measurements.
        obs=truth[i,:3]+errors[i]
        residual=obs-state[:3]
        S=H@P@H.T+R
        score=float(residual@np.linalg.solve(S,residual))
        alarm=score>threshold
        if not(alarm and c.reject_faults):
            gain=np.linalg.solve(S,(P@H.T).T).T
            state=state+gain@residual
            M=np.eye(6)-gain@H
            P=M@P@M.T+gain@R@gain.T
        P=(P+P.T)/2
        min_eigen=min(min_eigen,float(np.linalg.eigvalsh(P)[0]))
        position_error.append(float(np.linalg.norm(state[:3]-truth[i,:3])*1000))
        sigma_rss.append(float(np.sqrt(np.trace(P[:3,:3]))*1000))
        nis.append(score);alarms.append(bool(alarm));accepted.append(not(alarm and c.reject_faults))
    alarm_arr=np.asarray(alarms)
    have_fault=bool(faulty.any())
    counts={
        'pre_fault_alarms':int(np.sum(alarm_arr & ~faulty)),
        'pre_fault_measurements':int(np.sum(~faulty)),
        'fault_alarms':int(np.sum(alarm_arr & faulty)),
        'fault_measurements':int(np.sum(faulty)),
        'detected_fault':bool(np.any(alarm_arr & faulty)) if have_fault else None,
        'first_fault_alarm_s':float(times[np.where(alarm_arr & faulty)[0][0]]) if np.any(alarm_arr & faulty) else None,
    }
    result={
        'seed':int(seed),'true_thrust_calibration_factor':float(calibration),
        'final_position_error_m':position_error[-1],
        'final_position_1sigma_rss_m':sigma_rss[-1],
        'final_error_over_1sigma_rss':position_error[-1]/max(sigma_rss[-1],1e-9),
        'mean_nis':float(np.mean(nis)), 'gate_threshold_chi2_3dof':threshold,
        'rejected_measurements':int(np.sum(~np.asarray(accepted))),
        'min_covariance_eigenvalue':min_eigen,**counts}
    if include_series:
        result.update({'time_s':times.tolist(),'truth_states':truth.tolist(),
            'position_error_m':position_error,'position_1sigma_rss_m':sigma_rss,
            'innovation_nis':nis,'alarm':alarms,'fault_active':faulty.tolist(),
            'accepted_measurement':accepted})
    return result


def simulate_integrity(c:IntegrityConfig):
    c.validate()
    seeds=[c.seed+i for i in range(c.monte_carlo_runs)]
    results=[_single(c,seed,include_series=(i==0)) for i,seed in enumerate(seeds)]
    display=results[0]
    final_errors=np.array([v['final_position_error_m'] for v in results])
    faulted=[v['detected_fault'] for v in results if v['detected_fault'] is not None]
    truth=np.array(display['truth_states'])
    final={
        'name':'Navigation Integrity · correlated sensors and fault isolation',
        'input':c.__dict__.copy(),
        'trajectory':{'time_s':display['time_s'],'position_km':truth[:,:3].tolist(),
                      'velocity_km_s':truth[:,3:].tolist(),
                      'altitude_km':(np.linalg.norm(truth[:,:3],axis=1)-RE).tolist(),
                      'speed_km_s':np.linalg.norm(truth[:,3:],axis=1).tolist()},
        'orbital_elements':elements_from_state(truth[0]).to_dict(),
        'model':{'name':'DOP853 independent truth / nominal RK4 Cartesian EKF',
                 'reference_frame':FRAME,
                 'assumptions':['Ideal prograde engine; wet mass modeled analytically during burn',
                   'Independent Gaussian marginal noise innovations are assumed by filter',
                   'Actual sensor error is time-correlated AR(1) with fixed marginal sigma',
                   'Sensor step fault is a persistent positive inertial-X bias',
                   'Truth engine scale is sampled once per run and hidden from estimator',
                   'Synthetic Cartesian fixes, no real stations or star tracker',
                   'Single-measurement three-dimensional chi-square gate; no FDIR recovery policy']},
        'metrics':{'samples':len(display['time_s']),
                   'elapsed_s':display['time_s'][-1],
                   'step_s':display['time_s'][-1]/c.samples,
                   'min_altitude_km':float(np.min(np.linalg.norm(truth[:,:3],axis=1))-RE),
                   'max_altitude_km':float(np.max(np.linalg.norm(truth[:,:3],axis=1))-RE),
                   'conservation_applicable':False},
        'integrity':{
            **{k:v for k,v in display.items() if k!='truth_states'},
            'monte_carlo':{'runs':c.monte_carlo_runs,
                          'median_final_error_m':float(np.median(final_errors)),
                          'worst_final_error_m':float(np.max(final_errors)),
                          'false_alarm_rate_pre_fault':float(sum(v['pre_fault_alarms'] for v in results)/
                                                            max(1,sum(v['pre_fault_measurements'] for v in results))),
                          'fault_detection_rate':float(np.mean(faulted)) if faulted else None,
                          'false_alarms_total':sum(v['pre_fault_alarms'] for v in results),
                          'fault_detected_count':sum(bool(x) for x in faulted),
                          'per_run':[{k:v for k,v in run.items()
                                      if k not in ('time_s','truth_states','position_error_m',
                                                 'position_1sigma_rss_m','innovation_nis','alarm',
                                                 'fault_active','accepted_measurement')}
                                     for run in results]}},
        'warnings':['Synthetic bounded demonstration, not operational GN&C or a collision avoidance system.',
                    'AR(1) correlated sensor noise violates the EKF white measurement-noise assumption.',
                    'A chi-square innovation threshold does not guarantee a stated false-alarm rate when errors are temporally correlated.',
                    'The fault gate may reject valid measurements when the model or covariance is inconsistent.'],
    }
    return final
