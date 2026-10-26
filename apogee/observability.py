"""APOGEE v1.9: finite-burn engine observability and robust batch estimation.

An explicitly *batch* (noncausal) local seven-parameter navigation study.
It is separate from the v1.8 online 13-error-state attitude/orbit filter.
State is Earth-centered two-body Cartesian position/velocity plus scalar engine
thrust scale. DOP853 separately integrates nonlinear synthetic truth;
variational ODEs generate the local sensitivity matrix. Three estimators operate
on exactly the same synthetic measurements per Monte Carlo seed.

Not a flight-certified observability analysis or fault recovery system.
"""
from dataclasses import dataclass
from math import sqrt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import cho_factor, cho_solve
from scipy.stats import chi2
from .physics import MU, R_EARTH, elements_from_state

G0=9.80665

@dataclass(frozen=True)
class ObservabilityConfig:
    altitude_km: float=400.
    inclination_deg: float=51.6
    duration_minutes: float=12.
    samples: int=61
    observation_interval_s: float=30.
    observation_start_s: float=0.
    observation_sigma_m: float=30.
    noise_correlation: float=0.
    burn_start_s: float=180.
    burn_duration_s: float=120.
    true_thrust_scale: float=1.05
    fault_bias_m: float=250.
    fault_start_s: float=360.
    fault_duration_s: float=60.
    estimator: str='huber'
    monte_carlo_runs: int=48
    seed: int=29
    reference_frame: str='APOGEE_IDEAL_ECI'

    def validate(self):
        numbers=[self.altitude_km,self.inclination_deg,self.duration_minutes,self.observation_interval_s,
                 self.observation_start_s,self.observation_sigma_m,self.noise_correlation,self.burn_start_s,
                 self.burn_duration_s,self.true_thrust_scale,self.fault_bias_m,self.fault_start_s,self.fault_duration_s]
        if (not np.all(np.isfinite(numbers)) or not 200<=self.altitude_km<=5000
            or not 0<=self.inclination_deg<=180 or not 5<=self.duration_minutes<=60
            or not 31<=self.samples<=241 or not 10<=self.observation_interval_s<=300
            or not 0<=self.observation_start_s<=3600 or not .01<=self.observation_sigma_m<=1000
            or not 0<=self.noise_correlation<.99 or not 0<=self.burn_start_s<=3600
            or not 0<=self.burn_duration_s<=360 or not .7<=self.true_thrust_scale<=1.3
            or not 0<=self.fault_bias_m<=5000 or not 0<=self.fault_start_s<=3600
            or not 0<=self.fault_duration_s<=3600
            or self.estimator not in ('least_squares','huber','gate')
            or not 1<=self.monte_carlo_runs<=256 or not 0<=self.seed<=2147483647
            or self.reference_frame!='APOGEE_IDEAL_ECI'):
            raise ValueError('Invalid v1.9 observability configuration')
        if self.observation_start_s>=60*self.duration_minutes:
            raise ValueError('Observation start must precede mission end')
        if self.burn_start_s+self.burn_duration_s>self.duration_minutes*60+1.e-9:
            raise ValueError('Finite burn must finish before mission end')


def _base(c):
    r=R_EARTH+c.altitude_km
    speed=sqrt(MU/r)
    i=np.deg2rad(c.inclination_deg)
    state=np.array([r,0,0,0,speed*np.cos(i),speed*np.sin(i)],float)
    u=state[3:]/np.linalg.norm(state[3:])
    return state,u


def _propagate(c,times,alpha=1.,initial=None,sensitivities=False):
    """Solve full Earth two-body trajectory and 6x7 tangent dynamics in km/s.

    Thrust is a fixed inertial direction; engine mass decreases during burns.
    The burn starts/ends are integration segment boundaries to prevent inaccurate
    quadrature across throttle discontinuities.
    """
    state,u=_base(c)
    if initial is not None: state=state+np.asarray(initial,float)
    if sensitivities:
        X=np.zeros((6,7));X[:,:6]=np.eye(6)
        y=np.r_[state,X.flatten()]
    else:y=state.copy()
    flow=850/(315*G0)
    b0,b1=c.burn_start_s,c.burn_start_s+c.burn_duration_s
    def fun(t,z):
        r=z[:3];v=z[3:6];rad=np.linalg.norm(r)
        accel=-MU*r/rad**3
        firing=b0<=t<b1 and c.burn_duration_s>0
        mass=max(600.,750.-alpha*flow*np.clip(t-b0,0,c.burn_duration_s))
        if firing:accel=accel+(alpha*850/mass/1000)*u
        result=np.r_[v,accel]
        if not sensitivities:return result
        gravity=-MU*(np.eye(3)/rad**3-3*np.outer(r,r)/rad**5)
        A=np.zeros((6,6));A[:3,3:6]=np.eye(3);A[3:6,:3]=gravity
        X=z[6:].reshape(6,7)
        dX=A@X
        if firing:
            elapsed=np.clip(t-b0,0,c.burn_duration_s)
            dX[3:6,6]+=(850/1000)*(1/mass + alpha*flow*elapsed/mass**2)*u
        return np.r_[result,dX.flatten()]
    all_times=np.array(sorted(set(float(t) for t in times)|{0.,float(60*c.duration_minutes)}))
    if not np.all(np.diff(all_times)>0):raise ValueError('Duplicate times')
    breaks=sorted(set([0.,float(60*c.duration_minutes),float(b0),float(b1)]))
    solutions={0.:y.copy()}
    current=y.copy()
    for a,b in zip(breaks[:-1],breaks[1:]):
        if b-a<1e-10:continue
        target=all_times[(all_times>a+1e-10)&(all_times<=b+1e-10)]
        grid=np.r_[target,b] if len(target)==0 or target[-1]<b-1e-10 else target
        grid=np.unique(grid)
        res=solve_ivp(fun,(a,b),current,t_eval=grid,method='DOP853',rtol=1e-11,atol=1e-12,
                      max_step=10.)
        if not res.success:raise RuntimeError('DOP853 propagation failed: '+res.message)
        for t,z in zip(res.t,res.y.T):solutions[round(float(t),8)]=z
        current=res.y[:,-1]
    sampled=np.array([solutions[round(float(t),8)] for t in times])
    return sampled[:,:6],(sampled[:,6:].reshape(-1,6,7) if sensitivities else None)


def _design(c,t,stms):
    # Parameter coordinates: initial position m, initial velocity m/s, engine scale unitless.
    H=stms[:,:3,:].copy()
    H[:,:,6]*=1000.   # r in km per dimensionless scale, observed m
    return H


def information_analysis(H,observation_mask,noise_sigma_m,prior_scale=0.20):
    """Schur complement / orthogonal projection removes six nuisance states.

    The observational (no-prior) information on engine scale is zero if its
    column is in the span of the initial-state sensitivity columns.
    """
    A=H[observation_mask].reshape(-1,7)/noise_sigma_m
    if A.shape[0]<7:
        return {'observable':False,'effective_scale_information':0.,'scale_1sigma_from_data':None,
                'scale_signal_to_prior':0.,'independent_measurements':int(observation_mask.sum()),
                'rank':int(np.linalg.matrix_rank(A))}
    nuisance=A[:,:6]
    s=np.linalg.svd(nuisance,full_matrices=False)
    U,S,VT=s
    rank=int(np.sum(S>max(S[0],1.)*1e-10))
    q=A[:,6]-U[:,:rank]@(U[:,:rank].T@A[:,6])
    info=float(max(0.,q@q))
    # A scale that is barely informed compared to the declared prior is not usable.
    identifiable=bool(info>25 and info*prior_scale**2>4)
    return {'observable':identifiable,'effective_scale_information':info,
            'scale_1sigma_from_data':(1/sqrt(info) if info>1e-12 else None),
            'scale_signal_to_prior':sqrt(info)*prior_scale,
            'independent_measurements':int(observation_mask.sum()),
            'rank':int(np.linalg.matrix_rank(A))}


def _estimate(H,observed,nominal,mask,noise_sigma,method):
    h=H[mask].reshape(-1,7)
    y=(observed[mask]-nominal[mask]).reshape(-1)
    # Prior standard deviations in parameter units: m, m/s, dimensionless.
    ps=np.r_[np.full(3,1000.),np.full(3,.7),.20]
    a=h*ps[None,:]/noise_sigma
    b=y/noise_sigma
    weights=np.ones(b.shape[0])
    beta=np.zeros(7)
    for iteration in range(14):
        normal=np.eye(7)+(a.T*weights)@a
        rhs=a.T@(weights*b)
        fact=cho_factor(normal,check_finite=False)
        new_beta=cho_solve(fact,rhs,check_finite=False)
        if method=='least_squares':beta=new_beta;break
        residual=(b-a@new_beta).reshape(-1,3)
        norms=np.linalg.norm(residual,axis=1)
        if method=='huber':
            # Huber on the whitened three-component residual norm.
            w=np.minimum(1.,2.8/np.maximum(norms,1e-12))
        else:
            w=np.where(norms<=sqrt(chi2.ppf(.9973,3)),1.,0.02)
        new_weights=np.repeat(w,3)
        if np.linalg.norm(new_beta-beta)<1e-9 and np.max(np.abs(weights-new_weights))<1e-6:
            beta=new_beta;weights=new_weights;break
        beta=new_beta;weights=new_weights
    normal=np.eye(7)+(a.T*weights)@a
    P=(ps[:,None]*np.linalg.inv(normal))*ps[None,:]
    params=ps*beta
    return params,P,{'downweighted_fixes':int(np.sum(weights.reshape(-1,3)[:,0]<.999)),
                     'effective_fixes':float(np.sum(weights.reshape(-1,3)[:,0]))}


def _measure(c,actual,obs_mask,rng):
    n=len(actual)
    noise=np.zeros((n,3));z=actual[:,:3]*1000
    prev=np.zeros(3)
    for i in range(n):
        prev=c.noise_correlation*prev+sqrt(1-c.noise_correlation**2)*rng.normal(0,c.observation_sigma_m,3)
        noise[i]=prev
    observed=z+noise
    if c.fault_bias_m>0:
        times=np.linspace(0,60*c.duration_minutes,c.samples)
        affected=(times>=c.fault_start_s) & ((times<c.fault_start_s+c.fault_duration_s) if c.fault_duration_s>0 else True)
        observed[affected,0]+=c.fault_bias_m
    return observed


def _wilson(k,n,z=1.96):
    p=k/n;den=1+z*z/n
    return [float((p+z*z/(2*n)-z*sqrt((p*(1-p)+z*z/(4*n))/n))/den),
            float((p+z*z/(2*n)+z*sqrt((p*(1-p)+z*z/(4*n))/n))/den)]


def simulate_observability(c:ObservabilityConfig):
    c.validate()
    times=np.linspace(0,60*c.duration_minutes,c.samples)
    nominal,stm=_propagate(c,times,1.,sensitivities=True)
    initial_delta=np.array([.0002,0.,0.,0.,.00008,0.])
    truth,_=_propagate(c,times,c.true_thrust_scale,initial_delta,sensitivities=False)
    H=_design(c,times,stm)
    # Sampling on numerical output grid; report actual cadence (no implicit clock interpolation).
    mask=np.zeros(len(times),bool)
    last=-np.inf
    for k,t in enumerate(times):
        if t>=c.observation_start_s-1e-9 and t-last>=c.observation_interval_s-1e-9:
            mask[k]=True;last=t
    info=information_analysis(H,mask,c.observation_sigma_m)
    nominal_m=nominal[:,:3]*1000
    rngs=[np.random.default_rng(c.seed+1009*k) for k in range(c.monte_carlo_runs)]
    methods=['least_squares','huber','gate']
    data={m:[] for m in methods}; primary=None
    for index,rng in enumerate(rngs):
        observed=_measure(c,truth,mask,rng)
        for m in methods:
            params,P,meta=_estimate(H,observed,nominal_m,mask,c.observation_sigma_m,m)
            reconstructed=nominal_m+np.einsum('ijk,k->ij',H,params)
            error=np.linalg.norm(reconstructed-truth[:,:3]*1000,axis=1)
            covs=np.einsum('nij,jk,nlk->nil',H,P,H)
            sigma=np.sqrt(np.maximum(0,np.trace(covs,axis1=1,axis2=2)))
            # Marginal Gaussian ellipse consistency, theoretical 95% chi2(3)
            delta=(reconstructed[-1]-truth[-1,:3]*1000)
            end_cov=covs[-1]+np.eye(3)*1e-12
            stat=float(delta@np.linalg.solve(end_cov,delta))
            scale_err=(1+params[6])-c.true_thrust_scale
            rec={'seed':c.seed+1009*index,'final_position_error_m':float(error[-1]),
                 'estimated_thrust_scale':float(1+params[6]),
                 'scale_error_pct_points':float(100*scale_err),
                 'scale_95pct_contains_truth':bool(abs(scale_err)<=1.96*sqrt(P[6,6])),
                 'position_95pct_contains_truth':bool(stat<=chi2.ppf(.95,3)),
                 **meta}
            data[m].append(rec)
            if index==0 and m==c.estimator:
                primary={'parameter_estimate':params.tolist(),
                         'position_error_m':error.tolist(),'position_1sigma_rss_m':sigma.tolist(),
                         'estimated_thrust_scale':rec['estimated_thrust_scale'],
                         'scale_1sigma':float(sqrt(P[6,6])),
                         'downweighted_fixes':meta['downweighted_fixes'],
                         'observations_m':observed[mask].tolist(),
                         'position_error_at_observations_m':error[mask].tolist()}
    summaries={}
    for m,rs in data.items():
        errors=np.array([r['final_position_error_m'] for r in rs])
        scale_errors=np.array([r['scale_error_pct_points'] for r in rs])
        k=sum(r['position_95pct_contains_truth'] for r in rs)
        h=sum(r['scale_95pct_contains_truth'] for r in rs)
        summaries[m]={'runs':len(rs),'median_final_position_error_m':float(np.median(errors)),
             'p90_final_position_error_m':float(np.percentile(errors,90)),
             'worst_final_position_error_m':float(np.max(errors)),
             'thrust_scale_rmse_pct_points':float(np.sqrt(np.mean(scale_errors**2))),
             'position_95pct_coverage':k/len(rs),
             'position_coverage_wilson_95pct':_wilson(k,len(rs)),
             'thrust_95pct_coverage':h/len(rs),
             'mean_downweighted_fixes':float(np.mean([r['downweighted_fixes'] for r in rs]))}
    r=truth[:,:3]
    v=truth[:,3:]
    output={'name':'Observability · finite-burn scale identification and robust batch estimation',
        'version':'1.9.0','reference_frame':c.reference_frame,
        'metrics':{'samples':len(times),'step_s':float(times[1]-times[0]),
                   'conservation_applicable':False,'min_altitude_km':float(np.min(np.linalg.norm(r,axis=1)-R_EARTH)),
                   'max_altitude_km':float(np.max(np.linalg.norm(r,axis=1)-R_EARTH))},
        'orbital_elements':elements_from_state(np.r_[r[-1],v[-1]]).to_dict(),
        'warnings':['This is a noncausal batch calibration fit; it must not be interpreted as an onboard realtime estimator.',
                    'Innovation-based Gaussian uncertainty intervals assume independent sensor noise; correlation and faults invalidate that assumption.'],
        'input':{'altitude_km':c.altitude_km,'burn_start_s':c.burn_start_s,
                 'burn_duration_s':c.burn_duration_s,'fault_bias_m':c.fault_bias_m,
                 'observation_sigma_m':c.observation_sigma_m,'fault_duration_s':c.fault_duration_s,'seed':c.seed},
        'trajectory':{'time_s':times.tolist(),'position_km':r.tolist(),'velocity_km_s':v.tolist(),
            'altitude_km':(np.linalg.norm(r,axis=1)-R_EARTH).tolist(),
            'speed_km_s':np.linalg.norm(v,axis=1).tolist()},
        'observability':{'method':c.estimator,'time_s':times.tolist(),**primary,
           'effective_information':info,'observation_times_s':times[mask].tolist(),
           'actual_thrust_scale':c.true_thrust_scale,'monte_carlo':summaries,
           'assumptions':'Offline seven-parameter locally linearized batch estimation; two-body Earth; fixed inertial thrust axis; synthetic Cartesian measurements; measurement noise treated independent by fitter even when generated correlated. No causal EKF or flight validation.'}}
    return output
