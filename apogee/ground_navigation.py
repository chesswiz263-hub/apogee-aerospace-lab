"""APOGEE v2.0: causal simulated Earth-ground range/range-rate navigation.

Ground frame is *an explicitly idealized Earth-rotating sphere* tied to an
approximate GMST epoch, NOT GCRS, ITRS, TEME, or an operational station model.
Truth is propagated by independent DOP853 from v1.9. The causal EKF uses a
standalone RK4 variational prediction with 7-state (r,v,thrust-scale) error
covariance and Joseph-form measurement updates. Late packets are handled by
replaying all packets RECEIVED at the current time in CAPTURE-time order.
No future packet can enter an estimate early. No external truth is claimed.
"""
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from math import cos, sin, sqrt, radians, asin
import numpy as np
from .observability import ObservabilityConfig, _propagate, _base, G0
from .physics import MU, R_EARTH, elements_from_state

OMEGA=7.2921150e-5   # rad/s, illustrative uniform rotation
FRAME='APOGEE_SIM_ECI_EARTH_ROTATING'

@dataclass(frozen=True)
class GroundNavConfig:
    altitude_km: float=400.
    inclination_deg: float=51.6
    duration_minutes: float=12.
    samples: int=49
    burn_start_s: float=180.
    burn_duration_s: float=120.
    true_thrust_scale: float=1.05
    measurement_interval_s: float=45.
    telemetry_delay_s: float=20.
    range_sigma_m: float=20.
    range_rate_sigma_m_s: float=.04
    min_elevation_deg: float=5.
    station_count: int=24
    initial_position_error_m: float=600.
    initial_velocity_error_m_s: float=.4
    process_acceleration_sigma_m_s2: float=.00015
    fault_bias_m: float=0.
    fault_start_s: float=360.
    monte_carlo_runs: int=3
    seed: int=29
    epoch_utc: str='2026-10-25T12:00:00Z'
    reference_frame: str=FRAME

    def validate(self):
        values=(self.altitude_km,self.inclination_deg,self.duration_minutes,self.burn_start_s,
                self.burn_duration_s,self.true_thrust_scale,self.measurement_interval_s,
                self.telemetry_delay_s,self.range_sigma_m,self.range_rate_sigma_m_s,
                self.min_elevation_deg,self.initial_position_error_m,self.initial_velocity_error_m_s,
                self.process_acceleration_sigma_m_s2,self.fault_bias_m,self.fault_start_s)
        if (not np.all(np.isfinite(values)) or not 200<=self.altitude_km<=2000
            or not 0<=self.inclination_deg<=180 or not 5<=self.duration_minutes<=25
            or not 25<=self.samples<=101 or not 0<=self.burn_start_s<=1500
            or not 0<=self.burn_duration_s<=240 or not .7<=self.true_thrust_scale<=1.3
            or not 15<=self.measurement_interval_s<=120 or not 0<=self.telemetry_delay_s<=180
            or not .1<=self.range_sigma_m<=500 or not .001<=self.range_rate_sigma_m_s<=3
            or not 0<=self.min_elevation_deg<=35 or self.station_count not in (6,12,24)
            or not 0<=self.initial_position_error_m<=10000
            or not 0<=self.initial_velocity_error_m_s<=5
            or not 0<=self.process_acceleration_sigma_m_s2<=.01
            or not 0<=self.fault_bias_m<=3000 or not 0<=self.fault_start_s<=1500
            or not 1<=self.monte_carlo_runs<=12 or not 0<=self.seed<=2147483647
            or self.reference_frame!=FRAME):
            raise ValueError('Invalid v2.0 ground navigation parameters or frame')
        if self.burn_start_s+self.burn_duration_s>self.duration_minutes*60:
            raise ValueError('Burn must end before mission end')
        _epoch(self.epoch_utc)


def _epoch(value):
    if not isinstance(value,str) or not value.endswith('Z'):
        raise ValueError('Epoch must be an absolute UTC ISO-8601 timestamp ending in Z')
    try: dt=datetime.fromisoformat(value[:-1]+'+00:00')
    except ValueError as exc: raise ValueError('Invalid UTC epoch') from exc
    if dt.tzinfo is None or not 2000<=dt.year<=2035:
        raise ValueError('Epoch outside supported approximation 2000–2035')
    return dt.astimezone(timezone.utc)


def _gmst(epoch):
    # Mean sidereal angle polynomial; this is NOT a full IERS UT1 polar motion model.
    dt=_epoch(epoch)
    jd=dt.timestamp()/86400.+2440587.5
    t=(jd-2451545.)/36525.
    deg=280.46061837+360.98564736629*(jd-2451545.)+.000387933*t*t-t*t*t/38710000.
    return radians(deg%360)


def stations(count):
    # Synthetic global grid; elevations are geocentric on a spherical Earth.
    bands={6:[0],12:[-25,25],24:[-60,-20,20,60]}[count]
    return [{'id':f'S{j+1:02}', 'latitude_deg':float(lat),
             'longitude_deg':float(lon), 'altitude_km':0.}
            for j,(lat,lon) in enumerate((lat,lon) for lat in bands for lon in (-150,-90,-30,30,90,150))]


def station_state(s,t,theta0):
    lat=radians(s['latitude_deg']);lon=radians(s['longitude_deg'])+theta0+OMEGA*t
    r=(R_EARTH+s['altitude_km'])*np.array([cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat)])
    v=np.array([-OMEGA*r[1],OMEGA*r[0],0.])
    return r,v


def observable(state,station,t,theta0,min_elevation_deg=5.):
    rs,vs=station_state(station,t,theta0)
    rho=state[:3]-rs;rn=np.linalg.norm(rho)
    u=rho/rn
    e=asin(np.clip(float(u@rs/np.linalg.norm(rs)),-1.,1.))
    return e>=radians(min_elevation_deg),float(np.degrees(e))


def measurement(state,station,t,theta0):
    rs,vs=station_state(station,t,theta0)
    rel=np.asarray(state[:3])-rs;dv=np.asarray(state[3:6])-vs
    rho=np.linalg.norm(rel);u=rel/rho;radial=float(u@dv)
    h=np.array([rho*1000.,radial*1000.])  # meters, m/s
    H=np.zeros((2,7))
    H[0,:3]=1000*u
    H[1,:3]=1000*(dv-u*radial)/rho
    H[1,3:6]=1000*u
    return h,H


def _flow(t,y,c,u,alpha=None):
    # The physical mass flow scales with actual thrust, as in v1.9 truth.
    r=y[:3];v=y[3:6];a=float(y[6] if alpha is None else alpha)
    norm=np.linalg.norm(r);g=-MU*r/norm**3
    firing=c.burn_duration_s>0 and c.burn_start_s<=t<c.burn_start_s+c.burn_duration_s
    elapsed=np.clip(t-c.burn_start_s,0,c.burn_duration_s)
    mass=max(600.,750.-a*850/(315*G0)*elapsed)
    if firing:g=g+a*850/(1000*mass)*u
    return np.r_[v,g,0.]


def _jac(t,y,c,u):
    r=y[:3];rn=np.linalg.norm(r)
    A=np.zeros((7,7));A[:3,3:6]=np.eye(3)
    A[3:6,:3]=-MU*(np.eye(3)/rn**3-3*np.outer(r,r)/rn**5)
    if c.burn_duration_s>0 and c.burn_start_s<=t<c.burn_start_s+c.burn_duration_s:
        alpha=y[6];elapsed=t-c.burn_start_s;flow=850/(315*G0)
        m=750-alpha*flow*elapsed
        A[3:6,6]=850/1000*(1/m+alpha*flow*elapsed/m**2)*u
    return A


def _predict(y,P,a,b,c,u):
    """Independent RK4 state + second-order state transition; process Q per step.

    Split exactly at the maneuver switches. The covariance remains an *approximate*
    EKF covariance, not a calibrated probability distribution.
    """
    if b<=a+1e-10:return y.copy(),P.copy()
    boundaries=sorted(set([a,b]+[k for k in (c.burn_start_s,c.burn_start_s+c.burn_duration_s) if a<k<b]))
    y=y.copy();P=P.copy()
    for left,right in zip(boundaries[:-1],boundaries[1:]):
        n=int(np.ceil((right-left)/5.));h=(right-left)/n
        for j in range(n):
            t=left+j*h
            # Avoid evaluating right at the burn end in an RK stage.
            fire=c.burn_duration_s>0 and c.burn_start_s<=t+1e-8<c.burn_start_s+c.burn_duration_s
            def f(tt,yy):
                # Freeze this segment's firing status at discontinuity edges.
                if fire and tt>=c.burn_start_s+c.burn_duration_s:
                    tt=np.nextafter(c.burn_start_s+c.burn_duration_s,-np.inf)
                elif not fire and tt>=c.burn_start_s and tt<c.burn_start_s+c.burn_duration_s:
                    tt=c.burn_start_s+c.burn_duration_s
                return _flow(tt,yy,c,u)
            k1=f(t,y);k2=f(t+h/2,y+h*k1/2);k3=f(t+h/2,y+h*k2/2);k4=f(t+h,y+h*k3)
            mid=y+h*k2/2;F=_jac(t+h/2,mid,c,u)
            phi=np.eye(7)+h*F+0.5*(h*F)@(h*F)
            s=c.process_acceleration_sigma_m_s2/1000
            q=np.zeros((7,7));q[:3,:3]=(h**3/3)*s*s*np.eye(3)
            q[:3,3:6]=q[3:6,:3]=h*h/2*s*s*np.eye(3)
            q[3:6,3:6]=h*s*s*np.eye(3)
            q[6,6]=h*(2e-5)**2  # assumed thrust scale diffusion
            P=phi@P@phi.T+q;P=(P+P.T)/2
            y=y+h*(k1+2*k2+2*k3+k4)/6
    return y,P


def _update(y,P,packet,c,theta0,stations_by_id):
    h,H=measurement(y,stations_by_id[packet['station_id']],packet['capture_time_s'],theta0)
    z=np.array([packet['range_m'],packet['range_rate_m_s']])
    R=np.diag([c.range_sigma_m**2,c.range_rate_sigma_m_s**2])
    S=H@P@H.T+R
    K=np.linalg.solve(S,H@P).T
    nu=z-h
    y=y+K@nu
    I=np.eye(7);D=I-K@H
    P=D@P@D.T+K@R@K.T;P=(P+P.T)/2
    nis=float(nu@np.linalg.solve(S,nu))
    return y,P,nis


def _observations(c,truth,times,rng):
    theta0=_gmst(c.epoch_utc);network=stations(c.station_count)
    observations=[];availability=[]
    for t,state in zip(times,truth):
        visible=[]
        for s in network:
            yes,elev=observable(state,s,float(t),theta0,c.min_elevation_deg)
            if yes:visible.append((elev,s))
        availability.append({'time_s':float(t),'visible_stations':len(visible)})
        for elev,s in sorted(visible,key=lambda v:-v[0])[:2]:
            vals,_=measurement(state,s,float(t),theta0)
            vals+=rng.normal(0,[c.range_sigma_m,c.range_rate_sigma_m_s])
            if c.fault_bias_m and t>=c.fault_start_s:vals[0]+=c.fault_bias_m
            observations.append({'station_id':s['id'],'capture_time_s':float(t),
                'receipt_time_s':float(t+c.telemetry_delay_s+(int(s['id'][1:])%3)*4),
                'range_m':float(vals[0]),'range_rate_m_s':float(vals[1]),
                'elevation_deg':float(elev)})
    return observations,availability


def _initial_estimate(c):
    base,u=_base(c)
    # Filter never receives the true injected state perturbation.
    y=np.r_[base+np.array([c.initial_position_error_m/1000,0.,0.,0.,c.initial_velocity_error_m_s/1000,0.]),1.]
    P=np.diag([.9**2]*3+[.001**2]*3+[.2**2])
    return y,P,u


def replay_at(c,packets,now,theta0,network=None):
    """Re-estimate using ONLY received packets; replay at capture timestamps.

    This is an expensive fixed-origin replay used for educational correctness,
    not a real-time optimized OOSM/RTS smoother. Future packets are excluded.
    """
    mapping={s['id']:s for s in (network if network is not None else stations(c.station_count))}
    ordered=sorted((p for p in packets if p['receipt_time_s']<=now+1e-9),
                   key=lambda p:(p['capture_time_s'],p['station_id']))
    y,P,u=_initial_estimate(c);last=0.;nis=[]
    for p in ordered:
        if p['capture_time_s']>now+1e-9:raise RuntimeError('Noncausal received packet')
        y,P=_predict(y,P,last,p['capture_time_s'],c,u)
        y,P,score=_update(y,P,p,c,theta0,mapping)
        nis.append(score);last=p['capture_time_s']
    y,P=_predict(y,P,last,now,c,u)
    return y,P,ordered,nis


def _single(c,seed,include_series=True):
    rng=np.random.default_rng(seed)
    t_end=c.duration_minutes*60
    # DOP853 independent truth; observation times are exactly the requested cadence.
    obs_times=np.arange(0,t_end+1e-8,c.measurement_interval_s)
    if obs_times[-1]<t_end:obs_times=np.r_[obs_times,t_end]
    base,u=_base(c)
    truth_delta=np.array([.0002,-.0001,.00012,0,.0001,-.00008])
    baseconfig=ObservabilityConfig(altitude_km=c.altitude_km,inclination_deg=c.inclination_deg,
        duration_minutes=c.duration_minutes,samples=61,burn_start_s=c.burn_start_s,
        burn_duration_s=c.burn_duration_s,monte_carlo_runs=1)
    states,_=_propagate(baseconfig,obs_times,c.true_thrust_scale,initial=truth_delta)
    packets,availability=_observations(c,states,obs_times,rng)
    theta0=_gmst(c.epoch_utc);grid=np.linspace(0,t_end,c.samples)
    true_series,_=_propagate(baseconfig,grid,c.true_thrust_scale,initial=truth_delta)
    err=[];sigma=[];scale=[];used=[];nis=[]
    for t,real in zip(grid,true_series):
        x,P,received,scores=replay_at(c,packets,float(t),theta0)
        err.append(float(1000*np.linalg.norm(x[:3]-real[:3])))
        sigma.append(float(1000*sqrt(max(0.,np.trace(P[:3,:3])))))
        scale.append(float(x[6]));used.append(len(received))
        nis.append(float(np.mean(scores)) if scores else None)
    received=[p for p in packets if p['receipt_time_s']<=t_end+1e-8]
    outcome={'seed':int(seed),'final_error_m':err[-1],'final_1sigma_rss_m':sigma[-1],
        'final_thrust_scale':scale[-1],'packet_count':len(packets),'received_count':len(received),
        'in_flight_count':len(packets)-len(received),
        'visible_intervals':sum(q['visible_stations']>0 for q in availability),
        'mean_nis_received':float(np.mean([v for v in nis if v is not None])) if any(v is not None for v in nis) else None}
    if include_series:outcome.update(
        trajectory={'time_s':grid.tolist(),'position_km':true_series[:,:3].tolist(),
                    'velocity_km_s':true_series[:,3:].tolist()},
        navigation={'position_error_m':err,'position_1sigma_rss_m':sigma,
                    'estimated_thrust_scale':scale,'received_packet_count':used,'mean_nis':nis,
                    'packets':[dict(p) for p in packets], 'station_visibility':availability})
    return outcome


def simulate_ground_navigation(c:GroundNavConfig):
    c.validate()
    single=_single(c,c.seed)
    stats=[{'seed':single['seed'],'final_error_m':single['final_error_m'],
            'final_1sigma_rss_m':single['final_1sigma_rss_m'],
            'received_count':single['received_count']}]
    for i in range(1,c.monte_carlo_runs):
        z=_single(c,c.seed+i,False)
        stats.append({'seed':z['seed'],'final_error_m':z['final_error_m'],
                      'final_1sigma_rss_m':z['final_1sigma_rss_m'],
                      'received_count':z['received_count']})
    errs=[v['final_error_m'] for v in stats]
    traj=single['trajectory']
    state=np.r_[traj['position_km'][0],traj['velocity_km_s'][0]]
    elements=elements_from_state(state)
    radii=np.linalg.norm(np.asarray(traj['position_km']),axis=1)
    speeds=np.linalg.norm(np.asarray(traj['velocity_km_s']),axis=1)
    traj.update(altitude_km=(radii-R_EARTH).tolist(),speed_km_s=speeds.tolist(),
                relative_energy_drift=[0.]*len(radii),relative_angular_momentum_drift=[0.]*len(radii))
    return {'version':'2.0.0','name':'Causal ground tracking and scientific validation',
        'input':vars(c),'trajectory':traj,
        'orbital_elements':{'period_s':elements.period_s,'eccentricity':elements.eccentricity},
        'metrics':{'step_s':(c.duration_minutes*60)/(c.samples-1),'samples':c.samples,
                   'conservation_applicable':False,'max_relative_energy_drift':0.,
                   'max_relative_angular_momentum_drift':0.},
        'ground_navigation':{**single['navigation'],
             'summary':{k:v for k,v in single.items() if k not in ('trajectory','navigation')},
             'stations':stations(c.station_count),
             'monte_carlo':{'runs':len(stats),'median_final_position_error_m':float(np.median(errs)),
                            'p90_final_position_error_m':float(np.percentile(errs,90)),
                            'coverage_below_1sigma_rss_fraction':float(np.mean([s['final_error_m']<=s['final_1sigma_rss_m'] for s in stats])),
                            'seeds':stats},
             'assumptions':['Artificial ground network on a spherical rotating Earth',
                 'Approximate UTC-derived GMST, no UT1/polar motion, refraction, station calibration or media delays',
                 'No external observations or ground truth; independent DOP853 truth versus RK4 EKF',
                 'Position/velocity/thrust-only 7-state covariance; attitude errors omitted',
                 'Replay at receipt uses only delivered packets, sorted by capture epoch']}}


def compare_reference_observations(c:GroundNavConfig,records):
    """Auditable residuals against user-provided, *unverified* observation samples.

    This endpoint deliberately does not convert genuine ITRS/GCRS station data;
    input must explicitly assert the simulator's idealized rotating frame.
    Observations are never silently interpreted as authentic ground tracking.
    """
    c.validate()
    if not isinstance(records,list) or not 1<=len(records)<=200:
        raise ValueError('Provide 1–200 measurement records')
    mapping={s['id']:s for s in stations(c.station_count)}
    times=[];ordered=[]
    for r in records:
        if not isinstance(r,dict) or set(r)!={'capture_time_s','station_id','range_m','range_rate_m_s'}:
            raise ValueError('Each reference record needs time, station_id, range and range_rate only')
        t=r['capture_time_s'];station=r['station_id']
        if (station not in mapping or isinstance(t,bool) or not isinstance(t,(int,float))
            or not np.isfinite(t) or not 0<=t<=c.duration_minutes*60):
            raise ValueError('Invalid station or observation timestamp')
        values=(r['range_m'],r['range_rate_m_s'])
        if any(isinstance(z,bool) or not isinstance(z,(float,int)) or not np.isfinite(z) for z in values):
            raise ValueError('Reference measurements must be finite numbers')
        if r['range_m']<=0:raise ValueError('Range must be positive')
        ordered.append((float(t),str(station),float(values[0]),float(values[1])))
        times.append(float(t))
    # We compare against a DECLARED model, not externally authenticated truth.
    bc=ObservabilityConfig(altitude_km=c.altitude_km,inclination_deg=c.inclination_deg,
        duration_minutes=c.duration_minutes,burn_start_s=c.burn_start_s,
        burn_duration_s=c.burn_duration_s,monte_carlo_runs=1)
    distinct=sorted(set(times))
    state,_=_propagate(bc,np.asarray(distinct),1.,initial=np.zeros(6))
    by_time=dict(zip(distinct,state))
    theta=_gmst(c.epoch_utc)
    residual=[]
    for t,s,measured,rate in ordered:
        expected,_=measurement(by_time[t],mapping[s],t,theta)
        is_visible,elev=observable(by_time[t],mapping[s],t,theta,c.min_elevation_deg)
        residual.append({'capture_time_s':t,'station_id':s,
                         'range_residual_m':measured-float(expected[0]),
                         'range_rate_residual_m_s':rate-float(expected[1]),
                         'modeled_elevation_deg':elev,'model_visible':is_visible})
    ranges=np.array([r['range_residual_m'] for r in residual]);rates=np.array([r['range_rate_residual_m_s'] for r in residual])
    return {'version':'2.0.0','reference_metadata':{'source':'user_supplied_unverified',
               'epoch_utc':c.epoch_utc,'reference_frame':FRAME,'station_geometry':'synthetic spherical stations',
               'model_thrust_scale':1.,'physical_validation':False},
            'count':len(residual),'rms_range_residual_m':float(np.sqrt(np.mean(ranges**2))),
            'rms_range_rate_residual_m_s':float(np.sqrt(np.mean(rates**2))),
            'records':residual}
