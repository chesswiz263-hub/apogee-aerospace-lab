"""Reproducible v1.0 coupled attitude/translation numerical cross-check.

Compare DOP853 against a separately implemented fixed-step classical RK4 update.
Both solve *the same approximate physical model*; agreement is not empirical validation.
Run: python -m apogee.v10_benchmark
"""
from dataclasses import replace
import json
import numpy as np
from .attitude import AttitudeConfig, simulate_attitude, initial_state, dynamics, q_normalize


def rk4(y,t,dt,config,on):
    f=lambda time,x:dynamics(time,x,config,on)
    k1=f(t,y);k2=f(t+dt/2,y+dt*k1/2);k3=f(t+dt/2,y+dt*k2/2);k4=f(t+dt,y+dt*k3)
    return y+dt*(k1+2*k2+2*k3+k4)/6


def benchmark():
    config=AttitudeConfig(duration_minutes=3,samples=180,burn_start_s=40,
                          burn_duration_s=50,initial_misalignment_deg=25.,
                          j2_enabled=True,gravity_gradient_enabled=True)
    exact=simulate_attitude(config)
    reference=np.r_[exact['trajectory']['position_km'][-1],exact['trajectory']['velocity_km_s'][-1],
                    exact['attitude']['quaternion_BI_wxyz'][-1],exact['attitude']['angular_velocity_body_rad_s'][-1],
                    exact['trajectory']['wet_mass_kg'][-1]]
    out=[]
    for dt in (0.5,0.25):
        y=initial_state(config)
        n=int(round(config.duration_minutes*60/dt))
        for i in range(n):
            t=i*dt
            powered=bool(config.burn_start_s<=t+dt/2<config.burn_start_s+config.burn_duration_s)
            y=rk4(y,t,dt,config,powered)
            y[6:10]=q_normalize(y[6:10])
        error_q=abs(float(np.dot(reference[6:10],y[6:10])))
        out.append({'dt_s':dt,'position_difference_m':float(np.linalg.norm(reference[:3]-y[:3])*1000),
                    'velocity_difference_mm_s':float(np.linalg.norm(reference[3:6]-y[3:6])*1e6),
                    'attitude_difference_deg':float(np.rad2deg(2*np.arccos(np.clip(error_q,-1,1)))),
                    'mass_difference_kg':float(abs(reference[13]-y[13]))})
    default=simulate_attitude(AttitudeConfig())
    uncontrolled=simulate_attitude(replace(AttitudeConfig(),controller_enabled=False))
    return {'version':'1.0.0','integrator_comparison':out,'default_scenario':{
        'initial_pointing_error_deg':default['attitude']['pointing_error_deg'][0],
        'controlled_final_error_deg':default['attitude']['final_pointing_error_deg'],
        'uncontrolled_final_error_deg':uncontrolled['attitude']['final_pointing_error_deg'],
        'controlled_mean_burn_error_deg':default['attitude']['mean_burn_pointing_error_deg'],
        'uncontrolled_mean_burn_error_deg':uncontrolled['attitude']['mean_burn_pointing_error_deg'],
        'controlled_propellant_used_kg':default['metrics']['propellant_used_kg'],
        'ideal_delta_v_m_s':default['metrics']['ideal_thrust_delta_v_m_s']},
        'limitations':['Same approximate dynamics in both integrators; not comparison with flight data',
                       'DOP853 and RK4 share force, attitude and controller equations',
                       'This is not a hardware-in-the-loop, control stability, or flight-certification test']}

if __name__=='__main__': print(json.dumps(benchmark(),indent=2))
