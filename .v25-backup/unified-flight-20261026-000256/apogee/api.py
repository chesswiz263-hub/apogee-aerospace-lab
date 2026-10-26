"""Single-origin web server for a reproducible orbital mechanics laboratory."""
from pathlib import Path
from math import ceil, pi, sqrt

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
import os
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, model_validator

from .constants import EARTH_EQUATORIAL_RADIUS_KM as R_EARTH
from .constants import EARTH_MU_KM3_S2 as MU
from .physics import (elements_from_state, hohmann_transfer, integrate_orbit,
                      state_from_elements, trajectory_diagnostics, specific_energy)
from .propulsion import Burn, integrate_mission, rocket_equation_budget
from .perturbations import ForceModel, propagate as propagate_perturbed, diagnostics as perturbation_diagnostics
from .satellites import propagate_omm, fetch_catalog, sgp4_available
from .coupled import integrate_coupled, coast_segment_diagnostics, compare_trajectories
from .targeting import solve_apsis_target
from .frames import spin_transform, transform_teme, astropy_available
from .ephemeris import parse_horizons_vectors, format_reference_vectors

app = FastAPI(title="APOGEE Orbital Physics API", version="2.4.0",
              description="Numerically verified two-body propagation using independent RK4 implementation")
# CORS is for the public GitHub Pages frontend. Restrict to your own site
# for production if you do not want unrelated browser origins accessing the API.
_allowed_origins = [origin.strip() for origin in os.environ.get(
    'APOGEE_ALLOWED_ORIGINS',
    'http://localhost:8000,http://127.0.0.1:8000,http://localhost:5500,http://127.0.0.1:5500'
).split(',') if origin.strip()]
_allowed_regex = os.environ.get('APOGEE_ALLOWED_ORIGIN_REGEX') or None
app.add_middleware(CORSMiddleware, allow_origins=_allowed_origins,
                   allow_origin_regex=_allowed_regex, allow_credentials=False,
                   allow_methods=['GET','POST'], allow_headers=['Content-Type'])
WEB_DIR = Path(__file__).resolve().parents[1] / "web"
app.mount("/static", StaticFiles(directory=WEB_DIR), name="web")


class OrbitRequest(BaseModel):
    perigee_altitude_km: float = Field(default=400, ge=160, le=50000)
    eccentricity: float = Field(default=0.0, ge=0.0, le=0.75)
    inclination_deg: float = Field(default=51.6, ge=0.0, le=180.0)
    raan_deg: float = Field(default=30.0, ge=0.0, le=360.0)
    argument_of_periapsis_deg: float = Field(default=0.0, ge=0.0, le=360.0)
    true_anomaly_deg: float = Field(default=0.0, ge=0.0, le=360.0)
    velocity_factor: float = Field(default=1.0, ge=0.8, le=1.15)
    spacecraft_mass_kg: float = Field(default=750, gt=0, le=1000000)
    duration_orbits: float = Field(default=2.0, ge=0.25, le=8.0)
    samples_per_orbit: int = Field(default=240, ge=90, le=720)


class TransferRequest(BaseModel):
    start_altitude_km: float = Field(default=400.0, ge=160, le=50000)
    target_altitude_km: float = Field(default=700.0, ge=160, le=50000)
    inclination_deg: float = Field(default=28.5, ge=0, le=180)
    samples: int = Field(default=360, ge=90, le=1200)
    dry_mass_kg: float = Field(default=600, gt=0, le=1000000)
    propellant_kg: float = Field(default=150, ge=0, le=1000000)
    specific_impulse_s: float = Field(default=315, ge=50, le=5000)


class BurnRequest(BaseModel):
    start_s: float = Field(default=720, ge=0, le=100000)
    duration_s: float = Field(default=120, gt=0, le=10000)
    thrust_n: float = Field(default=850, gt=0, le=10000000)
    specific_impulse_s: float = Field(default=315, ge=50, le=5000)
    throttle: float = Field(default=1, gt=0, le=1)
    direction: str = Field(default='prograde', pattern='^(prograde|retrograde|radial_out|radial_in|normal|antinormal|transverse|antitransverse)$')
    name: str = Field(default='Burn 1', min_length=1, max_length=40)


class MissionRequest(BaseModel):
    perigee_altitude_km: float = Field(default=400, ge=160, le=50000)
    eccentricity: float = Field(default=0, ge=0, le=.75)
    inclination_deg: float = Field(default=51.6, ge=0, le=180)
    raan_deg: float = Field(default=30, ge=0, le=360)
    argument_of_periapsis_deg: float = Field(default=0, ge=0, le=360)
    true_anomaly_deg: float = Field(default=0, ge=0, le=360)
    dry_mass_kg: float = Field(default=600, gt=0, le=1000000)
    propellant_kg: float = Field(default=150, ge=0, le=1000000)
    duration_minutes: float = Field(default=120, ge=5, le=1440)
    samples: int = Field(default=600, ge=120, le=2400)
    burns: list[BurnRequest] = Field(default_factory=lambda: [BurnRequest()], max_length=12)
    j2_enabled: bool = True
    drag_enabled: bool = False
    cross_section_m2: float = Field(default=5.0, ge=0, le=10000)
    drag_coefficient: float = Field(default=2.2, gt=0, le=5)
    density_at_400_km_kg_m3: float = Field(default=3.725e-12, ge=0, le=1e-8)
    scale_height_km: float = Field(default=58.515, gt=1, le=200)


    @model_validator(mode='after')
    def validate_schedule(self):
        limit = self.duration_minutes * 60
        scheduled = sorted(self.burns, key=lambda b: b.start_s)
        for i, burn in enumerate(scheduled):
            if burn.start_s + burn.duration_s > limit + 1e-9:
                raise ValueError('Every burn must finish before mission end')
            if i and burn.start_s < scheduled[i-1].start_s + scheduled[i-1].duration_s - 1e-9:
                raise ValueError('Burns must not overlap')
        return self


def _mission_coast_diagnostics(states, times, burns, dry_mass_kg):
    """Only evaluate invariants on pure-coast intervals; exclude powered spans.

    A segment in which fuel exhausts is conservatively excluded in full, and
    the coast segment baseline is reset at the next interval boundary.
    """
    energies = np.array([specific_energy(s[:6]) for s in states])
    h = np.linalg.norm(np.cross(states[:, :3], states[:, 3:6]), axis=1)
    e_error, h_error = np.zeros(len(times)), np.zeros(len(times))
    coast_valid = np.zeros(len(times),dtype=bool)
    baseline = None
    for i in range(1,len(times)):
        mid = (times[i-1] + times[i])/2
        powered = any(b.start_s <= mid < b.end_s and states[i-1, 6] > dry_mass_kg + 1e-10
                      for b in burns)
        if powered:
            baseline = None
            continue
        coast_valid[i] = True
        if baseline is None:
            baseline = i-1
        e_error[i] = abs(energies[i]-energies[baseline]) / max(abs(energies[baseline]),1e-12)
        h_error[i] = abs(h[i]-h[baseline]) / max(h[baseline],1e-12)
    return e_error,h_error,coast_valid


def _serialize_mission(outcome, initial_state, dry_mass_kg, inputs, burns):
    states = outcome.states
    r = np.linalg.norm(states[:, :3],axis=1)
    v = np.linalg.norm(states[:,3:6],axis=1)
    if hasattr(outcome,'force_model'):
        diag = coast_segment_diagnostics(outcome,burns,dry_mass_kg)
        e_error = np.asarray(diag['relative_energy_drift'],dtype=float)
        h_error = np.asarray(diag['relative_angular_momentum_drift'],dtype=float)
        # A valid flag controls chart segments; drag has no conservation segments.
        coast_valid = np.asarray(diag['coast_diagnostics_valid'],dtype=bool) if diag['conservation_applicable'] else np.zeros(len(outcome.time_s),dtype=bool)
    else:
        e_error, h_error, coast_valid = _mission_coast_diagnostics(states,outcome.time_s,burns,dry_mass_kg)
        diag = {'max_relative_energy_drift':float(e_error.max()),
                'max_relative_angular_momentum_drift':float(h_error.max()),
                'conservation_applicable':True,'diagnostic_scope':'coasting arcs only'}
    initial_propellant = initial_state[6]-dry_mass_kg
    final_propellant = states[-1,6]-dry_mass_kg
    reports = outcome.burn_reports
    fuel_used = initial_propellant - final_propellant
    return {
        'name': 'Finite-thrust spacecraft mission',
        'input': inputs,
        'spacecraft_mass_kg': float(initial_state[6]),
        'initial_state': {'r_km':initial_state[:3].tolist(), 'v_km_s':initial_state[3:6].tolist()},
        'orbital_elements': elements_from_state(initial_state[:6]).to_dict(),
        'final_orbital_elements': elements_from_state(states[-1,:6]).to_dict(),
        'model': {
            'name': 'Earth point mass + optional J2, drag and continuously steered finite propulsion',
            'reference_frame': 'Earth-centered inertial Cartesian XYZ (idealized axes)',
            'integrator': 'classical fixed-step RK4 with internal substeps and exact burn boundaries',
            'central_body_mu_km3_s2':MU, 'central_body_radius_km':R_EARTH,
            'step_s':outcome.nominal_output_step_s,
            'impact_detected_at_sample':outcome.impact,
            'force_terms':['point-mass Newtonian gravity'] +
                (['Earth J2 geopotential'] if inputs.get('j2_enabled') else []) +
                (['co-rotating exponential-atmosphere drag'] if inputs.get('drag_enabled') else []) +
                ['thrust / instantaneous spacecraft mass'],
            'assumptions':['constant thrust and specific impulse within each scheduled burn',
                           'instantaneously tracked local direction',
                           'optional J2, simplified rotating-atmosphere drag; no third bodies or attitude dynamics',
                           'sampled impact detection only']
        },
        'metrics': {
            'elapsed_s':float(outcome.time_s[-1]), 'requested_duration_s':outcome.requested_duration_s,
            'step_s':outcome.nominal_output_step_s,'samples':len(states),
            'min_altitude_km':float(np.min(r)-R_EARTH),
            'max_altitude_km':float(np.max(r)-R_EARTH),
            'max_relative_energy_drift':diag['max_relative_energy_drift'],
            'max_relative_angular_momentum_drift':diag['max_relative_angular_momentum_drift'],
            'conservation_applicable':diag['conservation_applicable'],
            'diagnostic_scope':diag['diagnostic_scope'],
            'numerical_integrity_scope':'coasting arcs only',
            'dry_mass_kg':float(dry_mass_kg),
            'initial_propellant_kg':float(initial_propellant),
            'propellant_used_kg':float(fuel_used),
            'propellant_remaining_kg':float(final_propellant),
            'ideal_thrust_delta_v_m_s':float(sum(b['delivered_ideal_delta_v_m_s'] for b in reports)),
            'fuel_exhausted':bool(any(b['propellant_depleted'] for b in reports)),
        },
        'trajectory': {
            'time_s':outcome.time_s.tolist(), 'position_km':states[:,:3].tolist(),
            'velocity_km_s':states[:,3:6].tolist(),'altitude_km':(r-R_EARTH).tolist(),
            'speed_km_s':v.tolist(),'wet_mass_kg':states[:,6].tolist(),
            'propellant_remaining_kg':(states[:,6]-dry_mass_kg).tolist(),
            'thrust_active':outcome.thrust_active.tolist(),
            'cumulative_ideal_delta_v_m_s':outcome.cumulative_ideal_delta_v_m_s.tolist(),
            'coast_diagnostics_valid':coast_valid.tolist(),
            'relative_energy_drift':e_error.tolist(),
            'relative_angular_momentum_drift':h_error.tolist(),
        },
        'burn_reports':reports, 'events':outcome.events,
        'warnings':(['Nominal Earth impact sampled; event time not refined.'] if outcome.impact else [])
                     + (['Propellant exhausted before the end of a scheduled burn.'] if any(b['propellant_depleted'] for b in reports) else [])
                     + (['Atmospheric drag uses an illustrative exponential density profile; NOT an operational forecast.'] if inputs.get('drag_enabled') else []),
    }


def _serialize_trajectory(result, initial_state, mass_kg, name, extra=None):
    states = result.states
    positions = states[:, :3]
    velocities = states[:, 3:]
    radii = np.linalg.norm(positions, axis=1)
    speeds = np.linalg.norm(velocities, axis=1)
    diag = trajectory_diagnostics(states)
    elements = elements_from_state(initial_state)
    payload = {
        "model": {
            "name": "Earth-centered Newtonian two-body",
            "reference_frame": "Earth-centered inertial Cartesian XYZ (idealized axes)",
            "integrator": "classical fixed-step Runge–Kutta 4 (RK4)",
            "central_body_mu_km3_s2": MU,
            "central_body_radius_km": R_EARTH,
            "step_s": result.step_s,
            "impact_detected_at_sample": result.impact,
            "force_terms": ["point-mass Newtonian gravity"],
            "assumptions": ["spherical central gravity", "Earth at the inertial origin",
                            "no atmospheric drag, J2, third-body gravity, or thrust",
                            "spacecraft treated as a test particle"],
        },
        "name": name,
        "spacecraft_mass_kg": mass_kg,
        "initial_state": {
            "r_km": np.asarray(initial_state[:3]).tolist(),
            "v_km_s": np.asarray(initial_state[3:]).tolist(),
        },
        "orbital_elements": elements.to_dict(),
        "metrics": {
            "elapsed_s": float(result.time_s[-1]),
            "requested_duration_s": float(result.requested_duration_s),
            "step_s": result.step_s,
            "samples": int(len(result.time_s)),
            "min_altitude_km": float(np.min(radii) - R_EARTH),
            "max_altitude_km": float(np.max(radii) - R_EARTH),
            "max_relative_energy_drift": diag["max_relative_energy_drift"],
            "max_relative_angular_momentum_drift": diag["max_relative_angular_momentum_drift"],
            "specific_energy_km2_s2": diag["specific_energy_km2_s2"],
            "specific_angular_momentum_km2_s": diag["specific_angular_momentum_km2_s"],
        },
        "trajectory": {
            "time_s": result.time_s.tolist(),
            "position_km": positions.tolist(),
            "velocity_km_s": velocities.tolist(),
            "altitude_km": (radii - R_EARTH).tolist(),
            "speed_km_s": speeds.tolist(),
            "relative_energy_drift": diag["relative_energy_drift"],
            "relative_angular_momentum_drift": diag["relative_angular_momentum_drift"],
        },
        "warnings": []
    }
    if result.impact:
        payload["warnings"].append("Impact sampled: trajectory stopped at the first state at or below the Earth's nominal radius; impact time is not event-refined.")
    if elements.period_s is None:
        payload["warnings"].append("Initial orbit is unbound. Duration was defined using the unmodified reference ellipse, not an actual orbital period.")
    if extra:
        payload.update(extra)
    return payload


@app.get("/")
def index():
    return FileResponse(WEB_DIR / "index.html")


@app.get("/api/health")
def health():
    return {"status": "ok", "application": "APOGEE", "version": "2.4.0", "sgp4_available":sgp4_available(), "astropy_available":astropy_available()}


@app.get("/api/constants")
def constants():
    return {"earth_mu_km3_s2": MU, "earth_equatorial_radius_km": R_EARTH,
            "gravity": "point mass, optional J2 and drag", "units": "km / s / kg", "frame": "ECI; SGP4 uses TEME"}


@app.get("/api/presets")
def presets():
    return {
        "Low Earth orbit": {"perigee_altitude_km": 400, "eccentricity": 0.0, "inclination_deg": 51.6,
                            "raan_deg": 30, "argument_of_periapsis_deg": 0, "true_anomaly_deg": 0},
        "Elliptical orbit": {"perigee_altitude_km": 500, "eccentricity": 0.40, "inclination_deg": 35,
                              "raan_deg": 45, "argument_of_periapsis_deg": 20, "true_anomaly_deg": 0},
        "Sun-synchronous-like": {"perigee_altitude_km": 700, "eccentricity": 0.0, "inclination_deg": 98.2,
                                   "raan_deg": 20, "argument_of_periapsis_deg": 0, "true_anomaly_deg": 0},
        "Geostationary-height": {"perigee_altitude_km": 35786, "eccentricity": 0.0, "inclination_deg": 0,
                                 "raan_deg": 0, "argument_of_periapsis_deg": 0, "true_anomaly_deg": 0}
    }


@app.post("/api/propagate")
def propagate(req: OrbitRequest):
    radius_perigee = R_EARTH + req.perigee_altitude_km
    a = radius_perigee / (1 - req.eccentricity)
    unmodified = state_from_elements(a, req.eccentricity, req.inclination_deg, req.raan_deg,
                                     req.argument_of_periapsis_deg, req.true_anomaly_deg)
    original_period = 2 * pi * sqrt(a**3 / MU)
    state = unmodified.copy()
    state[3:] *= req.velocity_factor
    n = ceil(req.duration_orbits * req.samples_per_orbit)
    outcome = integrate_orbit(state, req.duration_orbits * original_period, n)
    payload = _serialize_trajectory(outcome, state, req.spacecraft_mass_kg, "Custom Earth orbit")
    payload["reference_period_s"] = original_period
    payload["input"] = req.model_dump()
    if req.velocity_factor != 1.0:
        payload["warnings"].append("Changing the velocity factor changes the osculating orbit. The requested duration still uses the original unmodified orbital period.")
    return payload


@app.post("/api/transfer")
def transfer(req: TransferRequest):
    """Preview: physical RK4 transfer arc + independently derived analytic impulses."""
    r1 = R_EARTH + req.start_altitude_km
    r2 = R_EARTH + req.target_altitude_km
    plan = hohmann_transfer(r1, r2)
    state = state_from_elements(r1, 0, req.inclination_deg)
    state[3:] *= (1 + plan["departure_delta_v_km_s"] / plan["departure_circular_speed_km_s"])
    outcome = integrate_orbit(state, plan["time_of_flight_s"], req.samples)
    arrival_radius_km = float(np.linalg.norm(outcome.states[-1, :3]))
    plan["arrival_radius_error_km"] = arrival_radius_km - r2
    plan["start_altitude_km"] = req.start_altitude_km
    plan["target_altitude_km"] = req.target_altitude_km
    budget = rocket_equation_budget([abs(plan['departure_delta_v_km_s'])*1000,
                                     abs(plan['arrival_delta_v_km_s'])*1000],
                                    req.dry_mass_kg, req.propellant_kg, req.specific_impulse_s)
    plan['fuel_budget'] = budget
    # Arrival impulse is applied to the propagated velocity to check insertion.
    final_state = outcome.states[-1].copy()
    final_state[3:] += plan['arrival_delta_v_km_s'] * (final_state[3:] / np.linalg.norm(final_state[3:]))
    inserted = elements_from_state(final_state)
    plan['post_circularization_eccentricity'] = inserted.eccentricity
    plan['post_circularization_altitude_km'] = np.linalg.norm(final_state[:3])-R_EARTH
    plan['maneuvers'] = [
        {'type':'impulse','name':'Departure burn','time_s':0.0,
         'delta_v_m_s':plan['departure_delta_v_km_s']*1000,
         'position_km':state[:3].tolist()},
        {'type':'impulse','name':'Circularization burn','time_s':plan['time_of_flight_s'],
         'delta_v_m_s':plan['arrival_delta_v_km_s']*1000,
         'position_km':outcome.states[-1,:3].tolist()}
    ]
    payload = _serialize_trajectory(outcome, state, req.dry_mass_kg+req.propellant_kg,
                                    "Hohmann transfer with propellant budget",
                                    {"transfer": plan, "input": req.model_dump(), 'events':plan['maneuvers']})
    payload["warnings"].append("Two ideal instantaneous tangential burns; no finite-burn gravity loss, plane change or phasing constraints. Transfer trajectory ends just before the arrival impulse.")
    if not budget['feasible']:
        payload['warnings'].append('INSUFFICIENT FUEL: this ideal transfer cannot be completed using the specified propellant reserve and Isp.')
    return payload



@app.post('/api/mission')
def mission(req: MissionRequest):
    perigee = R_EARTH + req.perigee_altitude_km
    state = state_from_elements(perigee/(1-req.eccentricity),req.eccentricity,
                                req.inclination_deg,req.raan_deg,
                                req.argument_of_periapsis_deg,req.true_anomaly_deg)
    initial = np.r_[state, req.dry_mass_kg+req.propellant_kg]
    burns = [Burn(b.start_s,b.duration_s,b.thrust_n,b.specific_impulse_s,
                  b.direction,b.throttle,b.name) for b in req.burns]
    force = _mission_force(req)
    result = integrate_coupled(initial,req.dry_mass_kg,req.duration_minutes*60,req.samples,burns,force)
    return _serialize_mission(result,initial,req.dry_mass_kg,req.model_dump(),burns)


def _mission_force(req: MissionRequest) -> ForceModel:
    return ForceModel(j2=req.j2_enabled, drag=req.drag_enabled,
                      mass_kg=req.dry_mass_kg+req.propellant_kg,
                      area_m2=req.cross_section_m2, cd=req.drag_coefficient,
                      density_at_400_km_kg_m3=req.density_at_400_km_kg_m3,
                      scale_height_km=req.scale_height_km)


@app.post('/api/compare')
def compare(req: MissionRequest):
    """Same initial state, burn schedule and time grid; vary force model only."""
    r0 = R_EARTH + req.perigee_altitude_km
    cart = state_from_elements(r0/(1-req.eccentricity),req.eccentricity,
                               req.inclination_deg,req.raan_deg,
                               req.argument_of_periapsis_deg,req.true_anomaly_deg)
    initial=np.r_[cart,req.dry_mass_kg+req.propellant_kg]
    burns=[Burn(b.start_s,b.duration_s,b.thrust_n,b.specific_impulse_s,
                b.direction,b.throttle,b.name) for b in req.burns]
    base_force = _mission_force(req)
    from dataclasses import replace
    baseline=integrate_coupled(initial,req.dry_mass_kg,req.duration_minutes*60,
                               req.samples,burns,replace(base_force,j2=False,drag=False))
    detailed=integrate_coupled(initial,req.dry_mass_kg,req.duration_minutes*60,
                               req.samples,burns,base_force)
    payload=_serialize_mission(detailed,initial,req.dry_mass_kg,req.model_dump(),burns)
    payload['name']='Force-model trajectory comparison'
    payload['comparison']=compare_trajectories(baseline,detailed)
    payload['comparison']['reference_model']='Earth point-mass with identical engine burns'
    payload['comparison']['target_model']='Earth point-mass, J2='+str(base_force.j2)+', drag='+str(base_force.drag)
    payload['comparison']['baseline_trajectory']={
        'time_s':baseline.time_s.tolist(),
        'position_km':baseline.states[:,:3].tolist(),
        'velocity_km_s':baseline.states[:,3:6].tolist()}
    payload['comparison']['baseline_final_fuel_kg']=float(baseline.states[-1,6]-req.dry_mass_kg)
    payload['comparison']['target_final_fuel_kg']=float(detailed.states[-1,6]-req.dry_mass_kg)
    return payload


class PerturbationRequest(BaseModel):
    perigee_altitude_km: float = Field(default=400, ge=180, le=50000)
    eccentricity: float = Field(default=0.0, ge=0, le=.65)
    inclination_deg: float = Field(default=51.6, ge=0, le=180)
    raan_deg: float = Field(default=30, ge=0, le=360)
    argument_of_periapsis_deg: float = Field(default=0, ge=0, le=360)
    true_anomaly_deg: float = Field(default=0, ge=0, le=360)
    duration_hours: float = Field(default=18, gt=0, le=168)
    samples: int = Field(default=720, ge=100, le=2400)
    j2_enabled: bool = True
    drag_enabled: bool = False
    mass_kg: float = Field(default=750, gt=0, le=1000000)
    cross_section_m2: float = Field(default=5, ge=0, le=10000)
    drag_coefficient: float = Field(default=2.2, gt=0, le=5)
    density_at_400_km_kg_m3: float = Field(default=3.725e-12, ge=0, le=1e-8)
    scale_height_km: float = Field(default=58.515, gt=1, le=200)


@app.post('/api/perturbations')
def perturbations(req: PerturbationRequest):
    r_perigee=R_EARTH+req.perigee_altitude_km
    state=state_from_elements(r_perigee/(1-req.eccentricity),req.eccentricity,
              req.inclination_deg,req.raan_deg,req.argument_of_periapsis_deg,req.true_anomaly_deg)
    model=ForceModel(j2=req.j2_enabled,drag=req.drag_enabled,
                     mass_kg=req.mass_kg,area_m2=req.cross_section_m2,
                     cd=req.drag_coefficient,density_at_400_km_kg_m3=req.density_at_400_km_kg_m3,
                     scale_height_km=req.scale_height_km)
    result=propagate_perturbed(state,req.duration_hours*3600,req.samples,model)
    states=result.states
    radii=np.linalg.norm(states[:,:3],axis=1)
    speeds=np.linalg.norm(states[:,3:],axis=1)
    diag=perturbation_diagnostics(states,model)
    el0=elements_from_state(state)
    elf=elements_from_state(states[-1])
    return {
        'name':'Earth perturbation dynamics','input':req.model_dump(),
        'model':{'name':'Earth point-mass + J2 + co-rotating exponential drag',
                 'reference_frame':'Idealized Earth-centered inertial; z-axis aligned with Earth spin',
                 'integrator':'RK4 with internal steps <= 25 s',
                 'force_terms':['point-mass gravity']+(['Earth J2 geopotential'] if model.j2 else [])+
                     (['co-rotating exponential-atmosphere drag'] if model.drag else []),
                 'assumptions':['Earth spherical for impact and altitude; J2 axis fixed in inertial coordinates',
                       'single exponential density at 400 km; not driven by solar/geomagnetic conditions',
                       'fixed area, CD, mass, no lift or thrust', 'impact sampled at substep boundary'],
                 'step_s':result.step_s,'impact_detected_at_sample':result.impact},
        'spacecraft_mass_kg':req.mass_kg,
        'orbital_elements':el0.to_dict(),'final_orbital_elements':elf.to_dict(),
        'initial_state':{'r_km':state[:3].tolist(),'v_km_s':state[3:].tolist()},
        'metrics':{'samples':len(states),'step_s':result.step_s,'elapsed_s':float(result.time_s[-1]),
                   'requested_duration_s':result.requested_duration_s,
                   'min_altitude_km':float(np.min(radii)-R_EARTH),
                   'max_altitude_km':float(np.max(radii)-R_EARTH),
                   'max_relative_energy_drift':diag['max_relative_energy_drift'],
                   'max_relative_angular_momentum_drift':diag['max_relative_angular_momentum_drift'],
                   'diagnostic_scope':diag['diagnostic_scope'],
                   'conservation_applicable':diag['conservation_applicable'],
                   'relative_specific_energy_change':diag['relative_specific_energy_change'],
                   'relative_angular_momentum_change':diag['relative_angular_momentum_change'],
                   'delta_raan_deg':float((elf.raan_deg-el0.raan_deg+180)%360-180),
                   'delta_semi_major_axis_km':float(elf.semi_major_axis_km-el0.semi_major_axis_km)},
        'trajectory':{'time_s':result.time_s.tolist(),'position_km':states[:,:3].tolist(),
                      'velocity_km_s':states[:,3:].tolist(), 'altitude_km':(radii-R_EARTH).tolist(),
                      'speed_km_s':speeds.tolist(),
                      'relative_energy_drift':diag['relative_energy_drift'],
                      'relative_angular_momentum_drift':diag['relative_angular_momentum_drift']},
        'warnings':(['Atmospheric density is a toy exponential approximation: not appropriate for operational lifetime prediction.'] if model.drag else [])+
                     (['Impact detected only at a numerical substep.'] if result.impact else [])
    }


class SatelliteRequest(BaseModel):
    omm: dict
    duration_minutes: int = Field(default=180, ge=1, le=1440)
    samples: int = Field(default=360, ge=10, le=1000)


@app.post('/api/satellite/propagate')
def satellite_propagate(req: SatelliteRequest):
    try:
        return propagate_omm(req.omm,req.duration_minutes,req.samples)
    except RuntimeError as exc:
        raise HTTPException(status_code=503,detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


@app.get('/api/catalog/{catalog_number}')
def catalog_lookup(catalog_number: int):
    try:
        return {'data':fetch_catalog(catalog_number),'source':'CelesTrak GP OMM JSON',
                'sgp4_available':sgp4_available()}
    except ValueError as exc:
        raise HTTPException(status_code=502,detail=str(exc)) from exc


# v0.5: targeted impulsive maneuvers, frame conversion, reference vectors.
class TargetRequest(BaseModel):
    start_altitude_km: float = Field(default=400, ge=160, le=50000)
    target_altitude_km: float = Field(default=700, ge=160, le=50000)
    inclination_deg: float = Field(default=28.5, ge=0, le=180)
    dry_mass_kg: float = Field(default=600, gt=0, le=1000000)
    propellant_kg: float = Field(default=150, ge=0, le=1000000)
    specific_impulse_s: float = Field(default=315, ge=50, le=5000)
    samples: int = Field(default=500, ge=120, le=2400)


@app.post('/api/target')
def target(req: TargetRequest):
    try:
        solved=solve_apsis_target(**req.model_dump())
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    outcome=solved.pop('outcome')
    state=solved.pop('departure_state')
    events=solved['maneuvers']
    payload=_serialize_trajectory(outcome,state,req.dry_mass_kg+req.propellant_kg,
                                   'Numerical impulsive apsis targeting',
                                   {'targeting':solved,'events':events,'input':req.model_dump()})
    payload['model']['name']='Two-body Earth orbit with numerically targeted impulse at departure'
    payload['model']['assumptions']+=['ideal instantaneous impulse at a circular starting orbit',
       'circularization impulse at the opposite apsis; no J2, drag or finite burn']
    payload['warnings']+=['Impulsive two-body targeting only; fixed circular start, tangential maneuver and half-transfer period.',
       'Bisection residual describes osculating apsis targeting; separate propagated arrival radius error describes RK4 precision.']
    if not solved['fuel_budget']['feasible']:
        payload['warnings'].append('INSUFFICIENT PROPELLANT for the ideal two-impulse target.')
    return payload


class SpinFrameRequest(BaseModel):
    state: list[float] = Field(min_length=6,max_length=6)
    greenwich_angle_deg_at_t0: float = Field(default=0,ge=-360,le=360)
    elapsed_s: float = Field(default=0,ge=-1e8,le=1e8)
    direction: str = Field(default='eci_to_ecef',pattern='^(eci_to_ecef|ecef_to_eci)$')

@app.post('/api/frames/spin')
def spin_frame(req: SpinFrameRequest):
    try:
        converted=spin_transform(req.state,req.greenwich_angle_deg_at_t0,req.elapsed_s,req.direction)
    except ValueError as exc:raise HTTPException(status_code=422,detail=str(exc)) from exc
    return {'state':converted.tolist(),'direction':req.direction,
            'frame_model':'uniformly rotating Earth, user-defined angle; not full ITRF/IERS',
            'units':'km and km/s',
            'warnings':['This is a uniform-spin ECI/ECEF approximation with explicit zero-angle epoch; not a TEME, GCRS, or ITRS transformation.']}


class TemeFrameRequest(BaseModel):
    state_teme: list[float] = Field(min_length=6,max_length=6)
    epoch_utc: str = Field(min_length=20,max_length=40)
    destination: str = Field(default='itrs',pattern='^(itrs|gcrs)$')

@app.post('/api/frames/teme')
def teme_frame(req: TemeFrameRequest):
    try:
        transformed=transform_teme(req.state_teme,req.epoch_utc,req.destination)
    except RuntimeError as exc:raise HTTPException(status_code=503,detail=str(exc)) from exc
    except (ValueError,TypeError) as exc:raise HTTPException(status_code=422,detail=str(exc)) from exc
    return {'state':transformed.tolist(),'destination_frame':req.destination.upper(),
            'epoch_utc':req.epoch_utc,
            'warnings':['Accuracy depends on available Earth-orientation data (IERS); auto-download is disabled.',
                        'GCRS is geocentric relativistic reference coordinates, not a synonym for Earth-centered ICRF.']}


class HorizonsImportRequest(BaseModel):
    text: str = Field(min_length=32,max_length=260000)
    metadata: dict[str,str]

@app.post('/api/ephemeris/import')
def import_ephemeris(req: HorizonsImportRequest):
    try:
        parsed=parse_horizons_vectors(req.text,req.metadata)
        return format_reference_vectors(parsed)
    except ValueError as exc:raise HTTPException(status_code=422,detail=str(exc)) from exc

# v0.6: numerically event-targeted, fuel-ranked ideal impulses and aligned
# comparisons against user-attested Earth-centered ICRF vectors.
from .optimization import optimize_impulsive_transfer
from .reference_check import compare_imported_reference

class OptimizationRequest(BaseModel):
    start_altitude_km: float = Field(default=400,ge=180,le=20000)
    target_altitude_km: float = Field(default=700,ge=180,le=20000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    dry_mass_kg: float = Field(default=600,gt=0,le=1000000)
    propellant_kg: float = Field(default=150,ge=0,le=1000000)
    specific_impulse_s: float = Field(default=315,ge=50,le=5000)
    phase_candidates: int = Field(default=8)
    j2_enabled: bool = True
    samples: int = Field(default=360,ge=120,le=1000)

@app.post('/api/optimize')
def optimize(req: OptimizationRequest):
    try:
        return optimize_impulsive_transfer(**req.model_dump())
    except ValueError as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

@app.post('/api/reference/compare')
def reference_compare(req: HorizonsImportRequest):
    try:
        return compare_imported_reference(req.text,req.metadata)
    except ValueError as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

# v0.7: finite-duration two-burn engine targeting and uncertainty propagation.
from .finite_optimization import optimize_finite_transfer
from .uncertainty import propagate_uncertainty

class FiniteOptimizationRequest(BaseModel):
    start_altitude_km: float = Field(default=400,ge=200,le=3000)
    target_altitude_km: float = Field(default=700,ge=200,le=3000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    dry_mass_kg: float = Field(default=600,gt=0,le=100000)
    propellant_kg: float = Field(default=150,gt=0,le=100000)
    specific_impulse_s: float = Field(default=315,ge=50,le=5000)
    thrust_n: float = Field(default=850,ge=10,le=1000000)
    phase_candidates: int = Field(default=4)
    j2_enabled: bool = False
    samples: int = Field(default=280,ge=120,le=800)

@app.post('/api/finite/optimize')
def finite_optimize(req: FiniteOptimizationRequest):
    try:
        return optimize_finite_transfer(**req.model_dump())
    except (ValueError,RuntimeError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

class UncertaintyRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=5000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=90,ge=1,le=480)
    position_sigma_m: float = Field(default=100,gt=0,le=10000)
    velocity_sigma_mm_s: float = Field(default=10,gt=0,le=1000)
    monte_carlo_samples: int = Field(default=64)
    seed: int = Field(default=42,ge=0,le=2147483647)
    j2_enabled: bool = False
    steps: int = Field(default=120)

@app.post('/api/uncertainty')
def uncertainty(req: UncertaintyRequest):
    try:
        return propagate_uncertainty(**req.model_dump())
    except (ValueError,RuntimeError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v0.8: position-only synthetic EKF navigation and bounded ideal impulse feedback.
from .guidance import simulate_guidance

class GuidanceRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=2000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=90,ge=20,le=180)
    measurement_interval_s: int = Field(default=90)
    measurement_sigma_m: float = Field(default=30,gt=0,le=1000)
    initial_position_error_m: float = Field(default=600,ge=0,le=5000)
    initial_velocity_error_m_s: float = Field(default=.4,ge=0,le=5)
    max_total_delta_v_m_s: float = Field(default=2.,ge=0,le=30)
    guidance_enabled: bool = True
    corrections: int = Field(default=3)
    seed: int = Field(default=42,ge=0,le=2147483647)
    j2_enabled: bool = False

@app.post('/api/guidance')
def guidance(req: GuidanceRequest):
    try:
        return simulate_guidance(**req.model_dump())
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

# v0.9: range/range-rate sensor geometry and finite-duration feedback burns.
from .robust_guidance import simulate_robust_guidance, robustness_sweep

class RobustGuidanceRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=2000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=90,ge=30,le=180)
    measurement_interval_s: int = Field(default=90)
    range_sigma_m: float = Field(default=15,ge=.01,le=1000)
    range_rate_sigma_m_s: float = Field(default=.03,ge=.001,le=5)
    initial_position_error_m: float = Field(default=600,ge=0,le=5000)
    initial_velocity_error_m_s: float = Field(default=.4,ge=0,le=5)
    dry_mass_kg: float = Field(default=600,ge=50,le=100000)
    fuel_mass_kg: float = Field(default=150,ge=0,le=50000)
    thrust_n: float = Field(default=850,ge=100,le=100000)
    specific_impulse_s: float = Field(default=315,ge=50,le=5000)
    max_total_delta_v_m_s: float = Field(default=2,ge=0,le=15)
    corrections: int = Field(default=3,ge=0,le=4)
    guidance_enabled: bool = True
    truth_j2_enabled: bool = True
    filter_j2_enabled: bool = True
    process_acceleration_sigma_mm_s2: float = Field(default=.001,ge=0,le=1)
    thrust_error_pct: float = Field(default=2,ge=0,le=20)
    pointing_error_deg: float = Field(default=.25,ge=0,le=10)
    station_count: int = Field(default=18)
    min_elevation_deg: float = Field(default=5,ge=0,le=20)
    seed: int = Field(default=42,ge=0,le=2147483647)

@app.post('/api/robust/guidance')
def robust_guidance(req: RobustGuidanceRequest):
    try:
        return simulate_robust_guidance(**req.model_dump())
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

class RobustSweepRequest(RobustGuidanceRequest):
    seeds: int = Field(default=5,ge=2,le=10)

@app.post('/api/robust/sweep')
def robust_sweep(req: RobustSweepRequest):
    try:
        data=req.model_dump()
        seeds=data.pop('seeds')
        return robustness_sweep(seeds=seeds,**data)
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

# v1.0: body-fixed propulsion coupled to rigid-body attitude dynamics.
from .attitude import AttitudeConfig, simulate_attitude

class AttitudeRequest(BaseModel):
    altitude_km: float = Field(default=400, ge=200, le=5000)
    inclination_deg: float = Field(default=51.6, ge=0, le=180)
    duration_minutes: float = Field(default=35, ge=1, le=180)
    samples: int = Field(default=210, ge=30, le=800)
    dry_mass_kg: float = Field(default=600, gt=0, le=1000000)
    fuel_mass_kg: float = Field(default=150, ge=0, le=1000000)
    thrust_n: float = Field(default=850, ge=0, le=100000)
    specific_impulse_s: float = Field(default=315, ge=50, le=5000)
    burn_start_s: float = Field(default=300, ge=0, le=10800)
    burn_duration_s: float = Field(default=120, ge=0, le=10800)
    initial_misalignment_deg: float = Field(default=25, ge=0, le=180)
    target_mode: str = Field(default='prograde', pattern='^(prograde|retrograde|inertial_x)$')
    controller_enabled: bool = True
    max_control_torque_nm: float = Field(default=.25, ge=0, le=50)
    kp_nm_per_rad: float = Field(default=1.2, ge=0, le=100)
    kd_nms_per_rad: float = Field(default=18, ge=0, le=500)
    j2_enabled: bool = True
    gravity_gradient_enabled: bool = False

    @model_validator(mode='after')
    def validate_burn_window(self):
        if self.burn_start_s+self.burn_duration_s>self.duration_minutes*60+1e-9:
            raise ValueError('Burn must end inside mission duration')
        return self

@app.post('/api/attitude/mission')
def attitude_mission(req:AttitudeRequest):
    try:
        return simulate_attitude(AttitudeConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

# v1.1 — attitude determination and reaction-wheel momentum management.
from .adcs import ADCSConfig, simulate_adcs

class ADCSRequest(BaseModel):
    altitude_km: float=Field(default=400,ge=200,le=5000)
    inclination_deg: float=Field(default=51.6,ge=0,le=180)
    duration_minutes: float=Field(default=18,ge=1,le=90)
    samples: int=Field(default=109,ge=31,le=601)
    initial_misalignment_deg: float=Field(default=25,ge=0,le=180)
    initial_estimate_error_deg: float=Field(default=4,ge=0,le=90)
    target_mode: str=Field(default='prograde',pattern='^(prograde|retrograde|inertial_x)$')
    controller_enabled: bool=True
    tracker_enabled: bool=True
    gyro_bias_deg_s: float=Field(default=.015,ge=0,le=2)
    gyro_noise_deg_s: float=Field(default=.003,ge=0,le=2)
    tracker_noise_deg: float=Field(default=.035,ge=0,le=5)
    tracker_interval_s: float=Field(default=30,ge=1,le=3600)
    tracker_dropout_start_s: float=Field(default=400,ge=0,le=5400)
    tracker_dropout_duration_s: float=Field(default=0,ge=0,le=5400)
    attitude_blend: float=Field(default=.75,ge=0,le=1)
    bias_adaptation: float=Field(default=.08,ge=0,le=1)
    wheel_max_torque_nm: float=Field(default=.2,ge=0,le=10)
    wheel_momentum_limit_nms: float=Field(default=8,ge=0,le=1000)
    kp_nm_per_rad: float=Field(default=1.2,ge=0,le=100)
    kd_nms_per_rad: float=Field(default=16,ge=0,le=500)
    dry_mass_kg: float=Field(default=600,gt=0,le=1e6)
    fuel_mass_kg: float=Field(default=150,ge=0,le=1e6)
    thrust_n: float=Field(default=850,ge=0,le=1e5)
    specific_impulse_s: float=Field(default=315,ge=50,le=5000)
    burn_start_s: float=Field(default=300,ge=0,le=5400)
    burn_duration_s: float=Field(default=120,ge=0,le=5400)
    j2_enabled: bool=True
    gravity_gradient_enabled: bool=False
    seed: int=Field(default=27,ge=0,le=2**32-1)

    @model_validator(mode='after')
    def check_burn_schedule(self):
        if self.burn_start_s+self.burn_duration_s>self.duration_minutes*60+1e-9:
            raise ValueError('Burn ends after the ADCS mission')
        return self

@app.post('/api/adcs/mission')
def adcs_mission(req:ADCSRequest):
    try:
        return simulate_adcs(ADCSConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v1.2 — multiplicative attitude EKF and synthetic geomagnetic wheel unloading.
from .precision_adcs import PrecisionADCSConfig, simulate_precision_adcs

class PrecisionADCSRequest(ADCSRequest):
    gyro_bias_random_walk_deg_s_sqrt_s: float=Field(default=.000015,ge=0,le=1)
    gyro_initial_bias_sigma_deg_s: float=Field(default=.025,ge=0,le=2)
    gyro_bias_process_sigma_deg_s_sqrt_s: float=Field(default=.00003,ge=0,le=1)
    star_tracker_sun_exclusion_deg: float=Field(default=25,ge=0,le=180)
    star_tracker_earth_limb_margin_deg: float=Field(default=8,ge=0,le=60)
    tracker_visibility_enabled: bool=True
    desaturation_enabled: bool=True
    magnetorquer_max_dipole_am2: float=Field(default=120,ge=0,le=2000)
    desaturation_gain_per_s: float=Field(default=.01,ge=0,le=5)
    initial_wheel_momentum_nms: float=Field(default=2,ge=0,le=1000)

    @model_validator(mode='after')
    def check_wheel_initialization(self):
        if self.initial_wheel_momentum_nms>self.wheel_momentum_limit_nms:
            raise ValueError('Initial wheel momentum must not exceed wheel capacity')
        return self

@app.post('/api/precision-adcs/mission')
def precision_adcs_mission(req: PrecisionADCSRequest):
    try:
        return simulate_precision_adcs(PrecisionADCSConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v1.3 — causal, time-aligned loosely coupled orbital and attitude navigation.
from .integrated_nav import IntegratedNavConfig, simulate_integrated_navigation, FRAME as INTEGRATED_FRAME

class IntegratedNavigationRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=5000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=18,ge=2,le=60)
    samples: int = Field(default=72,ge=12,le=120)
    navigation_period_s: float = Field(default=60,ge=10,le=600)
    telemetry_latency_s: float = Field(default=45,ge=0,le=600)
    position_sigma_m: float = Field(default=30,ge=.01,le=1000)
    initial_position_offset_m: float = Field(default=600,ge=0,le=10000)
    initial_velocity_offset_m_s: float = Field(default=.3,ge=0,le=5)
    measurement_mode: str = Field(default='cartesian',pattern='^(cartesian|ground_range_rate)$')
    tracker_enabled: bool = True
    gyro_bias_deg_s: float = Field(default=.015,ge=0,le=2)
    burn_start_s: float = Field(default=300,ge=0,le=3600)
    burn_duration_s: float = Field(default=0,ge=0,le=3600)
    j2_enabled: bool = True
    seed: int = Field(default=27,ge=0,le=2147483647)
    reference_frame: str = Field(default=INTEGRATED_FRAME,pattern='^APOGEE_IDEAL_ECI$')
    propagate_attitude_covariance: bool = False

    @model_validator(mode='after')
    def validate_timeline(self):
        IntegratedNavConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/integrated')
def integrated_navigation_mission(req:IntegratedNavigationRequest):
    try:
        return simulate_integrated_navigation(IntegratedNavConfig(**req.model_dump()))
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v1.4 — absolute epoch reference laboratory. Never reinterpret ideal ECI as GCRS.
from .flight_references import reference_geometry

class FlightReferenceRequest(BaseModel):
    epoch_utc: str = '2026-10-24T12:00:00Z'
    position_km: list[float] = Field(default=[6778.137,0.,0.],min_length=3,max_length=3)
    velocity_km_s: list[float] = Field(default=[0.,7.6686,0.],min_length=3,max_length=3)
    input_frame: str = 'GCRS'
    include_igrf: bool = False

@app.post('/api/references/epoch')
def epoch_reference(req: FlightReferenceRequest):
    try:
        return reference_geometry(**req.model_dump())
    except (ValueError,RuntimeError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v1.5 — 12D joint error-state position/velocity/attitude/gyro-bias EKF.
from .joint_navigation import JointNavConfig, simulate_joint_navigation

class JointNavigationRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=5000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=12,ge=2,le=60)
    samples: int = Field(default=48,ge=31,le=120)
    navigation_period_s: float = Field(default=60,ge=10,le=600)
    telemetry_latency_s: float = Field(default=45,ge=0,le=600)
    position_sigma_m: float = Field(default=30,ge=.01,le=1000)
    initial_position_offset_m: float = Field(default=600,ge=0,le=10000)
    initial_velocity_offset_m_s: float = Field(default=.3,ge=0,le=5)
    measurement_mode: str = Field(default='cartesian',pattern='^cartesian$')
    tracker_enabled: bool = True
    gyro_bias_deg_s: float = Field(default=.015,ge=0,le=2)
    burn_start_s: float = Field(default=300,ge=0,le=3600)
    burn_duration_s: float = Field(default=120,ge=0,le=3600)
    j2_enabled: bool = True
    seed: int = Field(default=27,ge=0,le=2147483647)
    reference_frame: str = Field(default=INTEGRATED_FRAME,pattern='^APOGEE_IDEAL_ECI$')
    propagate_attitude_covariance: bool = False
    tracker_period_s: float = Field(default=60,ge=10,le=600)
    tracker_latency_s: float = Field(default=5,ge=0,le=600)
    tracker_sigma_deg: float = Field(default=.04,ge=.001,le=5)
    gyro_sigma_deg_s: float = Field(default=.015,ge=.00001,le=2)
    initial_attitude_offset_deg: float = Field(default=3,ge=0,le=90)

    @model_validator(mode='after')
    def verify_joint_timeline(self):
        JointNavConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/joint')
def joint_navigation_mission(req:JointNavigationRequest):
    try:
        return simulate_joint_navigation(JointNavConfig(**req.model_dump()))
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v1.6 — standalone sensor integrity / calibration stress laboratory.
from .integrity import IntegrityConfig, simulate_integrity

class IntegrityRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=5000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=12,ge=3,le=60)
    samples: int = Field(default=48,ge=24,le=180)
    measurement_sigma_m: float = Field(default=30,ge=.01,le=1000)
    correlation: float = Field(default=.65,ge=0,lt=.99)
    thrust_error_pct: float = Field(default=3,ge=-30,le=30)
    fault_start_s: float = Field(default=360,ge=0,le=3600)
    fault_magnitude_m: float = Field(default=220,ge=0,le=5000)
    reject_faults: bool = True
    burn_start_s: float = Field(default=180,ge=0,le=3600)
    burn_duration_s: float = Field(default=120,ge=0,le=3600)
    j2_enabled: bool = True
    seed: int = Field(default=29,ge=0,le=2147483647)
    monte_carlo_runs: int = Field(default=12,ge=1,le=32)
    reference_frame: str = Field(default='APOGEE_IDEAL_ECI',pattern='^APOGEE_IDEAL_ECI$')

    @model_validator(mode='after')
    def verify_integrity(self):
        IntegrityConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/integrity')
def integrity_mission(req:IntegrityRequest):
    try:
        return simulate_integrity(IntegrityConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

# v1.7 — experimental multisensor checks inside the correlated 12-state joint filter.
from .joint_integrity import JointIntegrityConfig, simulate_joint_integrity

class JointIntegrityRequest(BaseModel):
    altitude_km: float = Field(default=400,ge=200,le=5000)
    inclination_deg: float = Field(default=51.6,ge=0,le=180)
    duration_minutes: float = Field(default=12,ge=2,le=60)
    samples: int = Field(default=48,ge=31,le=96)
    navigation_period_s: float = Field(default=60,ge=10,le=600)
    tracker_period_s: float = Field(default=60,ge=10,le=600)
    telemetry_latency_s: float = Field(default=15,ge=0,le=600)
    tracker_latency_s: float = Field(default=15,ge=0,le=600)
    position_sigma_m: float = Field(default=30,ge=.01,le=1000)
    tracker_sigma_deg: float = Field(default=.04,ge=.001,le=5)
    gyro_sigma_deg_s: float = Field(default=.015,ge=.00001,le=2)
    gyro_bias_deg_s: float = Field(default=.015,ge=0,le=2)
    initial_position_offset_m: float = Field(default=600,ge=0,le=10000)
    initial_velocity_offset_m_s: float = Field(default=.3,ge=0,le=5)
    initial_attitude_offset_deg: float = Field(default=3,ge=0,le=90)
    measurement_mode: str = Field(default='cartesian',pattern='^cartesian$')
    tracker_enabled: bool = True
    tracker_visibility_enabled: bool = False
    j2_enabled: bool = True
    burn_start_s: float = Field(default=180,ge=0,le=3600)
    burn_duration_s: float = Field(default=120,ge=0,le=3600)
    sensor_correlation: float = Field(default=.25,ge=0,lt=.99)
    fault_channel: str = Field(default='position',pattern='^(none|position|tracker|both)$')
    fault_start_s: float = Field(default=360,ge=0,le=3600)
    position_fault_m: float = Field(default=250,ge=0,le=5000)
    tracker_fault_deg: float = Field(default=8,ge=0,le=90)
    engine_model_error_pct: float = Field(default=0,ge=-30,le=30)
    thrust_sigma_pct: float = Field(default=5,ge=0,le=50)
    gate_probability: float = Field(default=.9973,ge=.8,lt=1)
    reject_faults: bool = True
    monte_carlo_runs: int = Field(default=3,ge=1,le=8)
    seed: int = Field(default=29,ge=0,le=2147483647)
    reference_frame: str = Field(default='APOGEE_IDEAL_ECI',pattern='^APOGEE_IDEAL_ECI$')
    propagate_attitude_covariance: bool = False

    @model_validator(mode='after')
    def verify_joint_integrity(self):
        JointIntegrityConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/joint-integrity')
def joint_integrity_mission(req:JointIntegrityRequest):
    try:
        return simulate_joint_integrity(JointIntegrityConfig(**req.model_dump()))
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


# v1.8 — 13-error-state adaptive estimator and experimental fault-response policies.
from .adaptive_navigation import AdaptiveNavConfig, simulate_adaptive_navigation

class AdaptiveNavigationRequest(JointIntegrityRequest):
    fault_channel: str = Field(default='none',pattern='^(none|position|tracker|both)$')
    monte_carlo_runs: int = Field(default=1,ge=1,le=4)
    actual_thrust_error_pct: float = Field(default=5.,ge=-30,le=30)
    estimate_thrust_scale: bool = True
    initial_thrust_sigma_pct: float = Field(default=15.,ge=0,le=30)
    process_thrust_sigma_pct_sqrt_s: float = Field(default=.005,ge=0,le=.5)
    measurement_response: str = Field(default='gate',pattern='^(monitor|gate|quarantine)$')
    quarantine_after_alarms: int = Field(default=2,ge=1,le=10)
    quarantine_duration_s: float = Field(default=120.,ge=0,le=1200)
    adaptive_noise: bool = True
    adaptive_gain: float = Field(default=.15,ge=0,le=1)

    @model_validator(mode='after')
    def verify_joint_integrity(self):
        AdaptiveNavConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/adaptive')
def adaptive_navigation_mission(req:AdaptiveNavigationRequest):
    try:
        return simulate_adaptive_navigation(AdaptiveNavConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v1.9: offline local thrust-scale observability and paired robust batch fitting.
from .observability import ObservabilityConfig, simulate_observability

class ObservabilityRequest(BaseModel):
    altitude_km: float=Field(default=400,ge=200,le=5000)
    inclination_deg: float=Field(default=51.6,ge=0,le=180)
    duration_minutes: float=Field(default=12,ge=5,le=60)
    samples: int=Field(default=61,ge=31,le=241)
    observation_interval_s: float=Field(default=30,ge=10,le=300)
    observation_start_s: float=Field(default=0,ge=0,le=3600)
    observation_sigma_m: float=Field(default=30,ge=.01,le=1000)
    noise_correlation: float=Field(default=0,ge=0,lt=.99)
    burn_start_s: float=Field(default=180,ge=0,le=3600)
    burn_duration_s: float=Field(default=120,ge=0,le=360)
    true_thrust_scale: float=Field(default=1.05,ge=.7,le=1.3)
    fault_bias_m: float=Field(default=250,ge=0,le=5000)
    fault_start_s: float=Field(default=360,ge=0,le=3600)
    fault_duration_s: float=Field(default=60,ge=0,le=3600)
    estimator: str=Field(default='huber',pattern='^(least_squares|huber|gate)$')
    monte_carlo_runs: int=Field(default=48,ge=1,le=256)
    seed: int=Field(default=29,ge=0,le=2147483647)
    reference_frame: str=Field(default='APOGEE_IDEAL_ECI',pattern='^APOGEE_IDEAL_ECI$')

    @model_validator(mode='after')
    def verify_observability(self):
        ObservabilityConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/observability')
def observability_mission(req:ObservabilityRequest):
    try:
        return simulate_observability(ObservabilityConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v2.0 — causal, synthetic rotating-Earth station range/range-rate EKF.
from .ground_navigation import GroundNavConfig, simulate_ground_navigation, FRAME as GROUND_FRAME

class GroundNavigationRequest(BaseModel):
    altitude_km: float=Field(default=400,ge=200,le=2000)
    inclination_deg: float=Field(default=51.6,ge=0,le=180)
    duration_minutes: float=Field(default=12,ge=5,le=25)
    samples: int=Field(default=49,ge=25,le=101)
    burn_start_s: float=Field(default=180,ge=0,le=1500)
    burn_duration_s: float=Field(default=120,ge=0,le=240)
    true_thrust_scale: float=Field(default=1.05,ge=.7,le=1.3)
    measurement_interval_s: float=Field(default=45,ge=15,le=120)
    telemetry_delay_s: float=Field(default=20,ge=0,le=180)
    range_sigma_m: float=Field(default=20,ge=.1,le=500)
    range_rate_sigma_m_s: float=Field(default=.04,ge=.001,le=3)
    min_elevation_deg: float=Field(default=5,ge=0,le=35)
    station_count: int=Field(default=24)
    initial_position_error_m: float=Field(default=600,ge=0,le=10000)
    initial_velocity_error_m_s: float=Field(default=.4,ge=0,le=5)
    process_acceleration_sigma_m_s2: float=Field(default=.00015,ge=0,le=.01)
    fault_bias_m: float=Field(default=0,ge=0,le=3000)
    fault_start_s: float=Field(default=360,ge=0,le=1500)
    monte_carlo_runs: int=Field(default=3,ge=1,le=12)
    seed: int=Field(default=29,ge=0,le=2147483647)
    epoch_utc: str='2026-10-25T12:00:00Z'
    reference_frame: str=GROUND_FRAME

    @model_validator(mode='after')
    def validate_ground_model(self):
        GroundNavConfig(**self.model_dump()).validate()
        return self

@app.post('/api/navigation/ground-tracking')
def ground_tracking_mission(req:GroundNavigationRequest):
    try:
        return simulate_ground_navigation(GroundNavConfig(**req.model_dump()))
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

class GroundReferenceRecord(BaseModel):
    capture_time_s: float=Field(ge=0,le=1500)
    station_id: str=Field(pattern='^S[0-9]{2}$')
    range_m: float=Field(gt=0)
    range_rate_m_s: float

class GroundReferenceRequest(BaseModel):
    configuration: GroundNavigationRequest
    records: list[GroundReferenceRecord]=Field(min_length=1,max_length=200)

@app.post('/api/navigation/ground-reference-residuals')
def ground_reference_residuals(req:GroundReferenceRequest):
    from .ground_navigation import compare_reference_observations
    try:
        return compare_reference_observations(
            GroundNavConfig(**req.configuration.model_dump()),
            [r.model_dump() for r in req.records])
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

# v2.1 — real WGS84 geodesy, epoch-tagged ITRS geometry, strict optional GCRS/EOP.
from .real_tracking import calculate_observation, compare_records
from typing import Literal

class GeodeticStationRequest(BaseModel):
    latitude_deg: float = Field(default=45.0, ge=-90, le=90)
    longitude_deg: float = Field(default=-120.0, ge=-180, le=180)
    height_m: float = Field(default=100.0, ge=-500, le=10000)

class EpochStateRequest(BaseModel):
    position_km: list[float] = Field(min_length=3,max_length=3)
    velocity_km_s: list[float] = Field(min_length=3,max_length=3)

class EpochObservationRequest(BaseModel):
    epoch_utc: str = Field(default='2026-10-25T12:00:00Z',min_length=20,max_length=35)
    frame: Literal['ITRS','GCRS']='ITRS'
    station: GeodeticStationRequest = Field(default_factory=GeodeticStationRequest)
    state: EpochStateRequest = Field(default_factory=lambda:EpochStateRequest(
        position_km=[-3500.,-5800.,2200.],velocity_km_s=[5.,-3.,1.]))
    provenance: Literal['synthetic','external_unverified']='synthetic'
    observed: dict[str,float] | None = None

@app.post('/api/navigation/epoch-ground-observation')
def epoch_ground_observation(req:EpochObservationRequest):
    try:
        return calculate_observation(req.epoch_utc,req.station.model_dump(),
            req.state.position_km+req.state.velocity_km_s,req.frame,
            observed=req.observed,provenance=req.provenance)
    except (ValueError,RuntimeError,TypeError,OverflowError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

class ReferenceBatchRecord(BaseModel):
    epoch_utc: str=Field(min_length=20,max_length=35)
    frame: Literal['ITRS','GCRS']
    station: GeodeticStationRequest
    state: EpochStateRequest
    observed: dict[str,float]

class EpochReferenceBatchRequest(BaseModel):
    reference_id: str=Field(min_length=3,max_length=150)
    provenance: Literal['synthetic','external_unverified']='synthetic'
    records: list[ReferenceBatchRecord]=Field(min_length=1,max_length=200)

@app.post('/api/navigation/epoch-reference-comparison')
def epoch_reference_comparison(req:EpochReferenceBatchRequest):
    try:
        return compare_records([r.model_dump() for r in req.records],req.provenance,req.reference_id)
    except (ValueError,RuntimeError,TypeError,OverflowError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# APOGEE v2.2: transparent software verification with strict reference frames.
from .independent_validation import suite_report, sgp4_reference_benchmark

class IndependentBenchRequest(BaseModel):
    steps_per_orbit: int = Field(default=240, ge=40, le=1200)
    orbits: int = Field(default=2, ge=1, le=5)

@app.post('/api/verification/independent')
def independent_benchmark(req:IndependentBenchRequest):
    try:
        return suite_report(steps_per_orbit=req.steps_per_orbit, orbits=req.orbits)
    except (ValueError,ArithmeticError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc

@app.post('/api/verification/sgp4-reference')
def independent_reference_check(document:dict):
    try:
        return sgp4_reference_benchmark(document,allow_skip=False)
    except (ValueError,RuntimeError,TypeError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# APOGEE v2.3: multi-regime Vallado software verification and optional frame checks.
from .v23_validation import validation_report, parse_vallado_raw, compare_many

@app.post('/api/verification/v23')
def v23_benchmark(req: IndependentBenchRequest):
    try:
        return validation_report(steps_per_orbit=req.steps_per_orbit, orbits=req.orbits)
    except (ValueError, ArithmeticError, KeyError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

class ValladoImportRequest(BaseModel):
    tle_text: str = Field(max_length=2000000)
    output_text: str = Field(max_length=8000000)

@app.post('/api/verification/vallado-import')
def import_vallado(req: ValladoImportRequest):
    try:
        loaded=parse_vallado_raw(req.tle_text, req.output_text)
        compared=compare_many(loaded['reference'])
        return {'version':'2.3.0', 'import_summary':{k:v for k,v in loaded.items() if k!='reference'},
                'reference_fingerprints':loaded['reference']['provenance'], 'comparison':compared,
                'warning':'Published software verification vectors, not real-spacecraft measurements. Missing SGP4 means SKIP.'}
    except (ValueError,TypeError,KeyError) as exc:
        raise HTTPException(status_code=422,detail=str(exc)) from exc


# v2.4: strictly separate software verification from flight observation validation.
from .v24_validation import evaluate as v24_evaluate

class V24ReferenceRequest(BaseModel):
    steps_per_orbit: int = Field(default=240, ge=40, le=600)
    orbits: int = Field(default=2, ge=1, le=5)
    tle_text: str | None = Field(default=None, max_length=2000000)
    vector_text: str | None = Field(default=None, max_length=8000000)
    expected_tle_sha256: str | None = None
    expected_vector_sha256: str | None = None

@app.post('/api/verification/v24')
def v24_audit(req: V24ReferenceRequest):
    try:
        return v24_evaluate(steps_per_orbit=req.steps_per_orbit, orbits=req.orbits,
            tle_text=req.tle_text, vector_text=req.vector_text,
            expected_tle_sha256=req.expected_tle_sha256,
            expected_vector_sha256=req.expected_vector_sha256)
    except (ValueError, ArithmeticError, KeyError, TypeError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
# APOGEE v2.5 additive operations routes
from apogee.v25_operations import router as v25_router
app.include_router(v25_router)
# APOGEE v2.5: public read-only CORS for the Pages frontend
from fastapi.middleware.cors import CORSMiddleware as _V25CORS
app.add_middleware(_V25CORS, allow_origins=["https://chesswiz263-hub.github.io"], allow_credentials=False, allow_methods=["GET","POST","OPTIONS"], allow_headers=["*"])
