from math import sqrt, cos, radians, pi
import numpy as np
import pytest
from fastapi.testclient import TestClient

from apogee.api import app
from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as RE, EARTH_MU_KM3_S2 as MU, EARTH_J2
from apogee.physics import state_from_elements, integrate_orbit
from apogee.perturbations import (ForceModel, j2_acceleration, atmosphere_density,
    drag_acceleration, j2_specific_potential, propagate, diagnostics)

CLIENT=TestClient(app)


def test_j2_equatorial_extra_gravity_radial():
    r=np.array([RE+400,0.,0.])
    expected=-1.5*EARTH_J2*MU*RE**2 / np.linalg.norm(r)**4
    assert j2_acceleration(r)[0]==pytest.approx(expected)
    assert j2_acceleration(r)[1]==0


def test_j2_polar_acceleration_orientation():
    r=np.array([0.,0.,RE+400])
    expected=3*EARTH_J2*MU*RE**2/np.linalg.norm(r)**4
    assert j2_acceleration(r)[2]==pytest.approx(expected)


def test_j2_potential_gradient_matches_acceleration():
    r=np.array([5100.,3300.,2700.]);h=0.001
    gradient=np.array([(j2_specific_potential(r+np.eye(3)[i]*h,True)-
             j2_specific_potential(r-np.eye(3)[i]*h,True))/(2*h) for i in range(3)])
    central=-MU*r/np.linalg.norm(r)**3
    assert np.allclose(-gradient-central,j2_acceleration(r),atol=2e-10)


def test_drag_opposes_corotating_atmosphere_relative_velocity():
    model=ForceModel(drag=True)
    r=np.array([RE+400.,0,0]);v=np.array([0.,7.67,0])
    a=drag_acceleration(r,v,model)
    assert a[1]<0 and a[0]==pytest.approx(0,abs=1e-18)
    assert np.dot(a,v-np.cross([0,0,7.292115e-5],r))<0


def test_atmosphere_density_reference_and_exponential():
    model=ForceModel()
    assert atmosphere_density(400,model)==pytest.approx(model.density_at_400_km_kg_m3)
    assert atmosphere_density(400+model.scale_height_km,model)==pytest.approx(model.density_at_400_km_kg_m3/np.e)


def test_disabled_forces_match_existing_two_body():
    initial=state_from_elements(RE+500,.03,51.6,30)
    duration=5400
    ours=propagate(initial,duration,180,ForceModel(j2=False,drag=False))
    reference=integrate_orbit(initial,duration,360)
    assert np.linalg.norm(ours.states[-1]-reference.states[-1])<.0001


def test_j2_energy_and_axial_momentum_invariants():
    initial=state_from_elements(RE+400,.01,51.6,30)
    out=propagate(initial,8*3600,320,ForceModel(j2=True,drag=False))
    d=diagnostics(out.states,ForceModel(j2=True,drag=False))
    assert d['max_relative_energy_drift']<1e-7
    assert d['max_relative_angular_momentum_drift']<1e-7


def test_j2_node_precession_matches_secular_sign_and_order():
    # For nearly circular 51.6° orbit, secular node rate is retrograde.
    client=CLIENT.post('/api/perturbations',json={'perigee_altitude_km':500,
                     'eccentricity':.001,'inclination_deg':51.6,'duration_hours':48,'samples':480,
                     'j2_enabled':True,'drag_enabled':False})
    assert client.status_code==200,client.text
    got=client.json()['metrics']['delta_raan_deg']
    a=RE+500;n=sqrt(MU/a**3)
    rate=-1.5*EARTH_J2*n*(RE/a)**2*cos(radians(51.6))*180/pi
    expected=rate*48*3600
    assert got<0
    assert got==pytest.approx(expected,rel=.10)


def test_drag_physically_dissipates_energy_and_disables_invariant_badges():
    model=ForceModel(j2=True,drag=True,mass_kg=200,area_m2=15)
    initial=state_from_elements(RE+400,0,25)
    out=propagate(initial,3*3600,150,model)
    d=diagnostics(out.states,model)
    assert d['relative_specific_energy_change']<0
    assert d['max_relative_energy_drift'] is None
    assert not d['conservation_applicable']


def test_area_zero_makes_drag_identically_zero():
    r=np.array([RE+400,0.,0.]);v=np.array([0.,7.7,0.])
    assert np.all(drag_acceleration(r,v,ForceModel(drag=True,area_m2=0))==0)


def test_new_api_and_validation():
    r=CLIENT.post('/api/perturbations',json={'duration_hours':1,'samples':120})
    assert r.status_code==200,r.text
    d=r.json()
    assert d['model']['force_terms']==['point-mass gravity','Earth J2 geopotential']
    assert d['metrics']['conservation_applicable']
    assert len(d['trajectory']['time_s'])==121
    r=CLIENT.post('/api/perturbations',json={'duration_hours':200})
    assert r.status_code==422


def test_drag_api_explicitly_not_conserved():
    r=CLIENT.post('/api/perturbations',json={'duration_hours':1,'samples':120,'drag_enabled':True})
    assert r.status_code==200
    d=r.json();assert d['metrics']['conservation_applicable'] is False
    assert d['warnings']
