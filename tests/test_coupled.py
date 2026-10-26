"""v0.4: analytic consistency, cross-module regression, API semantics."""
from dataclasses import replace
from math import exp, log
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from apogee.coupled import integrate_coupled, compare_trajectories, derivative, coast_segment_diagnostics
from apogee.perturbations import ForceModel, acceleration as pert_acceleration, j2_acceleration
from apogee.physics import state_from_elements, integrate_orbit
from apogee.propulsion import Burn, G0_M_S2, integrate_mission


@pytest.fixture
def initial():
    return np.r_[state_from_elements(RE+400,0,51.6,30),750.]


def test_coupled_gravity_reduces_to_two_body(initial):
    model=ForceModel(j2=False,drag=False)
    result=integrate_coupled(initial,600,5400,180,[],model)
    conventional=integrate_orbit(initial[:6],5400,180)
    assert np.max(np.linalg.norm(result.states[:,:3]-conventional.states[:,:3],axis=1)) < 0.02
    assert result.states[-1,6]==750


def test_mass_loss_and_rocket_equation(initial):
    burn=Burn(0,120,850,315,'prograde')
    result=integrate_coupled(initial,600,600,100,[burn],ForceModel(j2=False,drag=False))
    mfinal=750-burn.mass_flow_kg_s*120
    assert result.states[-1,6]==pytest.approx(mfinal,abs=1e-8)
    assert result.burn_reports[0]['fuel_used_kg']==pytest.approx(750-mfinal,abs=1e-9)
    assert result.cumulative_ideal_delta_v_m_s[-1]==pytest.approx(315*G0_M_S2*log(750/mfinal),abs=1e-8)
    assert len([e for e in result.events if e['type']=='cutoff'])==1


def test_coupling_drag_depends_on_current_mass(initial):
    model=ForceModel(j2=True,drag=True,mass_kg=750)
    y=initial.copy();burn=Burn(0,30,900,300,'prograde')
    derivative_y=derivative(y,model,burn)
    g=pert_acceleration(y[:3],y[3:6],model)
    assert np.linalg.norm(derivative_y[3:6]-g)>0
    y[6]=700
    no_thrust=derivative(y,model,None)[3:6]
    assert np.allclose(no_thrust,pert_acceleration(y[:3],y[3:6],replace(model,mass_kg=700)),atol=1e-15)


def test_j2_and_burn_both_affect_motion(initial):
    burn=Burn(100,110,850,315,'normal')
    m=ForceModel(j2=True,drag=False)
    coupled=integrate_coupled(initial,600,5400,180,[burn],m)
    without_j2=integrate_coupled(initial,600,5400,180,[burn],replace(m,j2=False))
    without_burn=integrate_coupled(initial,600,5400,180,[],m)
    assert np.linalg.norm(coupled.states[-1,:3]-without_j2.states[-1,:3]) > 1
    assert np.linalg.norm(coupled.states[-1,:3]-without_burn.states[-1,:3]) > 10


def test_fuel_depletion_not_negative_and_event(initial):
    y=initial.copy();y[6]=602.
    b=Burn(0,250,850,315)
    result=integrate_coupled(y,600,500,100,[b],ForceModel())
    assert np.min(result.states[:,6])>=600
    assert result.states[-1,6]==pytest.approx(600,abs=1e-10)
    assert len([e for e in result.events if e['type']=='fuel_exhaustion'])==1
    assert result.events[1]['time_s']==pytest.approx(2/b.mass_flow_kg_s,abs=1e-8)


def test_compare_identical_forces_zero_difference(initial):
    force=ForceModel(j2=False,drag=False)
    a=integrate_coupled(initial,600,4200,140,[],force)
    b=integrate_coupled(initial,600,4200,140,[],force)
    cmp=compare_trajectories(a,b)
    assert cmp['max_separation_km']==0
    assert cmp['final_velocity_difference_m_s']==0


def test_compare_nonzero_when_j2_enabled(initial):
    a=integrate_coupled(initial,600,4200,140,[],ForceModel(j2=False,drag=False))
    b=integrate_coupled(initial,600,4200,140,[],ForceModel(j2=True,drag=False))
    assert compare_trajectories(a,b)['final_separation_km']>1


def test_conservation_scope(initial):
    m=ForceModel(j2=True,drag=False)
    r=integrate_coupled(initial,600,5400,180,[],m)
    diag=coast_segment_diagnostics(r,[],600)
    assert diag['conservation_applicable']
    assert diag['max_relative_energy_drift']<1e-7
    drag=integrate_coupled(initial,600,5400,180,[],replace(m,drag=True))
    assert coast_segment_diagnostics(drag,[],600)['max_relative_energy_drift'] is None


def test_reject_overlapping_burns(initial):
    burns=[Burn(100,80,850,315),Burn(170,50,850,315)]
    with pytest.raises(ValueError,match='Overlapping'):
        integrate_coupled(initial,600,600,100,burns,ForceModel())


def test_compare_endpoint_contract():
    client=TestClient(app)
    req={'duration_minutes':30,'samples':120,'burns':[{'start_s':300,'duration_s':80}],
         'j2_enabled':True,'drag_enabled':False}
    r=client.post('/api/compare',json=req)
    assert r.status_code==200,r.text
    data=r.json()
    assert len(data['trajectory']['position_km'])>100
    assert data['comparison']['final_separation_km']>0
    assert len(data['comparison']['baseline_trajectory']['position_km'])==len(data['trajectory']['position_km'])
    assert data['model']['force_terms'].count('Earth J2 geopotential')==1


def test_mission_drag_conservation_not_claimed():
    client=TestClient(app)
    r=client.post('/api/mission',json={'duration_minutes':20,'samples':120,'burns':[],
                                       'j2_enabled':True,'drag_enabled':True})
    assert r.status_code==200,r.text
    data=r.json()
    assert data['metrics']['conservation_applicable'] is False
    assert data['metrics']['max_relative_energy_drift'] is None
    assert data['trajectory']['relative_energy_drift']==[]
    assert any('illustrative' in w for w in data['warnings'])


def test_comparison_zero_at_same_force_flags():
    client=TestClient(app)
    r=client.post('/api/compare',json={'duration_minutes':30,'samples':120,'burns':[],
                                        'j2_enabled':False,'drag_enabled':False})
    assert r.status_code==200
    assert r.json()['comparison']['max_separation_km']==pytest.approx(0)


def test_dop853_independent_numerical_reference(initial):
    scipy=pytest.importorskip('scipy')
    from apogee.benchmark import dop853_reference
    burn=Burn(100,60,850,315,'prograde')
    model=ForceModel(j2=True,drag=True)
    r=integrate_coupled(initial,600,900,90,[burn],model)
    high=dop853_reference(initial,600,r.time_s,[burn],model)
    assert np.max(np.linalg.norm(r.states[:,:3]-high[:,:3],axis=1))<1e-4
    assert abs(r.states[-1,6]-high[-1,6])<1e-8
