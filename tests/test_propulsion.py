import math
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.constants import EARTH_EQUATORIAL_RADIUS_KM as R
from apogee.physics import elements_from_state, state_from_elements
from apogee.propulsion import (Burn, G0_M_S2, finite_derivative, integrate_mission,
                               rocket_equation_budget, steering_unit)

client=TestClient(app)


def test_rocket_equation_feasible_mass_ratio_and_two_burn_budget():
    dv=[82.57,81.68]
    dry,isp=600.,315.
    budget=rocket_equation_budget(dv,dry,80,isp)
    total=sum(dv)
    expected=dry*math.expm1(total/(isp*G0_M_S2))
    assert budget['required_propellant_kg'] == pytest.approx(expected, rel=1e-12)
    assert budget['feasible']
    assert budget['ideal_final_mass_kg'] > dry
    assert sum(b['ideal_propellant_used_kg'] for b in budget['burns_if_fully_feasible']) == pytest.approx(680-budget['ideal_final_mass_kg'])


def test_rocket_equation_shortfall_and_limit():
    b=rocket_equation_budget([1000],1000,20,300)
    assert not b['feasible']
    assert b['burns_if_fully_feasible'] is None
    assert b['ideal_final_mass_kg'] is None
    assert b['maximum_ideal_delta_v_m_s'] == pytest.approx(300*G0_M_S2*math.log(1.02))
    with pytest.raises(ValueError): rocket_equation_budget([-1],1000,200,300)


def test_rtn_directions_are_orthonormal_and_prograde_matches_velocity():
    state=state_from_elements(R+400, 0, 63,30,0,11)
    radial=steering_unit(state,'radial_out')
    transverse=steering_unit(state,'transverse')
    normal=steering_unit(state,'normal')
    assert np.dot(radial,transverse)==pytest.approx(0,abs=1e-14)
    assert np.dot(radial,normal)==pytest.approx(0,abs=1e-14)
    assert np.dot(transverse,normal)==pytest.approx(0,abs=1e-14)
    assert np.dot(steering_unit(state,'prograde'),state[3:]/np.linalg.norm(state[3:])) ==pytest.approx(1)


def test_finite_derivative_matches_gravity_and_engine_mass_flow():
    state=np.r_[state_from_elements(R+400,0),750.]
    burn=Burn(0,120,850,315,'prograde')
    coast=finite_derivative(state,None,600)
    powered=finite_derivative(state,burn,600)
    assert powered[6] == pytest.approx(-850/(315*G0_M_S2))
    assert np.linalg.norm(powered[3:6]-coast[3:6]) ==pytest.approx(850/750/1000)


def test_no_burn_mission_is_close_to_two_body_ellipse_and_conserves_mass():
    s=state_from_elements(R+400,0,25)
    period=elements_from_state(s).period_s
    m=integrate_mission(np.r_[s,760.],600,period,600,[])
    assert np.linalg.norm(m.states[-1,:3]-s[:3])<0.001
    assert np.all(m.states[:,6]==760)
    assert len(m.events)==0


def test_finite_burn_consumes_expected_prop_and_changes_energy():
    state=state_from_elements(R+400,0)
    burn=Burn(300,100,950,315,'prograde')
    result=integrate_mission(np.r_[state,750.],600,2400,240,[burn])
    expected=burn.mass_flow_kg_s*burn.duration_s
    assert result.states[-1,6] == pytest.approx(750-expected,abs=1e-9)
    assert result.burn_reports[0]['fuel_used_kg'] == pytest.approx(expected,abs=1e-9)
    assert result.burn_reports[0]['delivered_ideal_delta_v_m_s'] == pytest.approx(315*G0_M_S2*math.log(750/(750-expected)),rel=1e-10)
    assert [ev['type'] for ev in result.events]==['ignition','cutoff']
    assert elements_from_state(result.states[-1,:6]).semi_major_axis_km > R+400


def test_fuel_exhaustion_coasts_and_never_restarts_without_fuel():
    state=state_from_elements(R+400,0)
    burn=Burn(30,180,6000,300,'prograde')
    result=integrate_mission(np.r_[state,610.],600,480,240,[burn])
    assert min(result.states[:,6]) == pytest.approx(600,abs=1e-8)
    assert result.states[-1,6] == pytest.approx(600,abs=1e-8)
    assert result.burn_reports[0]['fuel_used_kg'] == pytest.approx(10,abs=1e-8)
    assert result.burn_reports[0]['propellant_depleted']
    assert 'fuel_exhaustion' in [e['type'] for e in result.events]
    assert sum(e['type']=='fuel_exhaustion' for e in result.events)==1


def test_two_nonoverlapping_burns_and_boundary_events():
    state=state_from_elements(R+400,0)
    bs=[Burn(12.5,16,250,300,'prograde',name='A'),Burn(41,10,250,300,'retrograde',name='B')]
    out=integrate_mission(np.r_[state,750.],600,600,120,bs)
    assert {'ignition','cutoff'} == {e['type'] for e in out.events}
    for b in bs:
        assert np.any(np.isclose(out.time_s,b.start_s))
        assert np.any(np.isclose(out.time_s,b.end_s))
    with pytest.raises(ValueError): integrate_mission(np.r_[state,750.],600,600,120,[Burn(0,80,200,300),Burn(60,100,200,300)])


def test_mission_api_budget_and_validation():
    response=client.post('/api/mission',json={'duration_minutes':30,'samples':200,
                          'burns':[{'start_s':120,'duration_s':80,'thrust_n':600}]})
    assert response.status_code==200,response.text
    body=response.json()
    assert body['metrics']['propellant_used_kg']>0
    assert body['metrics']['ideal_thrust_delta_v_m_s']>0
    assert body['final_orbital_elements']['semi_major_axis_km']>body['orbital_elements']['semi_major_axis_km']
    assert len(body['trajectory']['wet_mass_kg'])==body['metrics']['samples']
    assert body['metrics']['numerical_integrity_scope']=='coasting arcs only'
    assert client.post('/api/mission',json={'duration_minutes':5, 'burns':[{'start_s':200,'duration_s':200}]}).status_code==422
    assert client.post('/api/mission',json={'burns':[{'thrust_n':-1}]}).status_code==422
    assert client.post('/api/mission',json={'burns':[{'direction':'magical'}]}).status_code==422
    assert client.post('/api/mission',json={'burns':[{'start_s':50,'duration_s':50},{'start_s':75,'duration_s':25}]}).status_code==422


def test_transfer_budget_feasible_and_insertion_verification():
    d=client.post('/api/transfer',json={'start_altitude_km':400,'target_altitude_km':700,
             'dry_mass_kg':600,'propellant_kg':100,'specific_impulse_s':315}).json()
    assert d['transfer']['fuel_budget']['feasible']
    assert abs(d['transfer']['post_circularization_eccentricity'])<1e-5
    assert d['transfer']['fuel_budget']['required_propellant_kg']>30
    d2=client.post('/api/transfer',json={'dry_mass_kg':600,'propellant_kg':1}).json()
    assert not d2['transfer']['fuel_budget']['feasible']
    assert any('INSUFFICIENT FUEL' in w for w in d2['warnings'])


def test_finite_burn_rk4_against_independent_scipy_dop853():
    scipy=pytest.importorskip('scipy.integrate')
    from apogee.constants import EARTH_MU_KM3_S2 as MU
    s=state_from_elements(R+460,.02,37,14)
    initial=np.r_[s,840.]
    burn=Burn(200,135,1100,325,'prograde')
    result=integrate_mission(initial,650,1600,320,[burn])
    y=initial.copy()
    for t0,t1,ison in [(0,200,False),(200,335,True),(335,1600,False)]:
        def rhs(t,vector):
            r,v=vector[:3],vector[3:6]
            gravity=-MU*r/np.linalg.norm(r)**3
            added=(burn.thrust_n/(1000*vector[6]))*v/np.linalg.norm(v) if ison else np.zeros(3)
            flow=-burn.thrust_n/(burn.isp_s*G0_M_S2) if ison else 0
            return np.r_[v, gravity+added, flow]
        sol=scipy.solve_ivp(rhs,(t0,t1),y,method='DOP853',rtol=2e-13,atol=2e-13)
        assert sol.success
        y=sol.y[:,-1]
    # 5 s powered and <=30 s coasting internal RK4, independently integrated.
    assert np.linalg.norm(result.states[-1,:3]-y[:3])<0.0003
    assert np.linalg.norm(result.states[-1,3:6]-y[3:6])<1e-6
    assert result.states[-1,6]==pytest.approx(y[6],abs=1e-8)


def test_two_different_isp_burns_cumulative_dv_history():
    state=state_from_elements(R+400,0)
    burns=[Burn(50,50,800,300),Burn(300,90,1400,450,'retrograde')]
    out=integrate_mission(np.r_[state,900.],600,800,200,burns)
    assert out.cumulative_ideal_delta_v_m_s[-1]==pytest.approx(sum(b['delivered_ideal_delta_v_m_s'] for b in out.burn_reports))
    assert np.all(np.diff(out.cumulative_ideal_delta_v_m_s)>=-1e-12)
    assert out.burn_reports[0]['delivered_ideal_delta_v_m_s']>0
    assert out.burn_reports[1]['delivered_ideal_delta_v_m_s']>0
