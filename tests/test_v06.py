"""v0.6 regression: true event roots, phase ranking and frame-aligned residuals."""
import numpy as np
import pytest
from fastapi.testclient import TestClient
from scipy.integrate import solve_ivp

from apogee.api import app
from apogee.constants import EARTH_MU_KM3_S2 as MU, EARTH_EQUATORIAL_RADIUS_KM as RE
from apogee.ephemeris import METADATA
from apogee.optimization import optimize_impulsive_transfer
from apogee.physics import state_from_elements, hohmann_transfer
from apogee.reference_check import compare_imported_reference

client=TestClient(app)


def fixture_vectors(alter_km=0, num=12, dt=120):
    """Clearly synthetic, known point-mass satellite in ICRF-like fixed axes."""
    state=state_from_elements(RE+500,.005,38.0,19.0,27.0,45.0)
    def f(t,y):
        return np.r_[y[3:],-MU*y[:3]/np.linalg.norm(y[:3])**3]
    tt=np.arange(num,dtype=float)*dt
    ode=solve_ivp(f,(0,float(tt[-1])),state,rtol=1e-13,atol=1e-13,method='DOP853',t_eval=tt)
    lines=['Target body name: SYNTHETIC TEST SPACECRAFT','Center body name: Earth (399)',
           'Output units: KM-S','Time format: TDB','$$SOE']
    for i,y in enumerate(ode.y.T):
        y=y.copy()
        if i>0:y[0]+=alter_km
        jd=2461000.5+tt[i]/86400
        lines.append(', '.join([f'{jd:.11f}','A.D. SYNTHETIC']+[f'{x:.14E}' for x in y])+',')
    return '\n'.join(lines+['$$EOE'])

@pytest.mark.parametrize('j2',[False,True])
def test_event_target_meets_radius(j2):
    r=optimize_impulsive_transfer(j2_enabled=j2,phase_candidates=4,samples=180)
    choice=r['optimization']['selected']
    assert abs(choice['target_radius_residual_m'])<0.005
    assert choice['arrival_time_s']>2000
    assert r['events'][1]['time_s']==pytest.approx(choice['arrival_time_s'],abs=1e-4)
    assert r['metrics']['samples']==181
    assert r['optimization']['fuel_budget']['feasible']
    assert abs(r['trajectory']['altitude_km'][-1]-700)<0.00001

@pytest.mark.parametrize('start,target',[(400,700),(700,400)])
def test_no_j2_optimization_agrees_with_hohmann(start,target):
    r=optimize_impulsive_transfer(start_altitude_km=start,target_altitude_km=target,
                                j2_enabled=False,phase_candidates=4,samples=120)
    target_dv=r['optimization']['selected']['total_delta_v_m_s']
    expected=hohmann_transfer(RE+start,RE+target)['total_delta_v_km_s']*1000
    assert target_dv==pytest.approx(expected,abs=0.001)
    assert np.sign(r['optimization']['selected']['departure_delta_v_m_s'])==np.sign(target-start)


def test_search_selects_smallest_fuel_from_its_actual_grid():
    r=optimize_impulsive_transfer(phase_candidates=8,j2_enabled=True,samples=120)
    successful=[x for x in r['optimization']['candidate_results'] if 'fuel_required_kg' in x]
    assert successful
    assert r['optimization']['selected']['fuel_required_kg']==min(x['fuel_required_kg'] for x in successful)
    assert 'globally optimal' in r['optimization']['optimality_scope']


def test_fuel_shortfall_is_not_marked_feasible():
    r=optimize_impulsive_transfer(phase_candidates=1,propellant_kg=1)
    assert not r['optimization']['fuel_budget']['feasible']
    assert any('INSUFFICIENT' in w for w in r['warnings'])

@pytest.mark.parametrize('change',[
    {'start_altitude_km':100},{'phase_candidates':7},
    {'target_altitude_km':400},{'specific_impulse_s':-1},
    {'target_altitude_km':float('nan')},{'samples':20},
])
def test_reject_bad_optimization(change):
    with pytest.raises(ValueError):optimize_impulsive_transfer(**change)


def test_synthetic_reference_matches_two_body_to_meters():
    r=compare_imported_reference(fixture_vectors(),METADATA)
    assert r['validation']['position_rms_km']<0.0002
    assert r['validation']['position_max_km']<0.0002
    assert r['comparison']['separation_km'][0]==pytest.approx(0)
    assert r['comparison']['baseline_trajectory']['time_s']==r['trajectory']['time_s']
    assert r['validation']['frame']=='Earth-centered ICRF'
    assert any('unverified' in w.lower() or 'provenance' in w.lower() for w in r['warnings'])


def test_reference_residual_detects_modified_samples():
    r=compare_imported_reference(fixture_vectors(alter_km=0.75),METADATA)
    assert r['validation']['position_final_km']>0.74
    assert r['validation']['position_max_km']>0.74
    assert r['validation']['position_rms_km']>0.5


def test_reference_rejects_mismatched_frame():
    with pytest.raises(ValueError):compare_imported_reference(fixture_vectors(),{**METADATA,'frame':'TEME'})


def test_reference_rejects_overlong_span():
    with pytest.raises(ValueError,match='36 hours'):
        compare_imported_reference(fixture_vectors(dt=5*3600),METADATA)


def test_reference_rejects_oversized_geocentric_orbit():
    text=fixture_vectors().replace('Time format: TDB','Time format: TDB')
    # Directly alter first-state position to an altitude > 60,000 km.
    lines=text.splitlines()
    idx=lines.index('$$SOE')+1
    row=lines[idx].split(',')
    row[2]='9.0E+04';lines[idx]=','.join(row)
    with pytest.raises(ValueError):compare_imported_reference('\n'.join(lines),METADATA)


def test_optimization_api():
    res=client.post('/api/optimize',json={'phase_candidates':4,'samples':120,'j2_enabled':True})
    assert res.status_code==200,res.text
    data=res.json()
    assert data['optimization']['selected']['feasible']
    assert len(data['trajectory']['time_s'])==121
    assert 'events' in data


def test_optimization_api_rejects_invalid_grid():
    assert client.post('/api/optimize',json={'phase_candidates':999}).status_code==422


def test_reference_comparison_api():
    res=client.post('/api/reference/compare',json={'text':fixture_vectors(),'metadata':METADATA})
    assert res.status_code==200,res.text
    assert res.json()['validation']['position_rms_km']<.0002


def test_reference_comparison_api_rejects_teme():
    res=client.post('/api/reference/compare',json={'text':fixture_vectors(),
                      'metadata':{**METADATA,'frame':'TEME'}})
    assert res.status_code==422
