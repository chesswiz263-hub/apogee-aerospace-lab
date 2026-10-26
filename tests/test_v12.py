"""v1.2: educational attitude MEKF, geometry and dipole-unloading regressions."""
import math
import numpy as np
import pytest
from fastapi.testclient import TestClient
from apogee.api import app
from apogee.attitude import q_normalize
from apogee.precision_adcs import (PrecisionADCSConfig,AttitudeMEKF,simulate_precision_adcs,
    cross_matrix,symmetric_psd,magnetic_field_inertial_tesla,magnetic_unload_torque,
    tracker_visibility)

@pytest.fixture(scope='module')
def baseline():return simulate_precision_adcs(PrecisionADCSConfig())

@pytest.fixture(scope='module')
def without_unloading():return simulate_precision_adcs(PrecisionADCSConfig(desaturation_enabled=False))


def test_cross_product_matrix():
    a=np.array([1.,-2.,3.]);b=np.array([2.,3.,-4.])
    assert np.allclose(cross_matrix(a)@b,np.cross(a,b))


def test_psd_symmetrization():
    a=np.array([[1.,2.],[-1.,-.1]])
    p=symmetric_psd(a,1e-12)
    assert np.allclose(p,p.T) and np.linalg.eigvalsh(p).min()>=-1e-14


def test_filter_zero_motion():
    kf=AttitudeMEKF([1.,0,0,0])
    for i in range(12):
        kf.predict([0,0,0],1,.003,.00003)
        if i%3==0:kf.update([1.,0,0,0],.035)
    assert np.allclose(kf.q,[1,0,0,0],atol=1e-12)
    assert np.linalg.eigvalsh(kf.P).min()>=0


def test_filter_innovation_near_zero():
    kf=AttitudeMEKF([1,0,0,0]);nis=kf.update([1,0,0,0],.035)
    assert nis==pytest.approx(0,abs=1e-12)


def test_filter_reduces_covariance_on_fix():
    kf=AttitudeMEKF([1.,0,0,0]);before=np.trace(kf.P[:3,:3]);kf.update([1.,0,0,0],.035)
    assert np.trace(kf.P[:3,:3])<before


def test_filter_estimates_fixed_rotation():
    q=[np.cos(.03),0,0,np.sin(.03)]
    kf=AttitudeMEKF([1,0,0,0]);kf.update(q,.005)
    assert abs(kf.q[3]-q[3])<1e-5


def test_filter_bias_covariance_positive_after_long_coast():
    kf=AttitudeMEKF([1,0,0,0]);
    for _ in range(40):kf.predict([.001,-.002,.003],10,.003,.00003)
    assert np.all(np.isfinite(kf.P)) and np.linalg.eigvalsh(kf.P).min()>0


def test_dipole_field_reverses_across_equator():
    a=magnetic_field_inertial_tesla([6700,0,0])
    b=magnetic_field_inertial_tesla([-6700,0,0])
    assert np.allclose(a,b)
    assert 1e-6<np.linalg.norm(a)<1e-4


def test_magnetic_torque_perpendicular_to_B():
    cfg=PrecisionADCSConfig(initial_wheel_momentum_nms=2)
    r=[6700.,100.,900.];q=[1.,0,0,0]
    tau,m=magnetic_unload_torque(r,q,[2,2,0],cfg)
    B=magnetic_field_inertial_tesla(r)
    assert abs(np.dot(tau,B))<1e-15
    assert np.allclose(tau,np.cross(m,B))


def test_magnet_dipole_limit():
    cfg=PrecisionADCSConfig(magnetorquer_max_dipole_am2=2)
    _,m=magnetic_unload_torque([6800,100,0],[1,0,0,0],[1,2,3],cfg)
    assert np.linalg.norm(m)<=2+1e-12


def test_disable_unloading_zero_moment():
    cfg=PrecisionADCSConfig(desaturation_enabled=False)
    t,m=magnetic_unload_torque([6800,0,0],[1,0,0,0],[1,2,3],cfg)
    assert not np.any(t) and not np.any(m)


def test_visibility_sun_block():
    cfg=PrecisionADCSConfig(star_tracker_sun_exclusion_deg=45)
    good,sa,em=tracker_visibility([6800,0,0],[1,0,0,0],cfg)
    assert not good and sa==pytest.approx(0)


def test_visibility_earth_limb_block():
    # Body +X points toward Earth for a location on inertial -X.
    cfg=PrecisionADCSConfig(star_tracker_sun_exclusion_deg=0)
    good,sa,em=tracker_visibility([-6800,0,0],[1,0,0,0],cfg)
    assert not good and em<0


def test_output_dimensions_and_quaternions(baseline):
    d=baseline['precision_adcs'];n=len(baseline['trajectory']['time_s'])
    assert n==110
    assert len(d['nis'])==len(d['nees'])==len(d['attitude_estimation_error_deg'])==n
    assert np.max(abs(np.linalg.norm(d['estimated_quaternion_BI_wxyz'],axis=1)-1))<1e-11


def test_filter_consistency_finite(baseline):
    d=baseline['precision_adcs']
    assert 0<d['mean_nis']<100
    assert 0<d['mean_nees']<100
    assert np.all(np.array(d['estimated_attitude_3axis_sigma_deg'])>=0)


def test_target_pointing_and_bias(baseline):
    d=baseline['precision_adcs']
    assert d['final_pointing_error_deg']<1
    assert d['final_estimation_error_deg']<1
    assert abs(d['true_final_bias_deg_s'][2]-d['estimated_final_bias_deg_s'][2])<.002


def test_gating_suppresses_some_fixes(baseline):
    d=baseline['precision_adcs']
    assert d['tracker_fixes']>10
    assert d['tracker_blocked_earth']>0 or d['tracker_blocked_sun']>0


def test_unloading_reduces_momentum(baseline,without_unloading):
    yes=baseline['precision_adcs']['final_wheel_momentum_norm_nms']
    no=without_unloading['precision_adcs']['final_wheel_momentum_norm_nms']
    assert yes<no*.7


def test_bounded_wheel_and_magnet(baseline):
    a=baseline['precision_adcs'];cfg=PrecisionADCSConfig()
    assert np.max(abs(np.asarray(a['wheel_momentum_body_nms'])))<=cfg.wheel_momentum_limit_nms+1e-9
    assert np.max(abs(np.asarray(a['wheel_commanded_momentum_rate_nm'])))<=cfg.wheel_max_torque_nm+1e-9
    assert max(np.linalg.norm(v) for v in a['magnetic_dipole_am2'])<=cfg.magnetorquer_max_dipole_am2+1e-8


def test_propellant_and_mass(baseline):
    m=baseline['metrics'];cfg=PrecisionADCSConfig()
    assert m['propellant_used_kg']==pytest.approx(cfg.thrust_n*cfg.burn_duration_s/(cfg.specific_impulse_s*9.80665),abs=1e-8)
    assert min(baseline['trajectory']['wet_mass_kg'])>=cfg.dry_mass_kg-1e-9


def test_no_tracker_observations():
    d=simulate_precision_adcs(PrecisionADCSConfig(tracker_enabled=False))['precision_adcs']
    assert d['tracker_fixes']==0 and d['mean_nis'] is None
    assert d['final_estimation_error_deg']>5


def test_dry_mass_protection():
    cfg=PrecisionADCSConfig(fuel_mass_kg=.1,burn_start_s=20,burn_duration_s=60,duration_minutes=3,samples=35)
    r=simulate_precision_adcs(cfg)
    assert r['metrics']['fuel_exhausted']
    assert r['metrics']['propellant_remaining_kg']==pytest.approx(0,abs=1e-7)


def test_seed_reproducible():
    cfg=PrecisionADCSConfig(duration_minutes=3,samples=35,burn_start_s=20,burn_duration_s=40)
    a=simulate_precision_adcs(cfg);b=simulate_precision_adcs(cfg)
    assert a['precision_adcs']['attitude_estimation_error_deg']==b['precision_adcs']['attitude_estimation_error_deg']


def test_invalid_config():
    for cfg in [PrecisionADCSConfig(initial_wheel_momentum_nms=9),PrecisionADCSConfig(star_tracker_sun_exclusion_deg=200),PrecisionADCSConfig(magnetorquer_max_dipole_am2=-2)]:
        with pytest.raises(ValueError):cfg.validate()


def test_new_api():
    c=TestClient(app)
    request={'duration_minutes':3,'samples':35,'burn_start_s':10,'burn_duration_s':40}
    r=c.post('/api/precision-adcs/mission',json=request)
    assert r.status_code==200,r.text[:300]
    assert 'precision_adcs' in r.json()
    assert c.get('/api/health').json()['version']=='2.4.0'


def test_invalid_api():
    c=TestClient(app)
    for request in [{'magnetorquer_max_dipole_am2':-1}, {'initial_wheel_momentum_nms':10,'wheel_momentum_limit_nms':8}, {'star_tracker_sun_exclusion_deg':200}]:
        assert c.post('/api/precision-adcs/mission',json=request).status_code==422
