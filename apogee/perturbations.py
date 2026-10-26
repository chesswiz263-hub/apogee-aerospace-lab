"""Earth J2 + illustrative co-rotating exponential-atmosphere drag.

Units: Earth-centered inertial km, km/s, seconds, kilograms; SI drag is
converted to km/s². This is NOT an operational atmospheric model.
"""
from dataclasses import dataclass
from math import ceil, exp
import numpy as np

from .constants import EARTH_EQUATORIAL_RADIUS_KM as RE
from .constants import EARTH_MU_KM3_S2 as MU, EARTH_J2

EARTH_SPIN_RAD_S = 7.2921150e-5
REF_ALTITUDE_KM = 400.0
REF_DENSITY_KG_M3 = 3.725e-12
SCALE_HEIGHT_KM = 58.515


@dataclass(frozen=True)
class ForceModel:
    j2: bool = True
    drag: bool = False
    mass_kg: float = 750.0
    area_m2: float = 5.0
    cd: float = 2.2
    density_at_400_km_kg_m3: float = REF_DENSITY_KG_M3
    scale_height_km: float = SCALE_HEIGHT_KM

    def validate(self):
        values = (self.mass_kg, self.area_m2, self.cd,
                  self.density_at_400_km_kg_m3, self.scale_height_km)
        if not all(np.isfinite(x) for x in values):
            raise ValueError('Force-model quantities must be finite')
        if self.mass_kg <= 0 or self.area_m2 < 0 or self.cd <= 0 or self.scale_height_km <= 0 or self.density_at_400_km_kg_m3 < 0:
            raise ValueError('Positive mass, CD and scale height; nonnegative area and density required')


def j2_acceleration(r_km: np.ndarray) -> np.ndarray:
    """Cartesian acceleration from Earth zonal harmonic J2, km/s²."""
    r = np.asarray(r_km, dtype=float)
    rad = np.linalg.norm(r)
    if rad <= RE / 2: raise ValueError('Invalid orbital radius for J2')
    z2_over_r2 = (r[2] / rad) ** 2
    factor = 1.5 * EARTH_J2 * MU * RE**2 / rad**5
    return factor * r * np.array([5*z2_over_r2-1, 5*z2_over_r2-1, 5*z2_over_r2-3])


def atmosphere_density(altitude_km: float, m: ForceModel) -> float:
    """Single exponential around 400 km; hard-capped at low altitude.

    The cap prevents math overflow near reentry. This extrapolation is not
    a reliable 0–200 km atmosphere.
    """
    exponent = np.clip(-(altitude_km - REF_ALTITUDE_KM)/m.scale_height_km, -100, 40)
    return float(m.density_at_400_km_kg_m3 * exp(float(exponent)))


def drag_acceleration(r_km: np.ndarray, v_km_s: np.ndarray, m: ForceModel) -> np.ndarray:
    """Cd A rho v_rel |v_rel| /(2m) opposing atmosphere-relative velocity."""
    if not m.drag or m.area_m2 == 0 or m.density_at_400_km_kg_m3 == 0:
        return np.zeros(3)
    omega = np.array([0., 0., EARTH_SPIN_RAD_S])
    rel_km_s = np.asarray(v_km_s) - np.cross(omega, r_km)
    rel_m_s = rel_km_s * 1000
    rho = atmosphere_density(float(np.linalg.norm(r_km) - RE), m)
    # Convert acceleration (m/s²) -> km/s²
    return -0.5 * rho * m.cd * m.area_m2 / m.mass_kg * np.linalg.norm(rel_m_s) * rel_m_s / 1000


def acceleration(r: np.ndarray, v: np.ndarray, m: ForceModel) -> np.ndarray:
    rad = np.linalg.norm(r)
    if rad <= 0: raise ValueError('Singular gravity')
    a = -MU*r/rad**3
    if m.j2: a += j2_acceleration(r)
    if m.drag: a += drag_acceleration(r, v, m)
    return a


def j2_specific_potential(r: np.ndarray, enabled: bool) -> float:
    """Potential U=-mu/r * [1 - J2 (Re/r)^2 P2(z/r)], km²/s²."""
    rad = np.linalg.norm(r)
    if not enabled: return -MU/rad
    P2 = (3*(r[2]/rad)**2-1)/2
    return -MU/rad * (1 - EARTH_J2*(RE/rad)**2*P2)


def conserved_values(state: np.ndarray, m: ForceModel) -> tuple[float, float]:
    r,v = state[:3],state[3:6]
    return (float(np.dot(v,v)/2+j2_specific_potential(r,m.j2)),
            float(np.cross(r,v)[2] if m.j2 else np.linalg.norm(np.cross(r,v))))


def derivative(y: np.ndarray, m: ForceModel) -> np.ndarray:
    return np.r_[y[3:6],acceleration(y[:3],y[3:6],m)]


def rk4(y: np.ndarray, dt: float, m: ForceModel) -> np.ndarray:
    k1=derivative(y,m)
    k2=derivative(y+dt*k1/2,m)
    k3=derivative(y+dt*k2/2,m)
    k4=derivative(y+dt*k3,m)
    return y+dt*(k1+2*k2+2*k3+k4)/6


@dataclass
class PerturbedResult:
    time_s: np.ndarray
    states: np.ndarray
    impact: bool
    step_s: float
    requested_duration_s: float


def propagate(initial: np.ndarray, duration_s: float, samples: int, m: ForceModel) -> PerturbedResult:
    m.validate()
    y = np.asarray(initial,dtype=float).copy()
    if y.shape!=(6,) or not np.all(np.isfinite(y)) or np.linalg.norm(y[:3])<=RE:
        raise ValueError('Initial state must be six finite components above the Earth')
    if not np.isfinite(duration_s) or not 0<duration_s<=7*86400 or not 10<=samples<=4000:
        raise ValueError('Duration must be <= 7 days with 10–4000 samples')
    times=np.linspace(0,duration_s,samples+1)
    states=[y.copy()]
    impact=False
    for i in range(samples):
        interval = times[i+1]-times[i]
        substeps=max(1,ceil(interval/25.0))
        for _ in range(substeps):
            y=rk4(y,interval/substeps,m)
            if not np.all(np.isfinite(y)):
                raise ValueError('Integration diverged')
            if np.linalg.norm(y[:3]) <= RE:
                impact=True
                break
        states.append(y.copy())
        if impact: break
    arr=np.asarray(states)
    return PerturbedResult(times[:len(arr)],arr,impact,float(duration_s/samples),duration_s)


def diagnostics(states: np.ndarray, m: ForceModel) -> dict:
    energy=np.array([conserved_values(y,m)[0] for y in states])
    axial_h=np.array([conserved_values(y,m)[1] for y in states])
    e_rel=(energy-energy[0])/max(abs(energy[0]),1e-12)
    h_rel=(axial_h-axial_h[0])/max(abs(axial_h[0]),1e-12)
    return {
        'relative_energy_drift':np.abs(e_rel).tolist() if not m.drag else [],
        'relative_angular_momentum_drift':np.abs(h_rel).tolist() if not m.drag else [],
        'max_relative_energy_drift':float(np.max(np.abs(e_rel))) if not m.drag else None,
        'max_relative_angular_momentum_drift':float(np.max(np.abs(h_rel))) if not m.drag else None,
        'relative_specific_energy_change':float(e_rel[-1]),
        'relative_angular_momentum_change':float(h_rel[-1]),
        'diagnostic_scope': 'energy (including J2 potential) and axial angular momentum' if m.j2 else 'Keplerian energy and angular-momentum magnitude',
        'conservation_applicable':not m.drag,
    }
