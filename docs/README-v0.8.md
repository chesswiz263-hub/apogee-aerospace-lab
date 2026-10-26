# APOGEE v0.8 — Autonomous Guidance & Navigation

**Aerospace Physics & Orbital Guidance Engineering Environment**

A local-first educational orbit-dynamics and mission-analysis laboratory. Version **0.8** preserves all v0.1–v0.7 features and adds **Guidance** as a twelfth simulation mode: a synthetic position-measurement **extended Kalman filter (EKF)**, a **six-state numerical state transition matrix**, and **bounded ideal-impulse trajectory correction**. The calculations come from Python, not scripted orbital animations.

> **Research/education software only. Not flight software, safety-rated navigation, collision avoidance, docking guidance, certified orbital data, or an operational satellite tracker.** The navigation sensor, target reference, and execution assumptions in the new guidance mode are synthetic. Do not command actual hardware with these results.

## Quick start

1. Extract the ZIP to a writable location.
2. Install **Python 3.10+** if it is not already installed.
3. **Windows:** double-click `start_apogee_windows.bat`. **macOS/Linux:** run `bash start_apogee.sh` from the extracted project directory.
4. Once the console reports server startup, visit **http://127.0.0.1:8000**.
5. Select the **Guidance** tab (or use the existing Orbit, Transfer, Thrust lab, J₂ + Drag, Catalog, Targeting, Reference, Optimize, Finite burns, Uncertainty, or Validate modes).

The launcher will attempt to install Python requirements on first run. Network access is required for missing dependencies; the built-in orbital simulations work offline thereafter. The server binds only to `127.0.0.1`. API docs: `http://127.0.0.1:8000/docs`.

## v0.8: Guidance laboratory

**API:** `POST /api/guidance` (ready-made request: `examples/v08_guidance_request.json`). All request values are bounded. For example:

```json
{
  "altitude_km": 400,
  "inclination_deg": 51.6,
  "duration_minutes": 90,
  "measurement_interval_s": 90,
  "measurement_sigma_m": 30,
  "initial_position_error_m": 600,
  "initial_velocity_error_m_s": 0.4,
  "max_total_delta_v_m_s": 2.0,
  "guidance_enabled": true,
  "corrections": 3,
  "seed": 42,
  "j2_enabled": false
}
```

**What actually happens:**

1. Define a circular Earth-centered reference orbit and an actual **simulated** spacecraft trajectory starting with a radial position error and along-track velocity error. The two trajectories are propagated independently from the same assumed epoch.
2. Create seeded noisy **three-axis Cartesian position measurements** of simulated truth. Measurement noise is independent and isotropic Gaussian with the specified per-axis standard deviation. The filter is **never passed true spacecraft state**.
3. Predict the six-component state `[r_x,r_y,r_z,v_x,v_y,v_z]` and covariance between measurements using SciPy's DOP853; integrate a six-by-six variational state-transition matrix simultaneously. The dynamical Jacobian uses the analytical two-body gravity gradient, and, when enabled, numerically differentiated J₂ acceleration. No drag, measurement delay, bias, clock error, or model process noise is added.
4. Assimilate the synthetic position fixes with a nonlinear-dynamics **extended Kalman filter**: position observation matrix `H=[I₃ 0]`, covariance predicted by `P⁻=ΦP⁺Φᵀ`, and the Joseph-stabilized update `P⁺=(I−KH)P⁻(I−KH)ᵀ+KRKᵀ`. The initial covariance is an **assumed prior**, not empirically calibrated from a real vehicle.
5. At one to four scheduled times in the middle part of the mission, linearize the estimated-state propagated terminal error with the STM, solve a regularized three-component **velocity-impulse command**, and cap it by the unspent total Δv budget. The commanded impulses change both the simulated truth and filter prediction, with **perfect execution** assumed. Guidance knows the precomputed reference terminal state but not the hidden simulated truth.
6. Continue orbit integration through the requested end time. Output guided and unguided error curves, measurement innovations, filter uncertainty, maneuver commands, terminal position/velocity residuals, and a JSON/CSV report.

**Important:** This is estimated-state terminal tracking under ideal instantaneous velocity increments, not finite-thrust optimal control, full 6-DOF GNC, or a docking/rendezvous controller. The optimizer is one-step weighted linearized least squares, **not a global minimum-fuel solution**. Sensor errors alone can prompt unnecessary corrections; commanding perfect burns and having exact force-model knowledge are optimistic assumptions.

### Default deterministic example

| Metric | Simulated result |
|---|---:|
| Initial radial position error | 600 m |
| Initial tangential velocity error | 0.40 m/s |
| Sensor noise | 30 m 1σ per Cartesian axis |
| Measurements | 60 over 90 min |
| Final **unguided** position error | **17,945.24 m** |
| Final **guided** position error | **62.48 m** |
| Simulated position error reduction | **99.65%** |
| Number of correction commands | **3** |
| Sum of executed ideal Δv | **1.5711 m/s** |
| Final state-estimation position error | **4.91 m** |
| Final estimated position 1σ RSS | **12.53 m** |

These numbers apply only to this initial error, seed, idealized model and measurement settings. They should never be described as a validated spacecraft navigation accuracy or a general controller guarantee.

### How to verify the feedback effect

- Run Guidance with the default settings. Inspect the **guided (cyan)** and **unguided (orange)** position-error chart. Read the three maneuver commands in the analysis panel.
- Uncheck **Enable active guidance corrections** and run again. With identical initial truth and measurement seed, executed Δv becomes 0, and the guided-trajectory field matches the unguided baseline.
- Try **Include J₂** and vary sensor noise, initial errors, measurement cadence, or the total Δv budget. Changing the seed regenerates noise.
- Export the generated JSON to inspect the EKF covariance and time history. CSV exports the primary propagated trajectory.

The guidance trajectory and reference are both rendered in the same *idealized* inertial axes. These are **not authenticated ICRF, TEME, or Earth-fixed ephemerides**. Do not directly combine them with imported real satellite positions without epoch-accurate frame transformations.

## Verification

```bash
python -m pip install -e '.[dev]'
python -m pytest -q tests
python -m apogee.v08_benchmark
python -m apogee.v07_benchmark
```

- **149/149 automated tests passed in the build environment**: 125 inherited regressions, plus 24 v0.8 checks. New checks cover EKF Joseph covariance PSD and symmetry, finite-difference vs integrated-STM sensitivity, seed reproducibility, disabled/zero-budget guidance behavior, maneuver budget, noisy measurements, J₂ propagation, invalid requests, and the live FastAPI test client.
- An additional **10-seed synthetic measurement test** kept total Δv below 2 m/s, with a **34.26 m median** and **108.58 m worst** guided final position error for that restricted set of initial states and noise realizations. This is not a statistically validated reliability or success-rate estimate.
- A **standalone fixed-step RK4** implementation and DOP853 agreed at the reference orbit's terminal position to approximately **0.000028 m** over 90 minutes with 2-second RK4 steps. This tests numerical integration agreement for the same idealized equations, not physical truth.
- New guidance desktop/mobile checks ran in an **isolated browser** using genuine Python-generated API responses, validating rendering, tab switching, chart/metrics and the guidance-off control without JS exceptions or mobile overflow. The build environment blocked Chromium's direct HTTP navigation to the local server; a networked full-browser walkthrough remains an on-device check.
- Separate FastAPI tests exercise the actual `/api/guidance` route. Existing optional SGP4/Astropy integrations and live JPL/CelesTrak retrieval were **not** newly verified in this release.

## Technical architecture

| Component | Location |
|---|---|
| 6-state guidance EKF and impulsive controller | `apogee/guidance.py` |
| Reproducible noise-seed/independent propagator benchmark | `apogee/v08_benchmark.py` |
| Versioned REST endpoint and request constraints | `apogee/api.py` |
| Mission-control form, overlay, telemetry, plot | `web/index.html`, `web/app.js`, `web/style.css` |
| Regression tests | `tests/test_v08.py` and all previous tests |
| Synthetic example request, no-guidance control | `examples/v08_*.json` |
| Actual offline browser screenshots | `docs/v08-guidance-*.png` |
| Detailed previous-version documentation | `docs/README-v0.7.md` |

**Units:** inertial state km/km s⁻¹; simulation time seconds; measurement errors meters; maneuver impulses m/s; EKF covariance mixed Cartesian state units. **Dependencies:** Python, NumPy, SciPy, FastAPI, Uvicorn and SGP4 (required by existing catalog feature). Astropy is optional for some reference-frame features. No language-model subscription or cloud API key is required for v0.8 navigation.

## Known limitations and next research directions

The current navigation demonstrations have no actual GNSS/ground-station sensor geometry, receiver clocks, atmospheric modeling uncertainty, gyros, real-time filtering latency, attitude constraints, thrust-to-force coupling, impulsive-execution error, fuel-mass propagation from the feedback burns, persistent actuator dynamics, third-body gravity, process noise, or collision safety requirements. An EKF can be statistically inconsistent if the assumptions are wrong. `seed=42` is only a convenient reproducibility default.

Potential v0.9 milestones: **finite-duration closed-loop correction with engine/attitude constraints**, process noise and realistic measurement geometry, Monte Carlo controller robustness studies, and a stronger demonstration of filter consistency via innovation/NEES statistics. These require further physics and independent validation.

**Distribution:** Choose an appropriate license before publishing the project; currently no license is granted automatically by this archive.
