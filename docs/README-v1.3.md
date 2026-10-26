# APOGEE v1.3 — Integrated Navigation & Flight Systems

**Aerospace Physics & Orbital Guidance Engineering Environment** — a local-first, numerical spacecraft mission design and flight dynamics laboratory. **17 interactive modes**, an independently written Python physics engine, a FastAPI backend, and a dependency-free browser visualization.

> **Research/education only, NOT flight software.** The reference axes are idealized Earth-centered inertial axes, not certified ICRF, GCRS, TEME or terrestrial frames. Earth orientation, magnetic field, and Sun models remain simplified. The new navigation filter is **loosely coupled**; it is **not** a fully coupled 12-state joint attitude/orbit Kalman filter. Navigation error statistics are synthetic and must not be interpreted as spacecraft hardware performance.

## Getting started (Windows)

1. Extract the **entire** `APOGEE-v1.3.zip` archive into its own directory.
2. Run `start_apogee_windows.bat`. This installs free Python dependencies if necessary; a Python 3.10+ installation is required.
3. Open <http://127.0.0.1:8000> once Uvicorn starts. Interactive API docs: <http://127.0.0.1:8000/docs>.
4. Select **Integrated Nav v1.3**, run the coast-only example, then change the `Burn duration` from `0` to `120` s to examine model mismatch.

Linux/macOS: `bash start_apogee.sh` (or install dependencies and run `python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000`). The offline simulation doesn't need accounts, an LLM or paid APIs.

## What's new in v1.3

APOGEE now reuses the **17-state orbit/attitude/reaction-wheel truth simulator** and the **six-state attitude-error MEKF** from v1.2, and adds a **six-state position/velocity extended Kalman filter (EKF)**. The translational EKF receives an attitude **estimate**, not hidden true spacecraft orientation, when predicting body-axis engine acceleration.

### Causal, time-aligned telemetry

Measurements are tagged with two distinct times: the **capture timestamp** and the **receipt timestamp**, both expressed in elapsed seconds from the same simulation origin. A measurement is not assimilated until it has been received. The estimator then **replays the received measurement history at original capture epochs**, propagating the posterior forward to the present. A packet received after the simulation ends remains pending; it is not smuggled into the final estimate.

The replay strategy is intentionally a **bounded offline demonstration**, not a production real-time fixed-lag smoother. It recomputes the state and covariance from the prior each time the receipt set changes; it is computationally inefficient for long missions and does not model network clock skew, simultaneous correlation, packet authentication, or sensor synchronization hardware.

Two synthetic measurement sources:

- **Cartesian inertial position fixes:** three noisy position coordinates in the exact same idealized simulator inertial frame, with user-configurable Gaussian measurement noise.
- **Hypothetical ground tracking:** geometric range and range rate to a fictional network of rotating stations, with idealized Earth spin, a visibility check, and a simplistic range-rate noise model. No atmosphere, relativity, UT1, ITRF station coordinates, real tracking network, or hardware clock model.

**Frame safety:** each packet carries `APOGEE_IDEAL_ECI` and the API strictly refuses other frame tags. This prevents silently mixing ICRF, TEME, or Earth-fixed vectors. A realistic GCRS/ITRF transformation with absolute epochs and Earth-orientation parameters is an explicitly **future** capability, not an implemented v1.3 feature.

### Estimator equations and assumptions

The translational state is `x = [r_I (km), v_I (km/s)]` with a 6×6 covariance. It propagates a point-mass gravity field, optional Earth J₂ perturbation and known nominal thrust when commanded. Thrust direction is computed using the estimated quaternion from the separate MEKF at the latest available attitude-sample time. Motor delay, uncertain thrust magnitude, gyro-to-engine misalignment calibration, and mass-estimate uncertainty are **not** represented.

An approximately linearized discrete transition uses `Phi = expm(A Δt)`, where `A` contains the central-gravity/J₂ position Jacobian. A simplified white-acceleration process covariance is added, and measurement updates use the Joseph covariance form plus PSD symmetrization. The attitude-to-translation cross-covariance is **not modeled**. When the attitude estimate or its time sampling is poor during a burn, the orbital state error may dramatically exceed the translational EKF's covariance. APOGEE deliberately displays that inconsistency.

The attitude MEKF still estimates 3 attitude-error and 3 gyro-bias states, using synthetic gyros, intermittent tracker quaternions, exclusion cones, and magnetic wheel unloading. This means **6 + 6 separate estimation-error states**, not a single cross-coupled 12×12 covariance.

### Numerical demonstration (seed 27)

Scenario: 400 km initial altitude, inclination 51.6°, 18-minute duration, 72 output intervals (15 s), Cartesian position measurement period 60 s, 30 m independent per-axis position noise, 45 s telemetry latency, initial position offset 600 m, initial along-track velocity offset 0.3 m/s, Earth J₂ enabled. An MEKF supplies propulsion attitude estimates.

| Metric | Coast-only reference | With a 120 s, 850 N commanded body-axis burn |
|---|---:|---:|
| Final position estimation error | **19.73 m** | **149.68 m** |
| Final reported 1σ position RSS | **26.39 m** | **26.39 m** |
| Final attitude estimation error | **0.019°** | **0.060°** |
| Accepted Cartesian packets | **17 of 18** | **17 of 18** |

The final captured packet arrives **after** mission end because of the configured latency. The powered example demonstrates under-modeled translation uncertainty: **149.68 m actual simulated error vs 26.39 m reported 1σ**, so do not treat the covariance as calibrated under maneuvering. This is an intentionally visible model limitation, not evidence of guaranteed centimeter or meter accuracy in actual spaceflight.

Reproduce with:

```bash
python -m apogee.v13_benchmark
```

This generates a JSON benchmark in `docs/v13-benchmark.json` with both experiments.

### New API

`POST /api/navigation/integrated`

```json
{
  "altitude_km": 400,
  "inclination_deg": 51.6,
  "duration_minutes": 18,
  "samples": 72,
  "navigation_period_s": 60,
  "telemetry_latency_s": 45,
  "position_sigma_m": 30,
  "initial_position_offset_m": 600,
  "initial_velocity_offset_m_s": 0.3,
  "measurement_mode": "cartesian",
  "tracker_enabled": true,
  "burn_start_s": 300,
  "burn_duration_s": 0,
  "j2_enabled": true,
  "seed": 27,
  "reference_frame": "APOGEE_IDEAL_ECI"
}
```

The response includes all inherited v1.2 trajectory, attitude, MEKF, fuel and wheel telemetry, plus `integrated_navigation`: estimated translational trajectory, position and velocity error curves, per-time received-packet counts, covariance RSS, packet capture/receipt timestamps, summary metrics, frames and model assumptions. Invalid frame tags, grid intervals, burn schedules, noise parameters and other out-of-range values return 422.

### Files

| Path | Role |
|---|---|
| `apogee/integrated_nav.py` | Time-tagged measurement creation, causal event-time replay, J₂-informed Cartesian EKF, optional schematic ground range/rate sensor |
| `apogee/precision_adcs.py` | Inherited MEKF and coupled 17-state truth dynamics |
| `apogee/api.py` | All legacy API routes plus `/api/navigation/integrated` |
| `web/` | All 17 modes including new Integrated Nav v1.3 controls and error-vs-covariance chart |
| `tests/test_v13.py` | 14 new tests for packets, frames, causality, API, covariance Jacobian, burn mismatch and reproducibility |
| `apogee/v13_benchmark.py` | Reproducible coast-vs-burn example |

## Verification

```bash
python -m pip install -e '.[dev]'
python -m pytest -q tests
node --check web/app.js   # optional Node.js syntax check
```

**259/259 tests passed in five independent completed batches:** 105 long-standing orbital mechanics/API tests, 74 v0.7–v0.9 regression tests, 19 v1.0 tests, 47 v1.1–v1.2 tests, and 14 new v1.3 tests. The full uninterrupted suite was not completed in one run in the build environment; the individual complete batches did pass.

New mode visually rendered in isolated Chromium at **1440×900 desktop** and **390×844 mobile**, using real FastAPI-generated responses injected into an offline browser harness. Neither viewport showed horizontal overflow or browser JavaScript exceptions. The browser does not allow local HTTP navigation in this environment, so a **fully networked browser-to-localhost walkthrough** and live toggling through the powered mode still need testing on your computer. The powered case was separately exercised and tested through Python and FastAPI.

## Limits and next milestone

APOGEE is not operational GN&C software. No GCRS/ITRF epoch and Earth orientation validation, hardware-in-the-loop bridge, sensor clock modeling, cross-correlated joint EKF, external orbit determination, real Sun vectors, IGRF magnetics, flight data benchmarking, flight processor timing, or hardware calibration has been verified. Source-coded models must not be used to navigate actual spacecraft.

**Proposed v1.4:** introduce vetted absolute-time frame transforms using Astropy/IERS, correctly sourced solar ephemerides and magnetic field, improve gyro/attitude time alignment and uncertainty coupling, then undertake external truth-model comparisons before calling any navigation figures accurate.
