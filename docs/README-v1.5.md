# APOGEE v1.5 — Coupled Navigation & Uncertainty

**Aerospace Physics & Orbital Guidance Engineering Environment** — a Python numerical flight-dynamics simulator with a browser mission dashboard. This release extends v1.4 without removing any of its 18 previous modes. The new **Joint Nav v1.5** tab brings the total to **19 modes**.

> Research/educational software. The simulated sensor measurements and orbit truth are synthetic. It is not validated against operational ephemerides, spacecraft hardware, or flight-software qualification requirements.

## Start on Windows

1. Extract the **entire** `APOGEE-v1.5.zip` (not just the launcher).
2. Install Python 3.11+ if needed, and run `start_apogee_windows.bat`.
3. Open **http://127.0.0.1:8000** after the local FastAPI server starts.
4. Choose **Joint Nav v1.5** in Mission Designer, or use any earlier simulator mode.

The standard launcher installs the **core** requirements (`requirements.txt`: NumPy, SciPy, FastAPI, Uvicorn and SGP4). Absolute-time reference functions from v1.4 are **optional**, and can be enabled separately with `python -m pip install -r requirements-reference.txt` (Astropy and ppigrf). Their absence returns a clear error, not fictional astronomical values.

macOS/Linux:

```bash
python -m pip install -r requirements.txt
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
```

## What's new: one correlated 12-state navigation filter

The **nominal state** is a 13-component vector comprising position (3), velocity (3), scalar-first body-to-inertial quaternion (4), and angular-rate gyro bias (3). The **error state** is 12-dimensional:

\[
\delta x = [\delta r_I,\delta v_I,\delta\theta_B,\delta b_B]^T,
\qquad P\in\mathbb R^{12\times 12}.
\]

The 12×12 covariance includes orbital/attitude/gyro-bias cross-covariances. Position measurements and star-tracker quaternion measurements **both update the same covariance**, unlike v1.4's separate filters. The nominal attitude is advanced by synthetic gyroscope samples with a bias estimate; tracker updates inject a right-multiplicative quaternion correction and reset the attitude error covariance.

For the body +X engine axis, the acceleration is calculated from the **estimated quaternion**:

\[
\mathbf a_{\rm thrust}=\frac{T}{1000m}R(q)[1,0,0]^T
\quad(\mathrm{km/s^2}).
\]

For small right-multiplicative attitude error, the linearized orbital/attitude coupling is

\[
\frac{\partial\dot v}{\partial\delta\theta_B}
=-\frac{T}{1000m}R(q)[e_x]_\times.
\]

This block produces position–attitude covariance during powered flight. The implementation uses RK4 for each nominal translational prediction interval and a matrix exponential for the local error-state transition. Covariance updates use the Joseph stabilized form and a small-angle quaternion reset, followed by numerical symmetry/PSD projection.

**Sensor timestamps:** every synthetic Cartesian position or star-tracker packet has a `capture_s` and `receipt_s`. Packets are not assimilated until receipt. At each output time the filter is reconstructed chronologically using **only packets received so far**, replaying each at its capture epoch. This is an intentionally transparent complete replay algorithm; runtime grows quadratically with sample count. Star-tracker geometric visibility uses the existing v1.2 idealized fixed Sun and Earth-limb exclusion cones, not a real star catalog.

**Frames:** all states in this new mode remain in `APOGEE_IDEAL_ECI`. These axes are *not* implicitly GCRS, ITRS, TEME, or ICRF. The v1.4 absolute-time reference laboratory remains separate.

## API

`POST /api/navigation/joint`

Example six-minute request:

```json
{
  "altitude_km": 400,
  "duration_minutes": 6,
  "samples": 36,
  "navigation_period_s": 60,
  "tracker_period_s": 60,
  "telemetry_latency_s": 45,
  "tracker_latency_s": 5,
  "position_sigma_m": 30,
  "tracker_sigma_deg": 0.04,
  "burn_start_s": 90,
  "burn_duration_s": 90,
  "tracker_enabled": true,
  "j2_enabled": true,
  "reference_frame": "APOGEE_IDEAL_ECI",
  "seed": 27
}
```

Output includes `trajectory`, `attitude`, `joint_navigation` (time histories of position error, estimated 1σ RSS, attitude error, cross-correlation, packet counts, and bias estimates), input echo, model metadata, limitations, and warnings. The existing `POST /api/navigation/integrated` route is unchanged and is available for comparing loosely coupled v1.4 estimators against v1.5.

A new sensor mode intentionally supports **synthetic Cartesian position fixes**, not real range/range-rate ground-station data. Requests for incompatible frames or invalid sampling periods fail validation.

## Reproducible numerical experiment

Run:

```bash
python -m apogee.v15_benchmark
python -m pytest -q tests/test_v15.py
```

A JSON result is already included in `docs/v15-benchmark.json`, showing 12-minute coast, powered, and star-tracker-disabled cases plus a five-seed powered experiment.

With default test inputs (400 km altitude, 51.6° inclination, 120-second burn beginning at 300 s, 30 m Cartesian measurement noise, 45 s position packet latency, 0.04° star-tracker noise, seed 27):

| Diagnostic | Result |
|---|---:|
| Final position-estimation error | 51.31 m |
| Final position 1σ RSS reported by filter | 39.70 m |
| Final attitude-estimation error | 0.595° |
| Maximum position/attitude component correlation | 0.159 |
| Received / generated measurements | 22 / 24 |
| Five-seed median final position error | 56.34 m |
| Five-seed worst observed position error | 74.21 m |

These are outcomes for synthetic truth and assumptions, **not calibrated or guaranteed confidence intervals**. The final position error exceeds the reported 1σ RSS in the default case. Disabling the star tracker can significantly worsen the estimate. The v1.5 filter **does not cure real-world powered-flight navigation uncertainty**.

## Verification status

- **291 automated tests passed**, in four completed batches: 125 early/core physics/API tests; 73 v0.8–v1.0 tests; 70 v1.1–v1.4 tests; and 23 new v1.5 tests.
- New tests check analytical thrust-to-attitude linearization against finite differences, position-attitude correlation from actual powered propagation, positive semi-definite covariance, star-tracker cross-covariance updates, delayed-packet causality, frame rejection, and deterministic sensor noise.
- Desktop (1440 px) and mobile (390 px) **isolated Chromium** checks passed with backend-generated response fixtures. The UI test toggled the star tracker and reran the simulation without JavaScript errors or horizontal overflow.
- **Not verified in this environment:** a fully connected browser-to-local-HTTP server, real satellite/SGP4 reference propagation, and positive Astropy/IGRF integrations. Synthetic browser fixtures were not a substitute for live networking.
- Release ZIP integrity is independently checked during packaging.

## Scientific limitations and next work

1. The dynamical truth comes from APOGEE's own finite-burn ADCS propagator. Its control law, Sun cone, magnetic field and tracking observations are simplified.
2. Nominal thrust, mass flow and inertia/actuator assumptions are known to the estimator. Engine error, accelerometer scale errors, correlated clock effects, unmodeled external torques and colored gyro noise are **not jointly calibrated**.
3. The filter relies on a local small-angle approximation and discretized process noise. Positive-semidefinite covariance is a *numerical property*, not proof of statistical consistency.
4. Whole-history replay is deliberately simple and bounded to at most 120 simulation intervals, not an efficient online fixed-lag smoother.
5. Before calling this a precision-navigation tool, compare against separately generated reference trajectories and implement real station geometry, time standards, time-correlated sensor models, Monte Carlo consistency tests, and realistic engine calibration errors.

**APOGEE v1.5 is a reproducible computational laboratory, not operational spacecraft guidance software.**
