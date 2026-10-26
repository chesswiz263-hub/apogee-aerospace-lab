# APOGEE v0.9 — Robust Autonomous Guidance

**Aerospace Physics & Orbital Guidance Engineering Environment**

APOGEE is a local-first, educational Python numerical spacecraft simulator with a browser mission-control dashboard. Version **0.9** preserves the 12 v0.8 modes and adds a **13th mode: Robust GNC**, which integrates finite-duration correction burns, geometric ground-station tracking, and filter/dynamics uncertainty experiments.

> **For research and education only. NEVER use APOGEE to operate a spacecraft, issue collision-avoidance maneuvers, navigate a real spacecraft, plan docking, or certify flight hardware.** The ground stations are *invented*, the models are simplified, and numerical consistency is not equivalent to real-world validity.

## Quick start

- Windows: extract the ZIP, then double-click `start_apogee_windows.bat` (requires Python 3.10+ and access to install dependencies when missing).
- Linux/macOS: `bash start_apogee.sh`.
- Open **http://127.0.0.1:8000**, choose **Robust GNC**, then run the default scenario. The Python simulation and dashboard work offline after dependencies are installed.
- API documentation: **http://127.0.0.1:8000/docs**. Server binds to localhost only.

No language-model service or subscription is needed. The simulations run locally using NumPy, SciPy, FastAPI, and the existing project's supporting packages.

## v0.9 capabilities

### Finite feedback burns instead of instantaneous impulses

At each guidance decision, the estimator predicts terminal state and solves a regularized, bounded, one-step **impulsive** targeting problem. The new actuator then maps the commanded Δv to a **finite burn duration** using the rocket equation:

\[
\Delta v = I_{sp} g_0\ln(m_0/m_f),\qquad
\dot m=-F/(I_{sp}g_0),\qquad
\ddot{\mathbf r}=\mathbf a_{gravity}+\frac{F}{1000m}\widehat{\mathbf u}
\]

Here inertial position is in km, time in seconds, thrust in newtons, and mass in kg. During the burn, DOP853 solves the 7-state position/velocity/mass equations. The direction is **held fixed in the inertial frame**, not continuously attitude-steered. After each burn, the filter predicts motion with nominal thrust, whereas the hidden truth uses a seeded thrust-magnitude dispersion and pointing error. The dry-mass boundary is enforced, including when the true engine is stronger than nominal.

**Important distinction:** the trajectory-targeting step still assumes an ideal impulse when choosing direction. The commanded correction is *executed* as finite thrust, with residual terminal errors reported. This is not finite-horizon optimal control or globally minimum-fuel trajectory optimization. The **nominal Δv budget** is enforced on nominal commanded burn execution; because thrust is uncertain, **actual integrated engine Δv can exceed the nominal budget**. Both values are shown.

### Simulated ground-station range and range-rate tracking

Ground-station positions rotate with a spherical Earth at a constant spin rate. An observation is accepted only above the selected geometric elevation mask (default 5°). The simulated station measures:

\[
\rho=\lVert\mathbf r-\mathbf r_s\rVert,\qquad
\dot\rho=\frac{(\mathbf r-\mathbf r_s)\cdot(\mathbf v-\mathbf v_s)}{\rho}.
\]

The EKF uses an analytical 2×6 measurement Jacobian, seeded Gaussian range and range-rate noise, propagated six-state sensitivities, and the Joseph covariance update. It **never reads hidden true state** to choose a maneuver. The simulator internally uses truth to synthesize an observation and decide physical station visibility, as a sensor simulator necessarily would.

Three fixed, fictional global networks are provided: 3, 9, or 18 stations. **These coordinates do not represent real antennas**, ground assets or authentic Earth-fixed frame transformations; there is no real station epoch/UT1, precision geodesy, signal light-time, receiver clock, atmospheric delay, multipath, bias, outage, or antenna scheduling.

### Robustness and consistency diagnostics

- Independent truth and filter Earth-J₂ switches. This exposes force-model mismatch instead of quietly assuming the navigation software knows all physical forces.
- Covariance inflation from an approximate interval-wise acceleration-disturbance model. **Process noise changes the estimator covariance, not the underlying simulated truth trajectory.** It is not a complete stochastic forcing or calibrated continuous-time process model.
- Normalized innovation squared (**NIS**) per accepted 2D measurement and final normalized estimation error squared (**NEES**) for the 6D state. Final NEES uses hidden truth **only after simulation**, as a performance diagnostic—not for navigation decisions.
- Time histories of guided vs uncorrected position error, filtered-state uncertainty, fuel consumption, nominal vs actual delivered engine Δv, burn start/end times, and sensor visibility.
- A bounded **2–10-seed ensemble** comparing final errors. This is a sensitivity demonstration with identical injected initial errors—not an operational reliability or collision probability estimator.

## Reproducible default experiment

`POST /api/robust/guidance`, with `examples/v09_robust_default.json` or the prefilled **Robust GNC** tab:

| Setting | Value |
|---|---:|
| Reference Earth-orbit altitude / inclination | 400 km / 51.6° |
| Time / observation interval | 90 min / 90 s |
| Initial radial position / along-track speed errors | 600 m / 0.4 m/s |
| Ground stations / geometric elevation mask | 18 hypothetical stations / 5° |
| Range / range-rate 1σ measurement noise | 15 m / 0.03 m/s |
| Mass at start / engine thrust / Isp | 750 kg / 850 N / 315 s |
| Nominal correction Δv budget | 2 m/s |
| Engine thrust / pointing 1σ dispersions | 2% / 0.25° |
| J₂ in truth / filter | On / On |
| Seed | 42 |

**Results for that reproducible simulated case:**

| Metric | Result |
|---|---:|
| Final unguided position error | **17,923.54 m** |
| Final guided position error | **31.70 m** |
| Position error reduction | **99.82%** for this seed |
| Visible range/rate measurements | **22** |
| Finite correction burn count | **3** |
| Nominal executed cumulative Δv | **2.0000 m/s** |
| Actual integrated engine Δv | **2.0341 m/s** |
| Consumed propellant | **0.49368 kg** |
| Final state-estimation position error | **6.71 m** |
| Final estimated position 1σ RSS | **8.38 m** |
| Mean measurement NIS (2D) | **1.267** |
| Final state NEES (6D) | **33.72** |

The final NEES of 33.72 is **not** a reassuring consistency result for a nominally 6-dimensional error; it illustrates that the chosen covariance model can be overconfident/miscalibrated. The NIS and NEES are not mission-validated acceptance tests.

### Sensitivity experiments: the same defaults, varied conditions

- **10 seeds, 42–51**: median guided final error **66.70 m**, worst **323.92 m**, 10 of 10 lower than unguided in *this limited synthetic sample*. This must not be extrapolated into a success rate or safety guarantee.
- **3 hypothetical stations**, same seed: guided final error approximately **1,430 m**, with fewer measurements.
- **Truth J₂ on, filter J₂ off**, same seed: final guided error approximately **13.80 km**—a conspicuous model-mismatch failure. The planner offers no global convergence guarantee.
- **Guidance disabled**: zero burns, zero fuel use and exactly the uncorrected trajectory within numerical roundoff.
- **Only 10 grams initial fuel**, high thrust dispersion: the software caps burn duration to keep wet mass at or above dry mass; the resulting orbital error remains large.

These scenarios use idealized shared inertial axes rather than authenticated Earth-centered ICRF/TEME or Earth-orientation data. No actual spacecraft or independent flight telemetry have been compared.

## HTTP API & reproducibility

| Method / route | Purpose |
|---|---|
| `POST /api/robust/guidance` | Complete synthetic single-run finite-burn guidance, EKF and telemetry |
| `POST /api/robust/sweep` | 2–10 consecutive seeded runs and summary of simulated outcomes |
| `POST /api/guidance` | Previous v0.8 ideal-impulse, Cartesian-position EKF (still available) |
| `GET /api/health` | Version and optional dependency status |
| `GET /docs` | FastAPI interactive API reference |

The existing v0.1–v0.8 APIs and mission modes remain intact.

### Run tests and benchmarks

```bash
python -m pip install -e '.[dev]'
python -m pytest -q tests
python -m apogee.v09_benchmark > v09-benchmark.json
python -m apogee.v08_benchmark
```

**179/179 automated tests pass locally** (149 inherited regressions, 30 v0.9 tests). The new suite validates station rotation, visibility, range-rate as a numerical derivative, finite-difference observation Jacobians, Joseph covariance symmetry/PSD, mass flow, rocket-equation duration, STM-integrated burns, deterministic noise, force-model mismatch, low/zero propellant, nominal Δv budget, API rejection paths and seeded ensembles.

The desktop/mobile dashboard was inspected in a local isolated Chromium renderer using genuine Python API results and a safe mocked network layer. The new form, chart, event markers, analysis rows, seed-sweep response, and viewport layout render without JavaScript errors or horizontal overflow. **The sandbox blocked full Chromium navigation to localhost**, so a full browser-to-server network walkthrough still needs to be run on your machine. Live CelesTrak/JPL retrieval, Astropy and SGP4 optional features are not newly reverified.

## Structure

- `apogee/robust_guidance.py` — new geometry, EKF, finite engine burns, robustness sweep.
- `apogee/v09_benchmark.py` — reproducible seeded experiments.
- `apogee/api.py` — two new bounded REST endpoints, version bump.
- `web/` — 13-mode mission-control interface, Robust GNC form and ensemble button.
- `tests/test_v09.py` — new regression tests.
- `examples/v09_*.json` — requests.
- `docs/README-v0.8.md` — previous-version technical documentation.
- `start_apogee_windows.bat`, `start_apogee.sh` — local launchers.

## Scope and next work

The current model excludes authentic station ephemerides, Earth-orientation parameters, drag, third-body gravity, general orbit determination, ionospheric/tropospheric correction, clock biases, signal light-time, attitude state, thruster vector dynamics, response delays and formal reachability/safety proofs. Process uncertainty is not a tuned continuous spectral-density model, and maneuver timing is not fully optimized. **Even if the tests pass, this simulation should not be treated as a validated operational guidance system.**

Candidate v1.0 research milestone: model sensors with real station geometry and frame/time-scale handling, add process-noise calibration and innovation consistency analysis across broader ensembles, and target full 6-DOF spacecraft attitude/actuator coupling. Validate numerical propagation against trusted external ephemerides before asserting real-world accuracy.

**Licensing:** Choose an explicit license before publishing; this archive does not automatically grant one.
