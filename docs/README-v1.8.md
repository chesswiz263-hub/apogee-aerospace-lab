# APOGEE v1.8 — Adaptive Navigation & Autonomous Fault Recovery Experiments

**Aerospace Physics & Orbital Guidance Engineering Environment.** A local, interactive simulator of orbital dynamics, propulsion, targeting, attitude determination and control, and spacecraft navigation. Version 1.8 adds the **Adaptive Nav** experiment as the 22nd operating mode; all 21 previous v1.7 modes and their APIs remain available.

> **Important: this is educational / research simulation software, NOT flight-certified GN&C or autonomous recovery software.** The measurements and ground truth are generated synthetically. No in-orbit performance, fault-detection probability, sensor calibration, physical actuator qualification, or safety guarantee follows from these examples. Model errors can masquerade as faulty sensors.

![APOGEE v1.8 actual desktop control screen](docs/v18-desktop.png)

## Install and launch

**Windows:** Extract the entire ZIP, then double-click `start_apogee_windows.bat`. Python 3.11+ is recommended. The launcher installs dependencies (internet access may be required on first launch) and starts the API/web interface on `127.0.0.1:8000`. Open [http://127.0.0.1:8000](http://127.0.0.1:8000), and select **Adaptive Nav v1.8**.

**macOS / Linux:**

```bash
python -m pip install -r requirements.txt
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
```

Other useful commands:

```bash
python -m pytest -q tests/test_v18.py
python -m pytest -q tests
python -m apogee.v18_benchmark docs/v18-benchmark.json
```

API documentation: `http://127.0.0.1:8000/docs`. Additional astronomical and geomagnetic backends remain optional and may need installation separately from `requirements-reference.txt`. The v1.8 adaptation demonstration does **not** require those optional packages.

## What v1.8 actually changes

The legacy v1.5/v1.7 joint navigation filter maintained **12 small-error states**: position (3), velocity (3), body attitude error (3), and gyro bias (3), with a nominal unit quaternion. v1.8 augments that joint **error covariance to 13 × 13** with **one scalar propulsion scale factor**. It is estimated through observed spacecraft motion during finite burns.

The filter reuses the v1.5 RK4 state-prediction and quaternion innovation update mathematics, but augments its covariance with thrust-scale sensitivity. The **actual** thrust used by the independently simulated attitude/propulsion truth trajectory is configured separately from the **assumed** engine scale in the estimator. The new UI can enable/disable calibration estimation to show whether that parameter is observable with the chosen measurements and burn schedule.

For the fixed engine architecture used here, when a burn is active, the linearized velocity-error sensitivity to the engine scale is approximately

\[
\frac{\partial \dot{\mathbf v}}{\partial \alpha}
=\frac{T_0}{m(t)}\, \mathbf R_{BI}(q)\, \mathbf e_x,
\]

where \(\alpha\) is the dimensionless engine scale, \(T_0\) is nominal thrust, \(\mathbf R_{BI}\) is the quaternion-derived body-to-inertial rotation, and \(m(t)\) is modeled wet mass. Units are converted to the simulator's kilometers and seconds internally. A first-order state-transition approximation couples scale covariance into position and velocity covariance. Parameter uncertainty is not magically eliminated by adding an extra state: it requires informative measurements during or after a burn.

### Navigation response policies

1. **Monitor:** calculate and record NIS alarms, but assimilate every observation.
2. **Gate:** reject each individual observation whose nominal 3-DoF chi-square normalized innovation exceeds its gate.
3. **Quarantine:** after a configured number of consecutive alarms in a channel, reject that observation and temporarily block further measurements from that channel. The channel may return after the time window. This is a **software experiment, not verified fault diagnosis or safe-mode logic**.

The two monitored channels are synthetic inertial Cartesian position fixes and synthetic quaternion star-tracker attitude measurements. Each has its own noise adaptation. Accepted observations can adjust the corresponding nominal measurement-noise multiplier with a bounded **exponentially weighted innovation heuristic**. The multiplier is bounded from 1× to 25×. It is deliberately labeled a *heuristic*, not a statistically calibrated adaptation algorithm: persistent errors may become harder to detect if the filter broadens its assumed noise.

### Sensor timeline integrity

Packets are marked with independent capture and receipt times. The estimator uses **only packets received by the current simulation time** and replays them at their original capture epochs. The current implementation replays from mission start for each displayed sample to make causal handling straightforward and auditable. This is computationally costly, and is not a production real-time fixed-lag smoother.

### True-model and covariance limitations

- The synthetic attitude-control dynamics, gyro and tracker models originate in APOGEE's older ADCS simulation. Engine calibration is a single *constant scalar*; no gimbals, valve timing, flow sensor, pressure or changing combustion efficiency are estimated.
- The physical truth uses changed engine thrust **and corresponding propellant flow**. The nominal estimator applies a scale factor to thrust acceleration but continues to use a simplified fixed reference mass history. Scale estimates may compensate for other model errors.
- Covariance propagation uses local first-order error dynamics with positive-semidefinite projection and simplified process noise. It is **not** a fully discretized nonlinear stochastic model and is **not guaranteed consistent** over untested scenarios.
- The original Earth frame remains `APOGEE_IDEAL_ECI`, not GCRS, TEME or ITRS. Incompatible frame labels are rejected. Synthetic Sun and magnetic references in older experiments are not real astronomy by default.
- The NIS gate thresholds assume a correctly specified Gaussian innovation covariance, but correlated noise, attitude mismatch, and engine calibration errors can invalidate that interpretation. **Alarm counts are not operational fault probabilities.**
- The positions, quaternions, and errors are compared against simulated truth for *post hoc* diagnostics only; truth is not provided as input to the filter's prediction, scoring, or measurement updates.

## New API: `POST /api/navigation/adaptive`

Example request:

```json
{
  "duration_minutes": 12,
  "samples": 48,
  "burn_start_s": 180,
  "burn_duration_s": 120,
  "actual_thrust_error_pct": 5,
  "engine_model_error_pct": 0,
  "estimate_thrust_scale": true,
  "measurement_response": "gate",
  "fault_channel": "none",
  "adaptive_noise": true,
  "sensor_correlation": 0.25,
  "monte_carlo_runs": 1,
  "seed": 29,
  "reference_frame": "APOGEE_IDEAL_ECI"
}
```

The response's `adaptive_navigation` object includes: final position/attitude error; estimated and actual synthetic thrust scale; its 1σ covariance; per-channel alarms, rejections, quarantine counts, updated sensor-noise multipliers; lists of timestamped packet decisions; and a seeded Monte Carlo summary. `position_error_m`, `position_1sigma_rss_m`, and `estimated_thrust_scale_history` are directly chartable series aligned with `time_s`. Returned `trajectory` and `attitude` describe the finite-burn simulation, not fabricated display positions.

## Reproducible v1.8 benchmark

The bundled [`docs/v18-benchmark.json`](docs/v18-benchmark.json) contains six fixed scenarios with **two synthetic seeds each**. Results below are median *final position-estimation* errors in meters; samples are far too small to support statistical performance guarantees.

| Synthetic experiment | Median final error | Interpretation |
|---|---:|---|
| Nominal, engine scale estimated | 43.34 m | Estimator can infer the assumed engine-scale offset in this example |
| Nominal, calibration estimation off | 2,971.58 m | Fixed incorrect engine model creates substantial navigation error |
| Position-sensor fault, individual gating | 426.68 m | Outlier rejection does not guarantee state recovery |
| Position-sensor fault, temporary quarantine | 426.68 m | In these seeds, temporary blocking does not improve final error |
| Star-tracker fault, individual gating | 43.24 m | This injected tracker fault is largely rejected in the example |
| 20% engine mismatch, adaptive estimation | 46.44 m | The estimator can absorb this *particular* modeled engine error |

Single-run default example: actual engine scale **1.0500**, estimated **1.0490**, final position-estimation error **57.67 m** and reported 1σ RSS position uncertainty **42.13 m**. Those uncertainty magnitudes are not independently calibrated; a 1σ discrepancy is not in itself proof of failure. The test with estimation disabled finishes much worse, emphasizing **the model's dependence on thrust-scale estimation in this specific setup**.

## Verification status

- **v1.8 dedicated automated tests:** 32 passing, including frame rejection, causal delayed packet processing, 13×13 covariance checks, offline thrust observability, fault response, API error handling, and seeded reproducibility.
- **Inherited regression tests:** complete results documented in `RELEASE_CHECKS.json`. Grouped execution is used because long single-command runs may exceed this environment's command duration limit.
- **Browser:** desktop and mobile offline Chromium tests use computed backend responses and test turning calibration estimation off; no JavaScript exceptions or horizontal overflow were observed. This is *not* a complete networked browser-to-API acceptance test.
- **External validation:** real satellite datasets, live SGP4/CelesTrak, optional Astropy/IGRF conversions, sensor calibration, and hardware-in-loop tests remain outstanding. The numerical tests validate model implementation—not flight accuracy.

## Project structure

- `apogee/adaptive_navigation.py` — new coupled engine-scale estimate, innovation adaptation, and response policies.
- `apogee/joint_navigation.py` — inherited 12-error-state navigation filter.
- `apogee/joint_integrity.py` — inherited separate 12-state integrity experiments.
- `apogee/precision_adcs.py` — simulated spacecraft attitude and finite-thrust truth integration.
- `apogee/api.py` — versioned FastAPI routes and input validation.
- `web/` — browser mission-control interface; trajectory data comes from the Python physics engine.
- `tests/test_v18.py` — dedicated regression tests.
- `examples/v18_adaptive_default.json` — example request payload.
- `apogee/v18_benchmark.py`, `docs/v18-benchmark.json` — reproducible study.

## Next milestone

**v1.9 — Navigation Observability & Robustness:** quantify observability of engine calibration under different burn profiles, test covariance consistency over much larger Monte Carlo ensembles, and compare more principled robust estimation methods against gate/quarantine heuristics. Do not upgrade this simulator to operational autonomy without verification against independent flight-dynamics and sensor-reference data.
