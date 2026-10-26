# APOGEE v1.6 — Navigation Integrity & Fault Detection

**Aerospace Physics & Orbital Guidance Engineering Environment** is an educational spacecraft numerical-analysis environment with a Python simulation engine, FastAPI services, a browser dashboard, and automated regression tests. v1.6 adds **Navigation Integrity**, bringing the existing 19 dashboard modes to **20**. All previous modes, including the v1.5 12-state coupled orbit/attitude EKF, remain available.

> **Research/educational model—not flight software.** Simulated sensors, inferred fault alarms, engine uncertainty, orbital dynamics, and models are synthetic. A failed innovation gate can signal sensor error, model inconsistency, or a poorly calibrated covariance. It is not proof of a hardware fault or a quantified safety guarantee.

![Integrity desktop preview](docs/v16-desktop.png)

## Launch on Windows

1. Extract the **whole** ZIP to a directory.
2. Install Python 3.11+ if needed and double-click `start_apogee_windows.bat`.
3. Wait for local dependencies and the web server to initialize.
4. Open `http://127.0.0.1:8000`.
5. Choose **Integrity v1.6** in the mission designer. Other modes remain available.

On macOS/Linux, from the extracted folder:

```bash
python -m pip install -r requirements.txt
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
```

Optional astronomical reference integration from v1.4 uses `requirements-reference.txt` separately.

## v1.6: navigation integrity experiment

The new scenario is a **standalone six-state position/velocity EKF** (it is not a claim that the v1.5 12-state filter has acquired fault isolation). Its differential equations use Earth point gravity, optional Earth J₂, and a prograde finite-duration engine. It assumes an initial wet mass of 750 kg, an 850 N engine, and 315 s specific impulse. The model's slowly varying engine thrust changes spacecraft mass analytically during the firing interval. The **true** engine-scale factor for each scenario is sampled from a user-specified normal calibration distribution and kept hidden from the estimator, which assumes nominal thrust.

- **Truth** is propagated with SciPy DOP853, split at the exact engine start and stop times.
- **State estimate** is propagated with an independently implemented RK4 integrator and a six-state covariance model. The EKF linearization is propagated by a matrix exponential, and covariance updates use Joseph form.
- **Measurements** are synthetic position fixes in APOGEE's explicitly labeled ideal Earth-centered inertial axes, `APOGEE_IDEAL_ECI`. Sensor error uses a stationary three-axis AR(1) process of marginal standard deviation σ and lag-one correlation ρ, generated with a seeded random-number generator.
- A **sustained sensor fault** is an added positive inertial-X bias starting at the specified simulation time. It is deliberately simplistic and does not stand for a particular real sensor failure.
- The EKF is intentionally configured to **assume independent** Gaussian position fixes with measurement covariance `σ² I`. The discrepancy is exposed in the innovation and final-error diagnostics.
- The **normalized innovation squared** (NIS) is `νᵀ S⁻¹ ν` with three degrees of freedom, compared to the nominal 99.73% chi-square threshold of approximately 14.16. Optionally reject measurements whose score exceeds that gate.
- Monte Carlo runs use deterministic seeds `seed + run_index`. Reports include detection fraction for the injected fault, alarm fraction before the fault, final position-estimation error, estimated 1σ RSS position uncertainty, and sampled engine calibration error.

**Critical statistical interpretation:** Under correct white Gaussian innovations, a 99.73% single-step chi-square gate has a nominal per-update exceedance rate of 0.27%. This rate is **not guaranteed** when observations are correlated, the engine model is mismatched, or the state covariance is inconsistent. Alert rate before the injected fault is **not necessarily a sensor false-positive rate**: an engine mismatch can be a legitimate cause for an innovation alarm.

No automatic spacecraft guidance response, maneuver abort, real tracking station, Earth orientation, calibrated reliability probability, or multi-sensor FDIR decision is implemented by the v1.6 laboratory.

## Run the new API

`POST /api/navigation/integrity` accepts for example:

```json
{
  "altitude_km": 400,
  "duration_minutes": 12,
  "samples": 48,
  "measurement_sigma_m": 30,
  "correlation": 0.65,
  "fault_start_s": 360,
  "fault_magnitude_m": 220,
  "thrust_error_pct": 3,
  "reject_faults": true,
  "monte_carlo_runs": 12,
  "seed": 29,
  "reference_frame": "APOGEE_IDEAL_ECI"
}
```

The result contains `trajectory` (the propagated truth), `integrity` (the nominal EKF error, estimated uncertainty, innovation scores, alarm flags, and Monte Carlo summary), `model`, `input`, and `warnings`. Field names and units are documented in the source module `apogee/integrity.py`. Reference-frame mismatches and out-of-range parameters are rejected rather than guessed.

## Reproducible benchmark

```bash
python -m apogee.v16_benchmark docs/v16-benchmark.json
python -m pytest -q tests/test_v16.py
python -m pytest -q tests
```

`docs/v16-benchmark.json` includes five 12-seed synthetic cases. The measured **median final position-estimation errors** for the checked-in seed set are:

| Scenario | Median final error |
|---|---:|
| Independent measurement noise, no injected fault or engine mismatch | 11.89 m |
| Time-correlated measurement noise only | 23.48 m |
| Sensor fault only, no engine mismatch | 93.85 m |
| Engine calibration uncertainty only | 1,570.76 m |
| Correlated noise, sensor fault, and engine calibration uncertainty | 1,570.76 m |

The identical rounded medians in the last two cases are the outputs of this particular gated estimator; they are **not** evidence that the injected sensor fault has no effect. When engine mismatch already drives the innovation scores above the threshold, the filter may reject many measurements. Do not interpret the large errors as reliable error bounds or compare them against real spacecraft navigation performance.

For the first default combined-stress run, final position-estimation error is approximately **588.8 m** versus an estimated 1σ RSS of **56.7 m**. This inconsistency is explicitly reported.

## Validation and limitations

- v1.6 adds **31 new tests**. All **322** tests passed in three completed batches during the release build: 180 + 70 + 72. The old tests' health-version assertions were updated to 1.6.0, while their computational and API checks were preserved. An additional uninterrupted 322-test command exceeded this environment's execution limit before its final summary, so only the completed grouped runs are claimed as passing.
- The new suite checks seeded reproducibility, AR(1) stationary variance and correlation, exact chi-square gate threshold, deterministic null-fault behavior, fault detection in a synthetic large-bias case, covariance positive semidefiniteness, API rejection rules, and an RK4/DOP853 agreement test on a coast trajectory.
- Chromium desktop/mobile **isolated rendering** was verified with real backend-generated responses. The form was rerun after disabling the injected fault, with no JavaScript exceptions or horizontal overflow. A *fully connected* browser-to-local-server walkthrough was blocked by browser networking restrictions in the build environment and remains an on-device check.
- The base program's optional Astropy and IGRF integrations and live CelesTrak satellite fetching are preserved but were not independently verified in this build.
- A numerical 6-state EKF with realistic statistical assumptions is **not** the same as a certified fault-detection/isolation/recovery architecture. Large Monte Carlo campaigns, trajectory independence studies, multi-sensor validation, process-noise calibration, delayed-data analysis, and real flight references remain future work.

Previous release documentation: [`docs/README-v1.5.md`](docs/README-v1.5.md).

**APOGEE is a reproducible engineering learning environment, not a flight-qualified guidance, navigation, or attitude-control system.**
