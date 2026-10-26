# APOGEE v1.9 — Navigation Observability & Robustness

**Aerospace Physics & Orbital Guidance Engineering Environment** · open-source-style educational mission analysis software with **23 operating modes** (the 22 inherited v1.8 modes plus Observability v1.9).

> **Research simulator, NOT flight software.** All navigation observations and faults here are synthetic. The v1.9 experiment is an **offline noncausal batch estimator**, **not** an upgrade to the v1.8 onboard-style 13-error-state joint attitude/navigation EKF. A converged batch fit is not a guarantee of online fault recovery, calibrated coverage, or real spacecraft performance.

![v1.9 Observability dashboard](docs/v19-desktop.png)

## Installation

**Windows:** Extract the ZIP and double-click `start_apogee_windows.bat`. The launcher chooses Python and installs the packages in `requirements.txt` (internet may be required initially). With the server running, open [http://127.0.0.1:8000](http://127.0.0.1:8000) and choose **Observability v1.9**.

**macOS / Linux:**

```bash
python -m pip install -r requirements.txt
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
```

**Validation:**

```bash
python -m pytest -q tests/test_v19.py
python -m pytest -q tests
python -m apogee.v19_benchmark docs/v19-benchmark.json
```

API documentation: `http://127.0.0.1:8000/docs`. API endpoint: `POST /api/navigation/observability`. The v1.9 endpoint requires only the normal package dependencies (no Astropy, SGP4 data download, or cloud API key).

## What is new?

* **Local thrust-scale observability:** a real numerical sensitivity matrix from a two-body, finite-duration-burn spacecraft model with changing mass. A measurement-only information calculation projects out all six nuisance initial-position and velocity parameters.
* **Robust batch fits:** compare three methods on **the same measurements in each random trial**: Gaussian weighted least squares, Huber residual downweighting, and threshold gating (0.02 residual weight for flagged observations).
* **Monte Carlo consistency:** measure median, 90th-percentile, and worst final fitted-position error, thrust-scale root-mean-square error, approximate Gaussian 95% coverage, and Wilson uncertainty intervals for empirical position coverage.
* **Failure demonstrations:** set the engine burn to zero (thrust scale becomes unobservable); start observing only late (confounding with unknown initial orbit), increase sensor correlation, or change the fault from a short outlier to a persistent bias. A persistent fault can defeat all three robust methods.
* **Retained previous modes:** no earlier API has been removed; v1.8 engine estimation, ADCS, maneuver optimization, guidance, and other laboratories remain in the project.

## Mathematical model (v1.9 only)

The nonlinear Earth-centered six-state motion is

\[
\dot{\mathbf r}=\mathbf v,\qquad
\dot{\mathbf v}=-\frac{\mu_E}{r^3}\mathbf r+
\mathbf 1_{[t_b,t_b+\Delta t_b)}\frac{\alpha T}{1000\,m(t)}\hat{\mathbf u},
\]

where **r** is in km, **v** in km/s, engine thrust **T** is 850 N, the assumed inertial thrust axis **u** is **fixed throughout the maneuver**, engine specific impulse is 315 s, and

\[
m(t)=750-\alpha\,\frac{T}{I_\mathrm{sp}g_0}
\operatorname{clip}(t-t_b,0,\Delta t_b)\ \mathrm{kg}.
\]

Burn durations are bounded at 360 s in this experiment to avoid propellant depletion for any allowed thrust scale. **These dynamics omit** J₂, drag, finite attitude steering, third bodies, solar pressure, orbital maneuvers to target a chosen destination, and realistic range/range-rate or GNSS sensor geometry. Those are available or approximated in other APOGEE modes, but **not** in this v1.9 observation fit.

The local error parameters are

\[
\boldsymbol\theta=[\delta\mathbf r_0\ (\mathrm m),\ \delta\mathbf v_0\ (\mathrm{m/s}),\ \delta\alpha]^{\top}\in\mathbb R^7.
\]

DOP853 propagates an augmented state with a **6×7 variational sensitivity matrix**. The Jacobian uses the exact Newtonian gravity Hessian and the derivative of thrust acceleration with respect to the engine scale, **including how mass depletion changes with scale**. Nonlinear synthetic truth is integrated in a separate pass with a different thrust scale and a known initial-state offset. No ground truth is supplied to the fit; it is only used afterward for benchmarking.

With Gaussian independent Cartesian position fixes at times \(t_k\), linearized observations take the form

\[
\mathbf z_k-\mathbf r_\mathrm{nom}(t_k)\simeq
\mathbf H_k\boldsymbol\theta+\boldsymbol\epsilon_k.
\]

The engine scale's *measurement-only effective Fisher information* with six unknown nuisance parameters is

\[
\mathcal I_{\alpha\mid x_0}=\mathbf h_\alpha^\top(\mathbf I-\mathbf Q\mathbf Q^\top)\mathbf h_\alpha,
\]

where the whitened columns in \(\mathbf Q\) span the six initial-state sensitivities. The code computes this orthogonal projection with SVD and reports whether the information is sufficiently strong **relative to a documented 0.20 prior standard deviation and a heuristic information threshold**. This is a *local, model-dependent identifiability diagnostic*, not a universal global-observability theorem. A zero-burn case has exactly zero engine-scale column and hence zero scale information.

The three estimator options apply a Gaussian prior over the seven error parameters. Huber's method uses a 3-component whitened residual-norm threshold of 2.8; the gate retains 0.02 weight for 3D residuals beyond the 99.73% nominal chi-square threshold. Neither robust method is guaranteed to converge to the correct state in the presence of a systematic bias. The reported covariance is an **approximate inverse weighted normal matrix** and is **not** a statistically calibrated sandwich covariance for robust losses. Every uncertainty statement is conditional on model and noise assumptions.

## API example

```bash
curl -X POST http://127.0.0.1:8000/api/navigation/observability \
  -H "Content-Type: application/json" \
  -d '{"duration_minutes":12,"burn_start_s":180,"burn_duration_s":120,"true_thrust_scale":1.05,"observation_sigma_m":30,"fault_bias_m":250,"fault_start_s":360,"fault_duration_s":60,"estimator":"huber","monte_carlo_runs":48,"seed":29,"reference_frame":"APOGEE_IDEAL_ECI"}'
```

Example JSON: [`examples/v19_observability_default.json`](examples/v19_observability_default.json).

Response: `trajectory` (numerical synthetic truth in km and km/s); `observability` (first-run selected-method parameter estimates, time-aligned batch-fit position error and estimated 1σ RSS, Fisher information, and per-method Monte Carlo summaries). The same random seed is reused across all three estimators in each trial; trials differ by seed. The dashboard uses no prescripted trajectory animations.

## Reproducible benchmarks

Bundled `docs/v19-benchmark.json` contains **six conditions, each with 48 paired Monte Carlo trials**. Default parameters: a 400 km, 51.6° circular orbit; 120-second burn starting at 180 s; 12-minute trajectory; ~36-second observation cadence (sample grid); Cartesian position noise σ = 30 m per axis; true engine scale = 1.05; and a short 250 m X-axis measurement fault from 360 s to 420 s (only for faulted cases).

| Synthetic scenario | Least squares median error | Huber median error | Gate median error | Engine-scale identifiable? |
|---|---:|---:|---:|---|
| Clean data | 20.49 m | 20.43 m | 20.49 m | Yes |
| Short 250 m outlier | 33.35 m | 22.07 m | 20.21 m | Yes |
| Persistent 250 m bias | 318.56 m | 315.40 m | 307.16 m | Yes, but biased fit |
| Correlated noise + outlier | 41.38 m | 34.05 m | 33.49 m | Yes, nominal noise assumptions violated |
| Zero thrust | 33.43 m | 20.17 m | 19.24 m | **No** |
| Observations only after 480 s | 33.43 m | 33.43 m | 33.43 m | **No**, confounded by initial-state uncertainty |

**Interpretation:** robust weighting helps this *specific short-outlier distribution*, but persistent biases remain severely damaging. The numbers are final trajectory-fit errors in the synthetic truth model, not operational spacecraft navigation errors. The no-burn case may still yield a well-fit orbit while **providing no information about engine scale**. Longer studies and independently calibrated sensor models are needed before drawing general conclusions.

## Verification and limitations

* **22 v1.9 unit/API tests passed.** They include variational Jacobian checks against independent finite-difference perturbations, no-burn zero-information assertion, late-only observations, seed reproducibility, API validation, and robust comparison.
* **379 inherited tests passed** in completed groups, for **401 passing tests total across seven completed groups**. A single uninterrupted 401-test command was not completed in this environment; no claim is made that it passed as one run.
* **UI:** desktop and mobile offline Chromium smoke tests passed with backend-generated FastAPI responses and no reported JavaScript exceptions or horizontal overflow. The zero-burn rerun was verified. This uses mocked transport; it is **not** a full browser-to-server E2E test.
* **Unverified:** external real satellite ephemerides, live SGP4/CelesTrak access, optional Astropy/IGRF transforms, hardware experiments, and statistical calibration against external observations. Earlier demonstrations may support some of these capabilities, but v1.9 does not newly validate them.

## Project contents

* `apogee/observability.py` — new DOP853 two-body/finite-burn truth, augmented variational ODE, identifiability diagnostic, and robust seven-parameter estimators.
* `apogee/v19_benchmark.py` — deterministic six-scenario paired Monte Carlo benchmark.
* `apogee/adaptive_navigation.py` — retained v1.8 13-state experimental online-style estimator; distinct from the v1.9 batch study.
* `apogee/api.py` — FastAPI endpoints, including the new route.
* `web/` — 23-tab browser interface with computational results, controls and export.
* `tests/test_v19.py` — 22 newly added tests; previous tests are retained.
* `docs/` and `examples/` — older references, v1.9 benchmark output, screenshot previews and sample request.

**Future work:** a fixed-lag smoother and jointly calibrated online EKF that handles time-correlated residuals, parameter observability under attitude errors, and real measurement geometries; leave those as future versions rather than conflating them with this batch demonstration.
