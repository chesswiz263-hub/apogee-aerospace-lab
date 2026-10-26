# APOGEE v2.0 — Scientific Validation & Causal Ground Navigation

**Aerospace Physics & Orbital Guidance Engineering Environment** · educational research simulator · **24 dashboard modes** (23 preserved from v1.9 plus Ground Nav v2.0).

> **NOT FLIGHT SOFTWARE. Not validated against real tracking data.** v2.0 models *synthetic* range/range-rate tracking from a fabricated spherical-Earth station network. The timestamp, station, and frame contracts are strict because externally supplied data must not be silently treated as compatible. A CSV or JSON file labeled “real” does **not** become operational truth merely by loading it. Historic real-source integrations in older tabs also retain their individual verification limits.

![Actual isolated Chromium check of new navigation dashboard](docs/v20-desktop.png)

## Quick start

On **Windows**, extract `APOGEE-v2.0.zip`, double-click `start_apogee_windows.bat`, and open `http://127.0.0.1:8000` once the local server starts. Internet access may be needed for the initial dependency installation.

On macOS / Linux:

```bash
python -m pip install -r requirements.txt
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
```

Choose **Ground Nav v2.0**. Change station count, minimum elevation, measurement uncertainties, packet delay, finite burn, fault injection and Monte Carlo seeds. The simulator propagates spacecraft trajectories numerically; the browser simply displays their outputs.

API documentation lives at `/docs`. No AI model, token subscription, paid cloud service, or live satellite feed is needed for the new ground-tracking mode.

## New capabilities

- **Finite-thrust Earth-orbit truth model**, integrated by SciPy DOP853 with a fixed inertial burn direction and thrust-dependent mass flow; independently implemented RK4 state propagation predicts the navigation estimate.
- **Rotating spherical-Earth stations:** 6, 12, or 24 hypothetical geocentric station locations; approximate UTC sidereal phase plus constant Earth spin; station inertial velocity included in range-rate calculations.
- **Geometric tracking:** line-of-sight elevation mask, two most-elevated visible stations per measurement epoch, noisy slant range (m) and range-rate (m/s), station IDs and capture timestamps.
- **Strict causal EKF:** 7-state estimate **[r_x,r_y,r_z,v_x,v_y,v_z,engine-scale]**; a full 7×7 covariance, analytic measurement Jacobians and Joseph-form measurement updates. The packet delay is explicit; **only packets received by the current simulation time** may affect that estimate. At each time, received packets are replayed in capture-time order, including packets that arrived late.
- **Observability failures remain visible:** zero burn makes engine scale unobservable, and poor station geometry can leave large predicted uncertainty; a 30° elevation mask may yield no received packets during the short scenario.
- **Reference-residual endpoint:** accepts user-provided packets only under the exact declared synthetic geometry and frame. It reports RMS residuals against a nominal **model prediction**, not an independently validated physical trajectory or a certified orbit fit.
- **Reproducible Monte Carlo:** 12 seeds per condition in the bundled six-scenario benchmark. Results are illustrative, with insufficient sample size for safety or operational coverage claims.

## Dynamics and estimation equations

The simulated translational dynamics are

\[\dot{\mathbf r}=\mathbf v,\qquad \dot{\mathbf v}=-\frac{\mu_E}{r^3}\mathbf r+\frac{\alpha T}{1000m(t)}\hat{\mathbf u}\,I_{\mathrm{burn}}(t),\qquad \dot\alpha=0.\]

For the finite burn, \(m(t)=750-\alpha\frac{T}{I_{sp}g_0}\Delta t_{\mathrm{burn}}\) kg. Standard constants are \(T=850\) N and \(I_{sp}=315\) s. Here r is in km, v in km/s, T is in newtons, and the 1000 converts meters to kilometers. The burn axis is frozen in inertial coordinates, and the mass-flow model is an idealization.

Ground-station inertial position r_s and velocity v_s are computed using an **approximate** sidereal rotation. With \(\boldsymbol\rho=\mathbf r-\mathbf r_s\) and \(\mathbf u=\boldsymbol\rho/\|\boldsymbol\rho\|\), the measurements are \(z_1=1000\|\boldsymbol\rho\|\) meters and \(z_2=1000\mathbf u\cdot(\mathbf v-\mathbf v_s)\) m/s. The two-component Jacobian differentiates both the slant range and line-of-sight range rate with respect to position and velocity.

The navigation filter propagates an approximate 7×7 covariance with numerically stepped Jacobians and assumed white acceleration/thrust-scale process noise. Measurement covariance is diagonal for range and range rate. The posterior uses the Joseph covariance update. These error distributions and covariances are uncalibrated against operational tracking data.

**The inertial frame is `APOGEE_SIM_ECI_EARTH_ROTATING`, not TEME, GCRS, or ITRS.** Its absolute UTC epoch is converted to an *approximate* mean sidereal angle, without polar motion, UT1–UTC, precession/nutation, ellipsoidal station geodesy, tropospheric/ionospheric delay, relativistic range corrections, transponder delays, or actual station clocks. Do **not** compare these samples directly with real satellite ranging records without adding correct frame and measurement models.

## First reproducible result

Default: 400 km altitude, 51.6° inclination, 12 min mission, 120 s burn from t=180 s, 24 synthetic stations, 5° mask, 45 s cadence, 20 s base latency, position-estimate offset 600 m and velocity offset 0.4 m/s, range σ=20 m, range-rate σ=0.04 m/s, true engine scale 1.05, seed 29.

- First-seed final position-estimation error: **243.12 m**.
- Final reported one-sigma position root-sum-square: **881.76 m** (not a guaranteed 68% probability sphere).
- Estimated thrust scale: **1.0493**, versus the synthetic true scale **1.0500**.
- Received ground-observation packets: **9** out of **9** generated. Ground visibility is sparse even with 24 hypothetical stations.
- Independent RK4 versus DOP853 position difference using exactly matched synthetic initial conditions and thrust scale over 720 s: approximately **0.000091 m**. This measures integration agreement only.

## Six-scenario benchmark, 12 seeds each

| Synthetic scenario | Median final error | 90th percentile | First-run packets received |
|---|---:|---:|---:|
| nominal finite burn | 338.66 m | 664.31 m | 9 |
| no burn scale unobservable | 399.39 m | 618.87 m | 9 |
| sparse six stations | 556.87 m | 798.42 m | 7 |
| high elevation mask | 3,147.89 m | 3,147.89 m | 0 |
| long packet latency | 338.66 m | 664.31 m | 9 |
| persistent range bias | 2,101.22 m | 2,422.27 m | 9 |

**Interpretation:** the no-burn case cannot estimate engine scale (it remains at the initial value 1.0). A sparse network increases median error. A high elevation mask can leave the spacecraft completely unobserved. A persistent biased range sensor can severely mislead the EKF. A large packet delay changes when observations are usable; with all generated packets received before mission end, the final estimate may match the shorter-delay experiment after full replay. These numbers are from fabricated measurements, not empirical navigation accuracy.

Reproduce:

```bash
python -m apogee.v20_benchmark docs/v20-benchmark.json
python -m pytest -q tests/test_v20.py
python -m pytest -q tests
```

## Example API calls

Numerical ground tracking:

```bash
curl -X POST http://127.0.0.1:8000/api/navigation/ground-tracking \
  -H "Content-Type: application/json" \
  -d '{"duration_minutes":12,"station_count":24,"burn_start_s":180,"burn_duration_s":120,"true_thrust_scale":1.05,"telemetry_delay_s":20,"monte_carlo_runs":3,"reference_frame":"APOGEE_SIM_ECI_EARTH_ROTATING"}'
```

**Reference-residual importer:** POST JSON to `/api/navigation/ground-reference-residuals` with `configuration` and `records` as shown in [`examples/v20_reference_synthetic.json`](examples/v20_reference_synthetic.json). Each record has `capture_time_s`, `station_id`, `range_m`, and `range_rate_m_s`. The configuration must explicitly declare the simulator's frame. Output RMS range and range-rate residuals compare packets with model predictions. The file is a synthetic fixture—not real tracking data.

## Verification status

- **25 newly added v2.0 automated tests pass**, including finite-difference checks of station measurement Jacobians, range-rate versus numerical derivative, Earth-rotation velocity, covariance PSD, zero-thrust observability, seed reproducibility, packet receipt causality, and API metadata/frame rejection.
- **401 inherited tests pass** across prior-feature batches, for **426 passing tests total** across completed test groups. A single uninterrupted 426-test command was not run.
- **Desktop/mobile isolated Chromium rendering checks pass**, using backend-generated responses and switching to a no-burn mission; no JavaScript exception or horizontal overflow was detected. These checks mock browser transport; they are **not live browser-to-server E2E verification**.
- The FastAPI endpoint was called directly, the example requests and archives were checked, and the ZIP integrity was verified.
- **Unverified:** real ground-observation comparisons, real station catalogs/time standards, calibrated covariance coverage, optional Astropy/IGRF execution, live CelesTrak/SGP4 operation, actual spacecraft navigation performance or any flight/hardware qualification.

### Project layout

`apogee/ground_navigation.py` is the ground geometry, causal filter, and reference residual module. `apogee/v20_benchmark.py` reproduces six seeded scenarios. `tests/test_v20.py` contains 25 new tests. The older physics/nav modules and all prior dashboard modes are retained. `docs/v20-desktop.png`, `docs/v20-mobile.png`, `docs/v20-benchmark.json`, and the `examples/` directory contain the new release references.

**Next:** validate ITRF/ITRS and celestial transformations using a trusted Earth-orientation data source; add true station antenna geodesy, clock and atmosphere models, and authenticated external tracking/ephemeris records with audited uncertainty propagation. Only then claim a comparison to real missions.
