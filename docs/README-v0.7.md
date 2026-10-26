# APOGEE v0.7 — Finite-Burn Optimization & Navigation Uncertainty

**APOGEE · Aerospace Physics & Orbital Guidance Engineering Environment**

A local-first orbital mechanics and aerospace mission-design prototype built around a Python numerical engine and a dependency-free interactive browser mission-control interface. Version **0.7** preserves v0.1–0.6 and adds **two new operating modes:** **Finite burns** and **Uncertainty** (11 modes total).

> **IMPORTANT:** APOGEE is an educational and research prototype, **not flight-qualified**, not an operational satellite navigation tool, and not an authenticated conjunction-analysis or collision-probability system. The mathematical outputs are reproducible only within the specified force, frame and uncertainty assumptions. No global optimizer certificate is claimed.

## Quick start

1. Extract `APOGEE-v0.7.zip` into a normal writable folder.
2. Install Python 3.10+ if needed.
3. Windows: double-click **`start_apogee_windows.bat`**. macOS/Linux: run `bash start_apogee.sh` from the project directory.
4. When the server is running, open **http://127.0.0.1:8000**. API documentation: **http://127.0.0.1:8000/docs**.
5. Select **Finite burns** or **Uncertainty** at the top of the left sidebar.

Dependency installation requires internet if the packages aren't available yet. Orbital simulations subsequently work offline. The server binds to **127.0.0.1** only.

## New — Finite burns

`POST /api/finite/optimize` accepts:

```json
{
  "start_altitude_km": 400,
  "target_altitude_km": 700,
  "inclination_deg": 51.6,
  "dry_mass_kg": 600,
  "propellant_kg": 150,
  "specific_impulse_s": 315,
  "thrust_n": 850,
  "phase_candidates": 4,
  "j2_enabled": false,
  "samples": 280
}
```

Implementation: `apogee/finite_optimization.py`.

**Method:** For each selected initial circular-orbit phase, simulate a constant-thrust prograde burn (retrograde for lowering), with mass depletion `dm/dt = -T/(I_sp g0)` and continuous local velocity-following steering. Brent root-finding adjusts the **departure duration** to reach the target radius at the **first post-cutoff opposite apsis**, located as a radial-velocity root event. At that event, ignite an actual **second finite-duration burn**. A bounded scalar minimizer adjusts the second duration to minimize the **postburn two-body osculating eccentricity**. Return the lowest-fuel tested phase among those satisfying `e <= 0.005` where possible, or the best achievable sampled trial; report the exact residual.

This is not simply converting an ideal delta-v into a duration; the trajectory is propagated with changing wet mass, gravity and optional J₂ throughout both powered phases and the coast. Each burn is bounded by the propellant on board. If no candidate can meet the target within the mass/thrust bounds, the API rejects the scenario with an explanatory 422 response.

**Default example (two-body, 400→700 km, 600 kg dry, 150 kg fuel, 850 N, 315 s):**

| Quantity | Output |
|---|---:|
| First burn | 71.901 s |
| Apsis event time | 2,905.556 s |
| Second burn | 69.118 s |
| Propellant used | 38.803 kg |
| Propellant remaining | 111.197 kg |
| Engine-integrated ideal delta-v | 164.105 m/s |
| Target prearrival apsis radius residual | ~0.00017 m |
| Postburn eccentricity | 0.0008043 |
| Postburn perigee / apogee altitude | 694.03 / 705.42 km |

**Important distinction:** the first burn targets the *prearrival apsis radius* precisely. The finite arrival burn starts **at that apsis**, not centered on it, and produces a slightly noncircular final orbit. A 0.00017 m apsis root residual **does not** imply centimeter-accurate mission injection; the actual postburn osculating orbit is stated separately. Unlike ideal impulses, the engine burns take time and follow a curved direction.

**Thrust strategies:** The API also independently resolves durations for **0.5×, 1× and 2× thrust** at the **selected** orbital phase. Their fuel use, duration and final eccentricity differ; these are not identical end-state transfers and should not be compared as a certified minimum-energy or globally minimum-fuel solution. They are constrained design alternatives.

**Known limits:** restricted to initial circular Earth orbits between 200–3,000 km and altitude changes 30–1,000 km; 1, 4, or 8 initial phase samples; constant engine Isp/thrust and local velocity steering; no out-of-plane targeting, throttle scheduling, drag during optimized transfer, attitude/slew constraints, third-body gravity, low-thrust spirals, or globally optimal schedules. J₂ is optional, and its inclusion does not make the postburn orbit J₂-stationary. The sample engine may not be physically representative of a particular spacecraft.

## New — Uncertainty

`POST /api/uncertainty` accepts:

```json
{
  "altitude_km": 400,
  "inclination_deg": 51.6,
  "duration_minutes": 90,
  "position_sigma_m": 100,
  "velocity_sigma_mm_s": 10,
  "monte_carlo_samples": 64,
  "seed": 42,
  "j2_enabled": false,
  "steps": 120
}
```

Implementation: `apogee/uncertainty.py`.

At one assumed epoch in idealized Earth-centered inertial Cartesian coordinates, form a six-state covariance with mutually **independent, isotropic Gaussian position and velocity errors**. Draw 16, 32, 64 or 128 seeded initial-state realizations. Numerically propagate each with SciPy DOP853 under two-body or optional J₂, with all model parameters assumed exact. At each shared epoch, compute the sample 6×6 covariance and local radial/along-track/cross-track position standard deviations.

A displayed **conservative Gaussian-approximation 95% positional radius** is `sqrt(chi2_3(0.95)) * sqrt(max_eigenvalue(P_rr))`, centered on the Monte Carlo sample mean for covariance estimation. This is an anisotropic-covariance enclosing-sphere approximation, **not a calibrated or guaranteed confidence region** under nonlinear dynamics or limited Monte Carlo sample sizes. No collision likelihood can be inferred.

Separately, a **6×6 Cartesian state-transition matrix** at the end epoch is calculated by central finite differences of six initial-state dimensions; the linearized covariance is `Phi P0 Phi^T`. The response contains both the Monte Carlo covariance and the independently computed linearized covariance to support discrepancy analysis. Cartesian position uses **km**, Cartesian velocity **km/s**, and mixed covariance units follow those state units.

**Default seed-42 example:** conservative nominal 95%-radius from ~289.5 m at the initial sample to ~5,279.7 m after 90 minutes; final MC 1σ major spatial axis ~1,888.7 m versus ~1,880.2 m from the STM approximation. This is propagation of *assumed input errors under a specified deterministic model*; it is not observed uncertainty for a real spacecraft. Seed and sample size affect MC estimates.

**Known limits:** no real tracking data or estimated covariance, sensor uncertainty model, maneuver errors, process noise, missing force-model uncertainty, atmospheric-density uncertainty, clock/epoch uncertainty, correlations in the initial Cartesian errors, TEME/ICRF conversion, or collision screening. Do not use for orbital safety decisions.

## Existing v0.1–v0.6 operating modes

| Mode | Purpose |
|---|---|
| Orbit | Keplerian initial states and RK4 two-body propagation |
| Transfer | Analytical Hohmann maneuvers with propagated transfer |
| Thrust lab | Editable finite-burn timelines; coupled drag/J₂ and mass depletion |
| J₂ + Drag | Earth oblateness and an illustrative exponential atmosphere |
| Catalog | CelesTrak OMM/JSON import and optional SGP4 TEME propagation |
| Targeting | Two-body ideal tangential impulse targeting |
| Reference | User-attested JPL Horizons vector text import, frame conversions |
| Optimize | Event-targeted **ideal impulse** phase-grid optimizer |
| Validate | Matched-epoch **assumed** Earth-centered ICRF reference residuals |
| **Finite burns** | Two actually propagated bounded finite engine burns and thrust strategy comparison |
| **Uncertainty** | Initial Gaussian-state Monte Carlo and 6×6 numerical STM |

The original v0.6 scientific conventions remain in the Python modules and prior tests. Imported ephemeris metadata is checked but **provenance is not authenticated**. Never directly subtract SGP4 TEME from ICRF or idealized ECI vectors.

## Tests and scientific checks

```bash
python -m pip install -e '.[dev]'
python -m pytest -q tests
python -m apogee.v07_benchmark
python -m apogee.validation
python -m apogee.benchmark
```

- **125 automated tests passed locally**, including the previous 105 and 20 new v0.7 tests. New tests cover 400→700 km and 700→400 km, Earth J₂ cases, propellant conservation and exhaustion rejection, physically separate ideal/finite calculations, seeded reproducibility, covariance positive semidefiniteness and numerical STM structure, request validation, and both API routes.
- An additional **independent RK4 vs DOP853** coupled-dynamics check compares the same two finite burns and returns ~**0.023 m** final position difference over this particular scenario. This is numerical agreement for the *same idealized dynamics*, not real-world orbital accuracy.
- Browser preview checks used **actual backend-generated JSON injected into an isolated browser page**, with desktop and mobile rendering and no JavaScript exceptions or horizontal overflow. Local HTTP navigation from browser was **blocked** in the build environment, so live browser/server end-to-end integration must still be confirmed on your machine. Screenshots: `docs/v07-*.png`.
- Actual JPL/CelesTrak retrieval, live satellite propagation, optional Astropy conversions and operational ephemeris validation remain out of scope for these v0.7 tests.

## Source code and examples

- `apogee/finite_optimization.py` — bounded powered transfer root/event search and engine-thrust comparisons.
- `apogee/uncertainty.py` — Monte Carlo RTN envelopes, Cartesian 6×6 covariance and STM.
- `apogee/v07_benchmark.py` — deterministic demonstrations and cross-integrator numerical comparison.
- `apogee/api.py` — FastAPI endpoints including `/api/finite/optimize`, `/api/uncertainty`.
- `web/index.html`, `web/app.js`, `web/style.css` — no-build dependency-free mission control.
- `tests/test_v07.py` — dedicated v0.7 regressions.
- `examples/v07_finite_request.json`, `examples/v07_uncertainty_request.json` — ready-to-send JSON examples.

**System of units:** position km; velocity km/s; time seconds; mass kg; thrust newtons; Isp seconds; uncertainty input position meters and velocity mm/s. **Distribution:** Select an appropriate open-source or proprietary license before publicly distributing the project.
