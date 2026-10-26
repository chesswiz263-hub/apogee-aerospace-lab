# APOGEE v1.0 — Integrated Aerospace Laboratory

**Aerospace Physics & Orbital Guidance Engineering Environment**

APOGEE is a local-first numerical spacecraft simulator with a Python physics backend, a FastAPI service, and a dependency-free Canvas-based interactive dashboard. **v1.0 adds a 14-state orbit–attitude–mass propagation engine** and a new **Attitude** tab while preserving the existing 13 simulation and analysis tabs (14 total).

> **Research and education only — not flight software.** Do not use this system to command real spacecraft, dock, avoid collisions, determine operational maneuver times, or certify avionics. The simplified force, control and sensor models have not been validated against authentic mission telemetry.

## Get started

1. Install Python 3.10 or newer.
2. Extract the ZIP. On Windows, double-click `start_apogee_windows.bat`. On Linux/macOS, run `bash start_apogee.sh`.
3. Open <http://127.0.0.1:8000> when the server reports that it has started. This is a localhost-only server, not a hosted account or subscription.
4. Choose **Attitude** in the left panel. Run the default example, then disable **Enable feedback attitude control** and run it again. Compare the pointing error chart, body-axis indicator and resulting orbit elements.
5. Use <http://127.0.0.1:8000/docs> for interactive API documentation.

The launcher installs the dependencies listed in `requirements.txt` if necessary. **No LLM subscription or cloud credentials are required.** Optional CelesTrak catalog access still requires an internet connection.

## The governing model

State vector (14 components):

- Position `r_I` in km (3), velocity `v_I` in km/s (3)
- Unit quaternion `q_BI = [q0, q1, q2, q3]`, scalar first, **actively rotating BODY to idealized INERTIAL coordinates** (4)
- Body-frame angular velocity `omega_B` in rad/s (3)
- Spacecraft wet mass in kg (1)

The integrated equations are

```text
r_dot     = v
v_dot     = -mu*r/|r|^3 + a_J2 + F/(1000*m)*R(q_BI)*[1,0,0]
q_dot     = 0.5 * q_BI ⊗ [0, omega_B]
omega_dot = I^-1*[tau_control + tau_gg - omega_B × (I*omega_B)]
m_dot     = -F/(Isp*g0) during the active burn, otherwise 0
```

The body-fixed main engine points along +X. The attitude controller cannot change the thrust axis instantaneously; it must rotate the modeled rigid body by applying bounded torques. The selected quaternion maps body-axis thrust into inertial accelerations at every solver evaluation. Gravity includes optional Earth J₂; optional gravity-gradient torque is evaluated using the radial vector in the body frame. The mass flow is exactly bounded by available propellant and the engine switches off at the dry-mass boundary.

The attitude controller uses quaternion error feedback, body angular-rate damping, and a *total vector norm* saturation on commanded torque. The **PD gains** have units N·m/rad and N·m·s/rad respectively. Inertia principal moments are fixed at `(120, 100, 80)` kg·m² in this first integrated experiment. Engine specific impulse and thrust are constant during the scheduled burn.

The propagator uses SciPy DOP853 with tight tolerances and splits integration at burn start, cutoff and any propellant depletion event. Quaternion output samples are normalized; no principal-rotation or Euler-angle singularity is used internally.

For scalar-first quaternion and spacecraft rate conventions see the [NASA Spacecraft Modeling, Attitude Determination and Control reference](https://ntrs.nasa.gov/api/citations/20240009554/downloads/Space%20Attitude%20Development%20Control.pdf); note that other spaceflight software packages may use different quaternion component orders, direction conventions and multiplication definitions. **Never interchange quaternion arrays without verifying the convention.**

## v1.0 example: controlled vs uncontrolled pointing

`POST /api/attitude/mission` with `examples/v10_attitude_default.json` (or use the default **Attitude** panel).

| Setting | Default |
|---|---:|
| Earth altitude and inclination | 400 km, 51.6° |
| Mission duration and samples | 35 minutes, 211 time samples |
| Initial body +X pointing error | 25° |
| Target | Prograde velocity direction |
| Dry mass + propellant | 600 + 150 kg |
| Engine | 850 N, 315 s Isp |
| Burn window | T+300 to T+420 seconds |
| PD gains and maximum torque | Kp 1.2, Kd 18, 0.25 N·m |
| J₂ enabled / gravity-gradient torque | yes / no |

Representative deterministic numerical outputs:

| Metric | Controller ON | Controller OFF |
|---|---:|---:|
| Final pointing error | 0.00958° | 27.61488° |
| Mean pointing error during sampled burn | 0.01324° | 24.77384° |
| Propellant used | 33.01938 kg | 33.01938 kg |
| Ideal engine integral Δv | 139.08463 m/s | 139.08463 m/s |

**The ideal Δv integral is not the achieved along-track Δv.** Pointing errors alter the *vector direction* of acceleration, so two scenarios with equal propellant consumption can produce different orbits. These are not estimates of real-spacecraft control performance.

## Numerical validation

Run from the extracted project root:

```bash
python -m pip install -e '.[dev]'
python -m pytest -q tests
python -m apogee.v10_benchmark
```

**198 automated tests passed in this build**: 179 inherited regressions and 19 new attitude-specific tests. The new tests cover Hamilton quaternion algebra, rotation-frame inversion, matrix-to-quaternion conversion, engine-direction mapping, zero-torque rigid-body energy conservation, controller saturation, actual fuel mass flow and exhaustion, control-on/off improvement, and FastAPI input validation.

`docs/v10-benchmark.json` compares the DOP853 integrated 14-state trajectory against a separate fixed-step classical RK4 implementation of the **same force and controller equations**. Over a 180-second demonstration with a 50-second finite burn and optional gravity-gradient torque, the final **position differences** are approximately 1.72 mm with 0.5 s RK4 steps and 0.31 mm with 0.25 s RK4 steps. This measures numerical consistency, **not physical-model correctness**. The rotation-error differences are near the numerical precision floor for this case.

Both desktop and mobile Attitude-panel layouts rendered through an isolated Chromium page with genuine Python-generated results and mocked fetch replies: no JavaScript errors and no horizontal overflow. The controls were exercised with the controller both on and off. **The environment blocks real Chromium navigation to localhost**, so a fully networked browser walkthrough should be performed on your computer. Live CelesTrak, authentic Horizons data, optional Astropy transformations and SGP4 propagation are not newly verified in this release.

## API and example file

- `POST /api/attitude/mission`: propagate orbit, quaternion attitude, body rates, bounded controller, body-axis thrust, and mass consumption.
- `GET /api/health`: reports v1.0.0 and optional dependency availability.
- `POST /api/robust/guidance`: retains v0.9 finite-burn synthetic orbit-navigation EKF.
- `POST /api/robust/sweep`: retains bounded multi-seed sensitivity study.
- All previous orbital propagation, transfer, target, satellite, comparison, navigation, and reference endpoints remain available.

Example using PowerShell:

```powershell
$body = Get-Content -Raw examples\v10_attitude_default.json
Invoke-RestMethod http://127.0.0.1:8000/api/attitude/mission -Method Post -ContentType 'application/json' -Body $body
```

## What v1.0 does **not** yet do

**Attitude/orbit coupling exists in the new Attitude Lab, but it is not yet wired into the previous robust orbit-navigation EKF as a single fully closed-loop 6-DOF GN&C system.** That is important: the v0.9 navigation estimator still has idealized thrust-pointing assumptions and cannot yet track the new 14-state attitude dynamics. There are no reaction-wheel momentum states, wheel saturation, magnetic torquers, detailed inertia changes, gimbals, sensor biases, star tracker or gyro simulation, attitude state estimation, proper Earth orientation epochs, higher-degree geopotential, third-body gravity, or calibrated atmospheric density.

The body controller assumes an ideal direct torque actuator with instantaneous response and norm saturation; it does **not** allocate torques to physically modeled reaction wheels. Prograde tracking is only approximate and does not include sophisticated feedforward rigid-body dynamics. These limitations are deliberate and exposed instead of falsely implying flight-qualified engineering fidelity.

**Recommended v1.1 work:** add actuator dynamics, sensor-driven attitude determination, and an attitude-aware closed-loop orbital guidance experiment. Use robust multi-seed metrics and external benchmark data before increasing claims about accuracy.

## Layout

- `apogee/attitude.py` — 14-state model, quaternion algebra, torque controller and body-axis thrust.
- `apogee/v10_benchmark.py` — DOP853 / independent RK4 comparison.
- `tests/test_v10.py` — 19 new targeted numerical and API checks.
- `web/` — integrated attitude and previous mission dashboard modes.
- `docs/README-v0.9.md` — archived previous version details.
- `examples/v10_attitude_default.json` — reproducible default API request.
- `docs/v10-benchmark.json` — saved numerical benchmark output.

No license has been chosen; decide licensing terms before publishing source code.
