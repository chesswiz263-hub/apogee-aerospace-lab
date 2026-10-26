# APOGEE v1.2 — Precision Attitude Navigation

**Aerospace Physics & Orbital Guidance Engineering Environment**  
A local-first, Python-based orbital mechanics and spacecraft mission-analysis laboratory with a browser mission dashboard.

APOGEE v1.2 preserves the previous **15 mission modes**, adding a sixteenth mode, **Precision ADCS v1.2**. The new laboratory adds a **multiplicative extended Kalman filter (MEKF)** for attitude and gyroscope bias estimation, simulated star-tracker field-of-view exclusion, gyroscope bias random walk, and dipole-limited **reaction-wheel magnetic momentum unloading**.

> **Research and teaching only — NOT flight software.** In particular, the Sun direction and terrestrial magnetic field are synthetic approximations, the tracker returns direct noisy attitude quaternions, and the filter's numerical covariance has not been calibrated against measured spacecraft data. The results are not guidance commands or operational navigation estimates.

## Start on Windows

1. Extract the entire `APOGEE-v1.2.zip` archive (preserve the folder structure).
2. Install Python 3.10+ if necessary. Double-click `start_apogee_windows.bat` in the extracted project folder. The first launch may use the internet to install Python libraries.
3. After the console reports a running Uvicorn server, open **http://127.0.0.1:8000**.
4. Select **Precision ADCS v1.2**, press **RUN PRECISION ADCS**, and compare the magnetic unloading checkbox on and off.
5. Interactive API documentation is at **http://127.0.0.1:8000/docs**. Linux/macOS users may run `bash start_apogee.sh`.

A cloud account, paid LLM, or API key is **not** required for the local physics engine or dashboard. Optional satellite-catalog network operations are separate.

## v1.2 implementation

| Module | Purpose |
|---|---|
| `apogee/precision_adcs.py` | 17-state spacecraft truth integration; quaternion MEKF with six-state error covariance; synthetic gyro and tracker measurements; magnetic-torquer unloading |
| `apogee/adcs.py` | Preserved v1.1 complementary filter and three reaction wheels |
| `apogee/api.py` | New `POST /api/precision-adcs/mission`, previous endpoints still available |
| `web/` | New Precision ADCS form, true-versus-estimated attitude chart, navigation consistency and wheel metrics |
| `tests/test_v12.py` | 26 new numerical, sensor, actuator, and API checks |
| `apogee/v12_benchmark.py` | Repeatable 10-seed synthetic benchmark written to `docs/v12-benchmark.json` |

### Truth equations

The **17-component physical state** is `[r_I(3), v_I(3), q_BI(4), omega_B(3), m(1), h_B(3)]`. Position is in km, velocity in km/s, scalar-first Hamilton quaternion rotates **body to inertial**, angular rate is rad/s, wet mass kg, wheel momentum N·m·s. The spacecraft has a constant diagonal inertia tensor and ideal rigid body and wheel axes.

```text
r_dot        = v
v_dot        = -mu*r/|r|^3 + a_J2 (optional) + [T/(1000*m)] R(q_BI) e_X
q_dot        = (1/2) q_BI ⊗ [0, omega_B]
I omega_dot  = tau_magnetic + tau_gg (optional)
               - h_dot - omega_B × (I omega_B + h_B)
h_dot        = bounded wheel motor momentum derivative
m_dot        = -T/(Isp*g0) during a powered, fuelled arc
```

The DOP853 integrator handles burn start/cutoff, dry-mass fuel exhaustion, and zero-order-held motor and magnetic dipole **commands**; the local field and attitude-dependent `m × B` torque are reevaluated throughout integration. Thrust follows physical body +X orientation. No explicit motor voltage, current, friction, wheel geometry, or power model exists.

### Error-state multiplicative EKF

The nominal estimator consists of `(q_hat, b_hat)` with a **6×6** error covariance over **right-multiplicative** `delta_theta_B` and three gyro-bias errors. It receives gyroscope angular-rate measurements plus intermittent synthetic direct-quaternion tracker fixes. It does **not** receive the simulator's true attitude or true angular velocity. Truth is accessed separately only to synthesize measurements and calculate diagnostic errors.

For corrected gyro angular rate `omega = omega_meas − b_hat`, a local linearization uses

```text
d/dt [delta_theta] = -[omega]x delta_theta - delta_bias + gyro_noise
d/dt [delta_bias ] = bias_random_walk
F = [ -[omega]x  -I ; 0  0 ]
H = [ I  0 ]
```

The predictor uses measured gyro samples, a second-order discrete transition approximation, configurable process noise, and an explicitly normalized quaternion. Each tracker innovation is the small rotation `log(q_hat⁻¹ ⊗ q_measurement)`; the correction is applied multiplicatively, with a **Joseph covariance update**, a first-order reset map, covariance symmetrization and PSD flooring. Gyro bias is explicitly estimated. The sensor measurement model has independent Gaussian attitude noise and gyro sample noise; true bias undergoes a seeded three-axis random walk.

This is an **educational MEKF**, not a high-rate sensor-accurate estimator. Trapezoidal gyro integration partially compensates for coarse measurement timing. The covariance propagation and reset use truncated approximations, rather than an exact discretization or measured calibration. Reports include per-sample attitude uncertainty, 3-DOF normalized innovation squared (**NIS**) at star-tracker updates, and 6-DOF normalized estimation error squared (**NEES**) using truth **for diagnostics only**. The interpretation of NIS/NEES requires many properly calibrated runs; one visually plausible realization is not sufficient.

### Star-tracker visibility

A synthetic tracker boresight is the spacecraft body **+X** axis. It can be gated by:

- The angular exclusion cone about a **fixed inertial Sun +X direction** (this is *not* an astronomical Sun ephemeris).
- The Earth's apparent disc angular radius `asin(R_Earth / |r|)` plus a configurable limb margin.
- A configurable scheduled tracker blackout and measurement interval.

The tracker does not process star pixels or a catalog, perform celestial light-time corrections, calculate real Earth orientation, or represent hardware-specific exclusion geometry.

### Reaction-wheel momentum unloading

APOGEE simulates an **external magnetorquer**, rather than arbitrarily resetting stored wheel momentum. A simple axial, stationary dipole field is used:

```text
B_I(r) = B_equator (R_Earth/|r|)^3 [ 3 n (z_hat·n) − z_hat ]
B_body = R(q_BI)^T B_I
requested torque = -k_unload h_B
magnetic dipole command = bounded( B_body × requested_torque / |B_body|² )
actual torque = magnetic dipole × B_body
```

The dipole is limited in A·m² and the applied magnetic torque is **always perpendicular to the instantaneous field**. Reaction wheel torque is separately limited, and the simulated controller offsets the external torque to maintain pointing when possible. There is no guaranteed ability to unload all wheel axes simultaneously, no IGRF/WMM field, no Earth rotation in the field model, no electromagnetic coil circuit model, and no guarantee of stable control under arbitrary saturation. The idealized field orientation is fixed in inertial space and should **not** be used as an operational Earth-field prediction.

## Default benchmark

The deterministic default is an 18-minute, 400 km, 51.6° Earth-orbit scenario, starting with **2 N·m·s** of Y-axis wheel momentum. Attitude initially points 25° away from prograde. It uses an 850 N, 315 s engine from T+300 s through T+420 s and a 600 kg dry / 150 kg propellant vehicle. Synthetic sensors update on the numerical decision grid; the tracker attempts to take fixes every 30 seconds. The simulated magnetorquer is limited to **120 A·m²**.

With random seed 27:

| Outcome | Unloading enabled | Same case without unloading |
|---|---:|---:|
| Final wheel momentum magnitude | **0.642 N·m·s** | **2.017 N·m·s** |
| Final true pointing error | 0.015° | 0.076° |
| Final attitude estimation error | 0.070° | 0.069° |
| Tracker fixes | 27 | 27 |
| Engine propellant consumed | 33.019 kg | 33.019 kg |

Default seed-27 average **NIS = 3.12** (for three measured attitude axes) and average **NEES = 5.08** (for six error components). The tracker is excluded by Earth geometry at two scheduled sample epochs. With tracker updates disabled, synthetic final attitude estimation error grows to approximately **20.29°**.

A separate **ten-seed** synthetic experiment, seeds 25–34, finds median final estimation error **0.107°**, maximum observed **0.186°**, median mean NIS **2.93**, and median mean NEES **6.59**. These are **not** confidence bounds or real-world reliability estimates.

Reproduce these values with:

```bash
python -m apogee.v12_benchmark
```

## API example

`POST /api/precision-adcs/mission` with JSON:

```json
{
  "altitude_km": 400,
  "duration_minutes": 18,
  "target_mode": "prograde",
  "gyro_bias_deg_s": 0.015,
  "gyro_bias_random_walk_deg_s_sqrt_s": 0.000015,
  "tracker_enabled": true,
  "tracker_visibility_enabled": true,
  "star_tracker_sun_exclusion_deg": 25,
  "star_tracker_earth_limb_margin_deg": 8,
  "initial_wheel_momentum_nms": 2,
  "wheel_momentum_limit_nms": 8,
  "desaturation_enabled": true,
  "magnetorquer_max_dipole_am2": 120,
  "seed": 27
}
```

Results include the full numerical `trajectory` and `attitude`, a `precision_adcs` payload (estimated quaternion, estimated/actual gyro bias, tracker count, gating, per-sample covariance radius, NIS, NEES, wheel momentum, motor and magnetic commands), model assumptions and warnings, and summary metrics. `nis` uses JSON `null` where no measurement update was taken. Invalid bounds return HTTP 422.

## Tests and verification

```bash
python -m pip install -e '.[dev]'
python -m pytest -q tests
node --check web/app.js  # optional, if Node is installed
```

**245 tests pass across three completed groups**, without a failed physics regression:

- 105 tests for the long-standing core physics, orbit perturbations, API, targeting, and earlier numerical modules.
- 93 inherited regression tests for v0.7–v1.0 (three stale version assertions were updated from 1.1.0 to 1.2.0).
- 47 v1.1 and v1.2 tests, including 26 new v1.2 checks.

The **Precision ADCS v1.2** interface was exercised at 1440×900 desktop and 390×844 mobile viewports in isolated Chromium. The test supplied actual responses from FastAPI's TestClient through the browser's mocked network route. There were no JavaScript exceptions and no horizontal viewport overflow. The magnetic unloading toggle was exercised. This is **not** proof of a fully connected Chromium-to-localhost deployment; that still requires on-device verification.

### What has NOT been validated

There is no comparison to spacecraft sensor logs, real tracker visibility ephemerides, real star catalogs, magnetometer observations, verified magnetic field standards, high-rate sensor hardware, onboard processor timings, hardware reaction-wheel transfer functions, or full nonlinear navigation/attitude uncertainty consistency. Translational position/velocity estimation is not fused into this attitude MEKF (the orbit used by its controller remains simulated/known). There are no flight qualification, safety, or reliability claims. The Earth gravity and Sun/dipole approximations restrict all results to research-scale demonstrations.

**Candidate v1.3:** sensor-aligned, time-stamped attitude/navigation fusion, realistic ephemerides and inertial/Earth-fixed transforms, magnetic-field model validation, and a genuine hardware-in-the-loop interface.

## Background

- [NASA Small Spacecraft Technology State of the Art: Guidance, Navigation & Control](https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/)
- [NASA NESC Academy: Spacecraft Attitude Determination and Estimation](https://nescacademy.nasa.gov/video/bdeb764e048940a6b2ae05c3cfdf5d261d)

Background references inform terminology only; they do not validate or endorse APOGEE's implementation.
