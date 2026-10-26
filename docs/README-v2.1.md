# APOGEE v2.1 — Real-World Reference Frames & Geometric Ground Tracking

**Aerospace Physics & Orbital Guidance Engineering Environment** · educational research simulator · **25 dashboard modes** (24 retained from v2.0 plus Real Frames v2.1).

> **Not flight software. No actual mission tracking data have been authenticated or verified in this release.** The v2.1 addition replaces v2.0's synthetic spherical-station *geometry* with an independently testable **WGS84 geodetic station / ITRS observation model**, and offers an **optional strict GCRS-to-ITRS transformation** through Astropy's IERS Earth-orientation machinery. The new observational model is **instantaneous and geometric**; it does not correct light time, atmospheric refraction/delay, relativistic effects, clock or transponder delays, or hardware calibration. Do not represent residuals as operational navigation accuracy.

![APOGEE v2.1 offline browser validation](docs/v21-desktop.png)

## Launch

1. Install Python 3.10 or newer; extract the full project directory.
2. On Windows, double-click **`start_apogee_windows.bat`**. On Linux/macOS: `python -m pip install -r requirements.txt` and `python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000`.
3. Open **http://127.0.0.1:8000**, then select **Real Frames v2.1**.
4. Set WGS84 station latitude/longitude/ellipsoidal height and **UTC epoch**, and supply a Cartesian six-state expressed in the declared frame (position km, velocity km/s). Hit **Calculate Ground Geometry**.

The default input is a **synthetic ITRS spacecraft directly overhead** a WGS84 station at 45°N, 120°W, 100 m ellipsoidal height. It yields approximately 400,000 m geometric slant range, zero radial velocity, and 90° elevation. **Azimuth is undefined at zenith**; APOGEE reports that explicitly. This is a mathematical test case, not a real satellite ephemeris.

**ITRS mode runs offline with only the base dependencies.** GCRS states require Astropy and appropriate local IERS Earth-orientation data. To enable the *optional* reference packages: `python -m pip install -r requirements-reference.txt`. The installer does **not** silently fall back to an approximate Earth-spin rotation if those reference dependencies or Earth-orientation tables are missing or stale. Astropy's IERS auto-download is disabled in this path; bundled Earth-orientation data need to cover the requested epoch. Updating the IERS data is a separate explicit maintenance operation.

## New in v2.1

| Feature | Implementation | Scientific boundary |
|---|---|---|
| WGS84 station geodesy | Latitude, longitude, ellipsoidal height → ITRS Cartesian, including Earth flattening | Height is above WGS84 ellipsoid, not mean sea level |
| ITRS tracking | Geometric slant range and range rate from a terrestrial satellite six-state | Velocity must be *ITRS coordinate derivative*, not inertial velocity |
| Local viewing geometry | East/north/up azimuth and elevation using geodetic surface normal | Azimuth undefined exactly at zenith |
| Absolute UTC | Strict ISO-8601 UTC with trailing `Z`, epoch metadata retained | No silent UTC ≈ UT1 assumption |
| Optional GCRS transform | Astropy GCRS Cartesian representation/differential → ITRS at matching epoch, strict IERS policy | Requires correct EOP; positive path unverified in this build environment |
| Reference residuals | Bounded 1–200 observation comparison, range/range-rate bias and RMS | Input provenance **never** treated as authenticated |
| Frame safety | Reject TEME, ICRF labels, v2.0's approximate synthetic ECI frame, mixed-frame batches | Prevents invalid subtraction across reference frames |
| Dashboard | New Real Frames mode with separate observation panel and error readout | Prior orbit plots deliberately hidden in the mode |

All previous 24 modes remain present. The v2.0 synthetic Ground Nav tab **has not** been retroactively promoted to real-IERS navigation; it remains a separate synthetic model with the same limitations and outputs.

## Governing geometry

For geodetic latitude \(\phi\), longitude \(\lambda\), and ellipsoidal height \(h\) in meters,

\[
N(\phi)=\frac{a}{\sqrt{1-e^2\sin^2\phi}},\qquad e^2=f(2-f),
\]

\[
\mathbf r_s = \begin{bmatrix}(N+h)\cos\phi\cos\lambda\\(N+h)\cos\phi\sin\lambda\\(N(1-e^2)+h)\sin\phi\end{bmatrix},
\]

with \(a=6378137\) m, \(f=1/298.257223563\). For satellite ITRS position \(\mathbf r\) and ITRS coordinate velocity \(\mathbf v\), define \(\boldsymbol\rho=\mathbf r-\mathbf r_s\), \(\hat\rho=\boldsymbol\rho/\|\boldsymbol\rho\|\). A fixed station has zero *ITRS coordinate velocity*, giving

\[
\rho=\|\boldsymbol\rho\|,\qquad \dot\rho=\hat\rho\cdot\mathbf v.
\]

The ENU basis uses the **geodetic** local vertical, not a spherical radial approximation. Azimuth is clockwise from north. The functions return SI units for range and range rate. Input states are in kilometers and kilometers per second.

**Critical:** When the input state is GCRS, a full epoch-tagged Astropy transformation obtains the ITRS position *and its frame-appropriate velocity* before evaluating the same geometric equations. An inertial GCRS velocity cannot be directly inserted as an ITRS range-rate velocity. The reference service fails closed when the required transformation is unavailable.

## Reference comparison API

`POST /api/navigation/epoch-ground-observation` takes one epoch, station and spacecraft state, with optional observed geometric range and rate. The endpoint includes reference-frame provenance, Earth-orientation status, and model limitations.

```json
{
  "epoch_utc":"2026-10-25T12:00:00Z",
  "frame":"ITRS",
  "station":{"latitude_deg":0,"longitude_deg":0,"height_m":0},
  "state":{"position_km":[6778.137,0,0],"velocity_km_s":[0,7.5,0]},
  "provenance":"synthetic"
}
```

`POST /api/navigation/epoch-reference-comparison` accepts a `reference_id`, `provenance` (`synthetic` or `external_unverified`), and a list of records each including an epoch, station, frame, satellite state and observed range/rate. It computes **observed minus modeled** residuals, RMS and bias. It rejects mixed coordinate frames. **This endpoint neither authenticates an external feed nor models light-time or atmosphere.** A real tracking range measurement generally requires a much richer measurement model before comparison has physical meaning.

Try the included reproducible six-record synthetic batch:

```bash
python -m apogee.v21_benchmark examples/v21_reference_synthetic_itrs.json docs/v21-benchmark.json
```

Then POST `examples/v21_reference_synthetic_itrs.json` to `/api/navigation/epoch-reference-comparison`. Expected results:

| Test quantity | Synthetic result |
|---|---:|
| Samples | 6 |
| Range residual RMS | **2.000 m** |
| Range residual mean | **0.000 m** |
| Range-rate residual RMS | **0.010 m/s** |
| Range-rate residual mean | **0.000 m/s** |

These values are **deliberately constructed test offsets**, not comparisons to real observations.

## Verification and limitations

- **461 automated tests passed** across completed grouped runs: 426 inherited + 35 new v2.1 tests. A single uninterrupted full-suite run was not completed. The v2.1 checks cover WGS84 equator/pole, ellipsoidal height, ENU azimuth/elevation, numerical range-rate differentiation, strict epochs/frames, missing optional Astropy, batch provenance and API validation.
- An actual **FastAPI TestClient** request to both new endpoints passed. The six-record comparison has known residuals.
- The **desktop and mobile offline Chromium checks** used backend-generated API responses, exercised the new tab, and explicitly checked the absent-Astropy error path. No JavaScript errors or horizontal overflow were detected. These checks **mock browser network transport**; a fully connected browser-to-local-server walkthrough remains an on-device check.
- **Not yet verified here:** Astropy GCRS→ITRS positive conversion, current IERS products and quality, real station position catalogs, authenticated measured tracking records, observation models with light-time/atmospheric corrections, SGP4/CelesTrak and JPL end-to-end comparison, calibrated navigation covariance, real flight accuracy.

Trusted external reference documentation used in design:

- [IERS / USNO Earth orientation products](https://maia.usno.navy.mil/products/bulletin-a) — Bulletin A and finals2000A.
- [Astropy IERS tables and Earth-orientation behavior](https://docs.astropy.org/en/stable/utils/iers.html).
- [Astropy EarthLocation](https://docs.astropy.org/en/stable/api/astropy.coordinates.EarthLocation.html) — terrestrial station positions and GCRS pos/vel APIs.
- [JPL Horizons manual](https://ssd.jpl.nasa.gov/horizons/manual.html) — vector state and range/range-rate output conventions. Horizons vector outputs are **not** automatically comparable to APOGEE's geometric terrestrial station observations.

**Next engineering step:** create a reproducible, authenticated dataset bundle containing station ITRF coordinates, IERS EOP snapshots, a suitable reference ephemeris and time-standard metadata; validate full GCRS/ITRS conversion and finite-light-time measurement residuals against independent software. This release does not assert that those checks have already been accomplished.
