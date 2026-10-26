# APOGEE — Aerospace Physics & Orbital Guidance Engineering Environment

### Web edition — GitHub Pages + Render

APOGEE is now packaged as a **public website with a real Python physics engine**. Deploy the `site/` folder through the included GitHub Pages workflow, and deploy `apogee.api:app` using `render.yaml`. The same numerical engine powers the existing 26 mission/analysis modes. Before an API is connected, the Pages site shows a **labeled precomputed orbit preview**.

**[Complete GitHub + Render setup guide](DEPLOY_WEB.md)** · [Frontend connection settings](site/config.js) · [Python API](apogee/api.py) · [Website deployment workflow](.github/workflows/pages.yml)

The source bundle is ready for upload but has **not** been published to GitHub or Render automatically. Scientific validation status remains unchanged (partial reference-verification coverage).

---

# APOGEE v2.4 — Reproducible Software Verification & Release Gates

**Status in the packaged build environment: PARTIAL, not certified.** The
independent Kepler and numerical-convergence tests execute and pass, but the
full Vallado corpus is not bundled and the SGP4 / Astropy libraries could not
be installed here. The software does **not** claim real-flight accuracy.

## What changed since v2.3

- Introduced a fail-closed **five-part release gate** (analytical orbit accuracy,
  RK4 numerical convergence, *full* Vallado reference coverage, per-satellite
  SGP4 TEME comparison, Astropy GCRS/ITRS roundtrip).
- Coverage requires **33 satellite cases and 666 state vectors** from external
  text supplied by the user, not the four-case bundled excerpt.
- Added optional SHA-256 source pins, explicit provenance reporting,
  dependency/version manifest, per-check PASS / FAIL / SKIP and machine-readable
  `PARTIAL` status, plus clear 422 errors for invalid source pairs.
- Added CLI with **exit 0** for fully passed gate; **exit 2** for PARTIAL;
  **exit 1** for failed numerical/verification gate; **exit 3** for invalid input.
  `--allow-partial` is an explicit override for report generation only.
- Added an optional, explicit-download script and GitHub Actions verification
  workflow. Neither implies that upstream data are cryptographically signed.
- Extended the existing **Benchmarks v2.4** dashboard; all original 26 modes
  remain available. The two-file import now gives a strict gate disposition.

## Run the software-verification audit

```powershell
# Windows PowerShell from the extracted APOGEE directory
py -3 -m pip install -r requirements.txt
py -3 -m apogee.v24_validation --output APOGEE-v2.4-audit.json
# Expected exit code 2 (PARTIAL) without full corpus and frame dependencies.
```

Optional full-corpus setup (network is accessed **only** when you run the
explicit script):

```powershell
py -3 scripts_fetch_vallado.py --confirm-download
py -3 -m apogee.v24_validation --tle reference_inputs/SGP4-VER.TLE --vectors reference_inputs/tcppver.out --output full-audit.json
```

For reproducible source identity, save the printed SHA-256 hashes in an
independently reviewed record, then supply `--expected-tle-sha256` and
`--expected-vectors-sha256` on subsequent audit runs. Downloaded files alone
are *not* authenticated evidence.

The canonical verification suite exercises cases including deep-space and
resonant SGP4/SDP4 trajectories. **Even a full PASS is only software output
verification**, not satellite tracking validation. The roundtrip also tests
internal frame consistency rather than comparison against independent Earth
orientation measurements. Error-only SGP4 cases need additional dedicated
verification before this gate can be interpreted as full error-code coverage.

## New API

`POST /api/verification/v24` accepts:

```json
{"steps_per_orbit":240,"orbits":2}
```

Optional `tle_text`, `vector_text`, `expected_tle_sha256`, and
`expected_vector_sha256` fields are supported. The report contains `results`,
`release_gate`, `environment`, `reference_bundle`, `import_summary` and
`limitations`. The legacy v2.3 endpoints remain available.

## Verification limits

No external spacecraft observation data have been authenticated or compared.
Actual SGP4 vector propagation and Astropy Earth-orientation transformations
still require an integration run with those packages installed. The bundled
subset exists to demonstrate the importer and reporting, **not** to establish
full corpus coverage. The source-fetching script relies on a public repository
URL that may change over time.

---

# Historical v2.3 — Multiregime Flight-Dynamics Verification

**APOGEE — Aerospace Physics & Orbital Guidance Engineering Environment** is a Python / FastAPI spacecraft mission simulator, numerical orbital dynamics engine, and browser dashboard. Version 2.3 **extends rather than replaces** the v2.2 project, retaining its 26 existing operating modes.

## What changed in v2.3

- **Expanded Vallado software-verification fixture:** 28 publicly published TEME position/velocity vectors drawn from four orbital cases (NORAD 00005, 04632, 06251 and 08195), including a near-Earth drag example and deep-space eccentric trajectories. Original publication data is referenced below. These are *software reference outputs*, not measured spacecraft positions.
- **Individual SGP4 case audit:** reports per-satellite PASS/FAIL/SKIP and position/velocity residuals. Uses **WGS72** and time in **minutes since TLE epoch**, never mixes TEME with GCRS or APOGEE's idealized Cartesian axes. When `sgp4` is missing, cases are honestly SKIPPED.
- **Full Vallado corpus import:** accepts the two complete original text files, `SGP4-VER.TLE` and `tcppver.out`, retaining the first 69 characters of each TLE and parsing published vectors; error-only entries and diagnostic lines are counted separately. Computes SHA-256 fingerprints of user-supplied source bytes.
- **Optional Astropy frame verification:** performs a GCRS → ITRS → GCRS position-and-velocity roundtrip at a specified UTC epoch, with explicit SKIP if Astropy/Earth-orientation information is unavailable. **This roundtrip alone is not an independent ephemeris validation.**
- **Numerical Kepler baseline:** remains unchanged (independent Kepler equation vs RK4). Structured JSON audit reports distinguish PASS, FAIL, SKIP, and PARTIAL.
- **New v2.3 benchmarks interface** with official-corpus file import and an exportable audit report.

## Quick start — Windows

Extract the ZIP. Double-click `start_apogee_windows.bat`. The launcher installs dependencies listed in `requirements.txt` (requires internet access for first install); open `http://127.0.0.1:8000`. Select **Benchmarks v2.3**.

Manual launch:

```shell
python -m pip install -r requirements.txt
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
```

Optional high-precision coordinate conversions require additional dependencies:

```shell
python -m pip install -r requirements-reference.txt
```

## Run and save benchmarks

```shell
python -m apogee.v23_benchmark --output APOGEE-v2.3-audit.json
```

To test the **full** public Vallado corpus, obtain the upstream files independently and run:

```shell
python -m apogee.v23_benchmark --tle SGP4-VER.TLE --vectors tcppver.out --output full-vallado-audit.json
```

For an offline demo of the two-file importer, use `examples/v23_DEMO_subset.TLE` and `examples/v23_DEMO_subset.out` (only the 28 manually curated vectors, **not** the official complete corpus).

Official upstream data distributed with the `python-sgp4` project:

- [SGP4-VER.TLE](https://raw.githubusercontent.com/brandon-rhodes/python-sgp4/master/sgp4/SGP4-VER.TLE)
- [tcppver.out](https://raw.githubusercontent.com/brandon-rhodes/python-sgp4/master/sgp4/tcppver.out)
- [Upstream SGP4 source and tests](https://github.com/brandon-rhodes/python-sgp4)
- [CelesTrak SGP4 verification guide](https://celestrak.org/software/tutorials/sgp4-verification.php)

**Important:** The full upstream verification files are *not bundled* with v2.3. The bundled 28-state sample is manually curated from published outputs, not a complete cryptographically authenticated snapshot. The import code accepts full published source files, but **the full 700-line corpus has not been run in this build environment** because `sgp4` and upstream downloading were unavailable here.

## API

`POST /api/verification/v23`:

```json
{"steps_per_orbit": 240, "orbits": 2}
```

Returns the Kepler benchmark, four-case Vallado audit and optional Astropy roundtrip.

`POST /api/verification/vallado-import`:

```json
{"tle_text": "<contents of SGP4-VER.TLE>", "output_text": "<contents of tcppver.out>"}
```

Returns parsed case and sample counts, source fingerprints, per-case residuals and an aggregate status. Text limits: TLE 2 MB, vectors 8 MB. Import occurs in the local APOGEE API; do not expose this development server publicly.

The existing `POST /api/verification/independent` endpoint and other v2.2 routes remain available for backward compatibility.

## Test and benchmark interpretation

Run the physics/API test suite with:

```shell
python -m pytest -q tests
```

The independent Kepler analytical comparison tests the ideal two-body integrator. The Vallado corpus checks *implementation consistency* of the separately installed SGP4 library against published vectors. It does **not** independently validate APOGEE's Newtonian perturbation models or compare against tracked-satellite measurements. The Astropy check is an internal transformation roundtrip; a successful result does not establish correct real-world Earth-orientation data or external ephemeris agreement.

This project still needs live-spacecraft ephemeris comparisons, external station ranging, time-of-flight/atmospheric corrections, and fully independent frame-reference tests before claims of mission-analysis accuracy are justified. **Not for operational flight navigation.**

## Research progression

v2.4 priorities: acquire an authenticated, version-pinned upstream full verification corpus; execute positive SGP4 and Astropy/IERS comparisons in a reproducible environment; compare actual third-party satellite ephemerides with frame/time-scale compatibility proven before subtracting states.
