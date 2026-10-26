# APOGEE v2.5 — Unified Solar System & Interplanetary Flight Pilot

This is an **additive pilot upgrade** for the working `chesswiz263-hub/apogee-aerospace-lab` repository. It is **not** a new repository, not a replacement physics engine, and not a certified flight dynamics system.

## What changes

- `site/solar-system.html` **uses its existing planetary canvas** to draw the interplanetary spacecraft **in the same projection**, AU units and approximate J2000 heliocentric ecliptic frame as all planets. The trail and moving craft are not a separate thumbnail or floating simulation.
- The new `site/apogee-v25-flight.js` adds **Launch spacecraft** (any pair of the eight planets; user-defined flight days), plus **Track ISS (current)** using SGP4 and Astropy when available. Both features are inside the central Solar System viewport.
- `site/apogee-v25-home.js` replaces the **old duplicate view controls, floating thumbnail and duplicate overlay buttons** with **one** selector: Solar System + Flights / Earth Mission.
- `apogee/v25_interplanetary.py` adds `/api/v25/interplanetary/transfer` (Lambert departure + high-tolerance DOP853 Sun-only propagation), and `/api/v25/satellites/{catalog_id}/heliocentric` (CelesTrak OMM → SGP4 TEME → Astropy heliocentric ecliptic).
- All **26 existing modes and endpoints are left alone**, and remain visible in Earth Mission view. The Earth-centered modes are **not** incorrectly added as heliocentric tracks; full bidirectional frame-aware visualization of each historic mode is a later milestone.
- `scripts/v25_render_gate.py` retains the original pinned Vallado files and compares the output vectors without strict checksum rejection of intentional artificial test records.

## Scientific limitations — please read

- A *Lambert transfer* calculates ideal impulsive heliocentric boundary conditions. It does **not** launch from a planet's surface, solve atmospheric ascent, estimate rocket mass/fuel, leave a planet's sphere of influence with patched conics, or capture at the destination.
- The Sun-only propagation is compared to its **own Lambert target**. Near-zero numerical miss is only an **internal solver consistency** test, *not* validation of predicted planet arrival in the actual Solar System.
- The planet locations use NASA/JPL fitted approximate elements, valid only 1800–2050; **Earth is the Earth–Moon barycenter**. Moon and Pluto positions in the current viewer are illustrative. A later milestone should use JPL SPICE/Horizons DE ephemerides and full n-body integration.
- Orbital-element **age** is shown for SGP4/CelesTrak. Stale data do not imply accuracy. The ISS marker is a snapshot for the current time and is drawn coincident with Earth at astronomical scale, not falsely enlarged in heliocentric distance.
- Astropy's Earth-orientation transformations may degrade using stale offline IERS tables. The API **refuses to fabricate** a heliocentric satellite position if Astropy is absent.
- The rendering canvas is 3D-*projected* (rotatable/pitch/zoom); not yet a photorealistic GPU-powered WebGL scene.

## Safe installation on Windows

**1. Create a protected upgrade branch from your working checkout** (GitHub Desktop Current Branch → New branch `apogee-v25-unified`), or Command Prompt:

```cmd
git switch -c apogee-v25-unified
```

**2. Extract this ZIP to a separate folder**, then open Command Prompt inside that extracted folder and run:

```cmd
py -3 install_unified_v25.py "C:\Users\Piano\OneDrive\Documents\GitHub\apogee-aerospace-lab"
```

The installer expects the original v2.5 `api.py`, `index.html` and `solar-system.html`, validates its patch anchors, backs up modified files to `.v25-backup/unified-flight-TIMESTAMP`, and **does not touch** `render.yaml`, CORS settings, existing numerical modules, or application startup commands. Running twice has no further effect.

**3. Run tests** (the complete local suite may encounter the Windows SciPy DLL policy restriction seen previously; CI on Linux is the full software gate):

```cmd
cd C:\Users\Piano\OneDrive\Documents\GitHub\apogee-aerospace-lab
py -3 -m pip install -r requirements.txt -r requirements-reference.txt -r requirements-v25.txt pytest
py -3 -m pytest -q -p no:cacheprovider tests\test_v25_interplanetary.py tests\test_reference_gate_unit.py
```

**4. Review and commit only the intended files**:

```cmd
git diff --check
git status --short
git add apogee/api.py apogee/v25_interplanetary.py site/index.html site/solar-system.html site/apogee-v25-home.js site/apogee-v25-flight.js scripts/v25_render_gate.py scripts/check_live_api.py tests/test_v25_interplanetary.py tests/test_reference_gate_unit.py
git commit -m "APOGEE v2.5 unified planetary scene and interplanetary flight pilot"
git push -u origin apogee-v25-unified
```

Open a **pull request**; let software verification and the separate Vallado validation finish green before merging to `main`. A dedicated **staging Render service** is preferable for a truly safe live endpoint check. Without staging, do not merge until all CI gates pass.

## Render — enable Astropy and validate independently

Look at the **existing** `render.yaml`; preserve every service name, environment, and the existing `startCommand`. For the existing Python web service, include all three requirement files and the new scientific gate in `buildCommand`:

```yaml
buildCommand: pip install -r requirements.txt -r requirements-reference.txt -r requirements-v25.txt && python scripts/v25_render_gate.py
```

Do not copy an entire example `render.yaml` over your existing configuration. If there are additional important existing build steps, **retain them** and append these requirements/gate accordingly. Ensure `requirements-v25.txt` continues to contain `astropy>=6.0,<9` and `sgp4>=2.25,<3`.

The **reference gate may still fail on the full official corpus**. The parser and standalone unit tests were run locally; the full downloaded reference files were not accessible in the packaging environment, so **a Vallado PASS is not claimed**. Inspect GitHub Action's `reports/v25-render-audit.json` and fix mismatches without weakening thresholds or skipping cases to force a deployment.

## What to verify after deploying

1. GitHub Pages homepage: mission designer modes remain visible, the central Solar System view renders and there is **only one** view selector.
2. Enter Solar System + Flights; select Earth → Mars, approximately 260 days; click **Launch spacecraft**. A cyan trajectory and spacecraft move within the planetary canvas as the simulation clock advances. This requires the new Render API to have successfully deployed.
3. Switch to Earth Mission and run an original Orbit simulation. The original track and telemetry must still update; `POST` preflight must return HTTP 200.
4. In Solar System, click **Track ISS (current)**. The backend must have Astropy installed; the marker is Earth-near at solar scale and is time-limited to the measurement epoch. Check `element_age_days` and warnings.
5. Run `py -3 scripts\check_live_api.py` and inspect `GET https://apogee-physics-api.onrender.com/api/v25/capabilities` for `astropy_available: true`.
6. Test the actual new flight endpoint in a browser (replace `2026-10-25` with a desired date):

   `https://apogee-physics-api.onrender.com/api/v25/interplanetary/transfer?origin=Earth&destination=Mars&departure=2026-10-25T00%3A00%3A00Z&flight_days=260&samples=80`

7. Verify the **Vallado gate** separately in GitHub Actions. The published audit should show `release_gate: PASS` before production deployment.

### Sources for model design

- JPL approximate planet elements: https://ssd.jpl.nasa.gov/planets/approx_pos.html
- CelesTrak current GP / OMM format: https://celestrak.org/NORAD/documentation/gp-data-formats.php
- Astropy TEME/satellite conversions: https://docs.astropy.org/en/stable/coordinates/satellites.html

### Rollback

If local tests or webpage behavior fail, **do not merge** the branch. `git switch main` restores the working baseline. If you already merged, use GitHub's **Revert** for the upgrade commit and redeploy both Pages and Render from the known-good revision. Installer backups in `.v25-backup` provide additional recovery copies; do not stage or commit them.
