# APOGEE v2.5 — Satellite Operations + Solar System

An **additive** upgrade for `chesswiz263-hub/apogee-aerospace-lab` v2.4: do not replace the existing repository or delete its 26 modes.

## What is implemented

- **New interactive Solar System** at `site/solar-system.html`: Sun, eight major planets, Pluto (illustrative), selected major moons (illustrative); drag/zoom, date and time controls, body focus, and an optional ISS state readout. Uses JPL Table 1 fitted orbital elements for major planets, valid only 1800–2050. Earth row is the Earth–Moon barycenter. No JPL Horizons/DE440 or moon accuracy claim. Orbit-distance compression and inflated sizes are explicitly visual-only.
- **FastAPI routes:** `GET /api/v25/capabilities`, `/api/v25/solar-system?at=2026-10-25T12:00:00Z` (8 planets + separately labeled illustrative Pluto/moons), `/api/v25/satellites/25544/state?at=...` and `/api/v25/verification`. Retrieves CelesTrak OMM JSON, caches an individual catalog query for at least 2 hours on the local worker and limits uncached queries to 24/hour, propagates it with SGP4/WGS72 into TEME, and uses Astropy for ITRS, GCRS and Earth subpoint when available. Old TLE-only workflows cannot represent new catalog IDs >99999. **This is not flight-certified software.**
- **Verification:** Run the existing v2.4 Vallado importer against the full reference corpus using `scripts/v25_render_gate.py`. GitHub Actions publishes the audit. Render build must be changed to call this gate after all deps are installed. A passing software reference audit checks implementation consistency, not independently observed orbital truth.

## Install locally from this ZIP

1. Extract this ZIP anywhere **outside** your project checkout.
2. Clone the current APOGEE repo (`git clone https://github.com/chesswiz263-hub/apogee-aerospace-lab.git`) or open your existing local checkout.
3. From the extracted v2.5 package, run:

   ```powershell
   py -3 install_v25.py "C:\path\to\apogee-aerospace-lab"
   cd "C:\path\to\apogee-aerospace-lab"
   py -3 -m pip install -r requirements.txt -r requirements-reference.txt -r requirements-v25.txt
   py -3 -m pytest -q tests
   py -3 -m uvicorn apogee.api:app --reload
   ```

4. Open `http://127.0.0.1:8000/api/v25/capabilities` and, separately, `site/solar-system.html` in your browser. The static viewer works without API; the satellite panel needs your API origin.
5. Commit changes and push to `main`; the GitHub Pages workflow will include `site/solar-system.html` **only if it publishes the full `site/` directory**. Otherwise adjust the existing workflow's copy step to include the new HTML file. The installer adds a visible Solar System link to `site/index.html`.

## Render changes (review before editing)

Update the existing Render service's `buildCommand` in `render.yaml` **without replacing other existing configuration**, e.g. if the current command is `pip install -r requirements.txt`, replace its command with:

```sh
pip install -r requirements.txt -r requirements-reference.txt -r requirements-v25.txt && python scripts/v25_render_gate.py
```

Keep the existing `startCommand` intact (normally `uvicorn apogee.api:app --host 0.0.0.0 --port $PORT`). Commit the YAML change. On each Render build, the old v2.4 script downloads the published Vallado reference text and the gate runs the full importer. It intentionally fails the deployment if audit is PARTIAL or FAIL. This depends on the upstream fixture remaining available. If Render cannot fetch it, fix the reference packaging/availability rather than bypass the validation.

Optional Render environment variables:

- `APOGEE_V25_AUDIT=reports/v25-render-audit.json` (default)
- `APOGEE_V25_CACHE_DIR=/tmp/apogee-v25-celestrak` (default, ephemeral on Render)
- `APOGEE_V25_TLE_SHA256` / `APOGEE_V25_VECTORS_SHA256` (optional reference hash pins; strongly recommended after independent source review)

**CORS:** The Solar System is hosted on GitHub Pages while the API runs on Render. The installer adds an origin-scoped, GET-only CORS middleware for `https://chesswiz263-hub.github.io` (no credentials). Confirm that any existing CORS middleware does not conflict.

### Manual verification

```powershell
py -3 scripts_fetch_vallado.py --confirm-download
py -3 -m apogee.v24_validation --tle reference_inputs/SGP4-VER.TLE --vectors reference_inputs/tcppver.out --output reports/v25-render-audit.json
```

Before calling the gate scientifically verified, pin the verified input SHA-256 values using the existing `--expected-tle-sha256` and `--expected-vectors-sha256` flags, audit the full Vallado error-code suite, and measure independent ephemeris/tracking residuals against observation data. The v2.5 gate does **not** yet achieve these latter tests.

## Verification data and model sources

- NASA/JPL planetary elements: https://ssd.jpl.nasa.gov/planets/approx_pos.html
- SGP4/OMM use: https://pypi.org/project/sgp4/
- CelesTrak GP formats: https://celestrak.org/NORAD/documentation/gp-data-formats.php
- Astropy satellite frame guidance: https://docs.astropy.org/en/stable/coordinates/satellites.html

## Known limitations and follow-up work

- This addon is designed against the publicly documented v2.4 API layout; it has **not** been executed against your complete GitHub source checkout in this environment. Review the installer diff, especially `apogee/api.py` and `site/index.html`, in your checkout.
- This package did not run full SGP4/ITRS tests in the creation environment, where `sgp4` and `astropy` were unavailable. The CI and Render steps are required for that.
- Live CelesTrak records are not themselves truth observations. Cache policies must respect CelesTrak's updated update-cycle limits; persistent storage is preferable for scaled Render instances/restarts.
- True full-Solar-System n-body propagation, DE440/441 ephemerides, accurate moon states, and interplanetary missions are future separate upgrades. The 3D-projected viewer is an orrery, not a numerically integrated multi-body gravity simulator.
