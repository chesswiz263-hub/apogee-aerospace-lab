# APOGEE — publish the real simulator on the web

**Architecture:** GitHub Pages hosts `site/` (HTML, CSS, JS). Render runs `apogee.api:app` (Python, NumPy, SciPy, SGP4) through HTTPS. The browser calls `https://YOUR-RENDER-SERVICE.onrender.com/api/...` directly. Never attempt to run Python on GitHub Pages; GitHub Pages is a static host.

**Status:** Repository ready for GitHub; it has not been pushed to the user's GitHub account, and no Render service has been created. No public URL is active yet.

## 1. Create and push your GitHub repository

Create a **public** empty GitHub repository named `apogee-aerospace-lab` (or a name you prefer). Do not initialize it with a README, because this package already has one. From the folder containing this file:

```bash
git init -b main
git add .
git commit -m "Deploy APOGEE v2.4 as web application"
git remote add origin https://github.com/YOUR-USERNAME/apogee-aerospace-lab.git
git push -u origin main
```

Replace `YOUR-USERNAME` with your GitHub username. Do not put GitHub tokens or Render credentials in the repository.

## 2. Publish the GitHub Pages frontend

In the new repository, open **Settings → Pages → Build and deployment → Source → GitHub Actions**. The included `.github/workflows/pages.yml` publishes `site/` on pushes to `main` (and can be started manually with **Actions → Deploy APOGEE website to GitHub Pages → Run workflow**). The deployed project-site address will normally resemble:

`https://YOUR-USERNAME.github.io/apogee-aerospace-lab/`

**This is only an example until GitHub reports an actual deployment URL.** The initial GitHub Pages page shows an honestly labeled precomputed orbit preview until a physics API is connected.

## 3. Deploy the Python API to Render

On [Render](https://dashboard.render.com/), create **New → Blueprint** and select the same GitHub repository. The included `render.yaml` defines a Python web service named `apogee-physics-api`:

- Build: `pip install -r requirements.txt`
- Start: `uvicorn apogee.api:app --host 0.0.0.0 --port $PORT`
- Health check: `/api/health`
- CORS: the blueprint initially allows GitHub Pages origins. For a more restrictive deployment, set `APOGEE_ALLOWED_ORIGINS=https://YOUR-USERNAME.github.io`, and remove `APOGEE_ALLOWED_ORIGIN_REGEX`.

Alternatively, create a regular **Web Service** with those build/start commands. Select a suitable plan; the free plan may spin down between requests and heavy simulation modes can be slow or exceed free-tier resources. Verify the actual service URL shown in Render; **do not assume the requested service name creates a guaranteed URL**.

Open `https://YOUR-ACTUAL-RENDER-SERVICE.onrender.com/api/health` in a browser. It must return a JSON object with `"application": "APOGEE"` and `"status": "ok"`.

## 4. Link the website with Python

On your published GitHub Pages site, paste the **actual Render HTTPS origin** (no `/api` suffix) into **CONNECT API**. It is saved in your browser's local storage. When it says **CONNECTED**, run **Orbit → PROPAGATE ORBIT**, then **Transfer → DESIGN TRANSFER** to verify calculations are live.

For a fixed default URL for all visitors, edit `site/config.js` to set `apiBaseUrl` to the actual Render origin, commit and push; the GitHub Pages workflow redeploys. Note that this URL is public, not a secret. The browser's saved preference overrides this default; **CLEAR** removes it.

## 5. Run locally before publishing

From the repository root, after installing requirements:

```bash
python -m pip install -r requirements.txt
python -m pip install httpx pytest
# Terminal 1: Python API
python -m uvicorn apogee.api:app --host 127.0.0.1 --port 8000
# Terminal 2: GitHub Pages-style static frontend
python -m http.server 5500 --directory site
```

Open `http://127.0.0.1:5500/`, enter `http://127.0.0.1:8000` in CONNECT API, and check the simulator. HTTP is permitted only for localhost/127.0.0.1.

Or use `http://127.0.0.1:8000/` to run the original single-server interface.

## 6. Tests and boundaries

```bash
python -m pytest -q tests/test_web_deploy.py tests/test_api.py tests/test_v24.py
node --check site/app.js
python -m apogee.v24_validation --allow-partial --output validation.json
```

The `.github/workflows/apogee-validation.yml` workflow runs broader regressions. The **SGP4/Astropy checks remain separate from real-flight validation** and may need reference data or optional dependencies; a PASS in numerical tests does not imply that APOGEE has been validated against real satellite tracking observations.

## Troubleshooting

- Website opens but says **DEMO**: no backend URL connected; the orbit is a static precomputed numerical sample.
- Website says **OFFLINE**: the Render service URL may be wrong, sleeping, failing startup, or blocked by CORS. Check Render logs and `/api/health` first.
- Browser says **CORS blocked**: set `APOGEE_ALLOWED_ORIGINS` to the exact GitHub Pages site origin (`https://USERNAME.github.io`), not the repository path, and remove or configure the origin regex.
- Simulation slow/timeouts: use smaller sample settings or a Render tier with sufficient CPU/memory; advanced Monte Carlo and full verification are compute-intensive.
- The browser's saved API URL can be reset with **CLEAR**; never paste private API keys into the public website settings.
