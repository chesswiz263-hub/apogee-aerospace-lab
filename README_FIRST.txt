APOGEE v2.5 HOMEPAGE REPAIR (safe, reversible bridge)
=====================================================

Purpose
-------
1. Repairs the installer-added GET-only CORS middleware to also allow POST/OPTIONS.
   This is needed for browser preflight requests to existing mission simulation APIs.
2. Removes the v2.5 floating Solar System link from the homepage.
3. Inserts the *existing* planetary viewer inside the main mission visualization
   viewport, using the homepage's real canvas geometry. Defaults to Solar System.
4. Keeps the original 26-mode Earth-centered simulation renderer, trajectory,
   mission controls, and telemetry untouched and continuously running.
5. Adds a visible in-panel switch to Mission view, where the original renderer
   reappears. The planetary view includes a mission preview inset when canvas
   capture is available.

Important physics limitation
---------------------------
The v2.5 planet display uses approximate JPL elements (1800-2050).
This quick repair does not assert that Earth-centered spacecraft trajectory
arrays are heliocentric trajectories. That requires mapping each existing
mission mode's state vectors and epoch into a consistent reference frame,
which requires modifications to the current site's renderer and data model.
It is NOT a validated multi-body gravitational spacecraft propagator.

Install from Windows Command Prompt
-----------------------------------
1. Extract the zip to a NEW folder. Do not overwrite your entire repository.
2. Open Command Prompt inside the extracted folder.
3. Run exactly:

py -3 repair_v25_homepage.py "C:\Users\Piano\OneDrive\Documents\GitHub\apogee-aerospace-lab"

4. Inspect changed files and test on GitHub Pages after push.
5. From your repository Command Prompt:

git status --short
git add apogee/api.py site/index.html site/solar-system.html site/apogee-v25-home.js
git commit -m "Repair APOGEE v2.5 mission requests and embed Solar System in homepage"
git push

Backend CORS change needs a fresh Render deployment. On Render, watch service
logs and confirm the latest commit was actually deployed, then use Ctrl+F5
on the GitHub Pages homepage.

Backups
-------
The installer saves timestamped backups of the three modified files in:
.v25-backup/homepage-repair-YYYYMMDD-HHMMSS/
Nothing inside .v25-backup needs to be committed.

Check the POST preflight in Windows Command Prompt:

curl.exe -i -X OPTIONS "https://apogee-physics-api.onrender.com/api/orbit" ^
  -H "Origin: https://chesswiz263-hub.github.io" ^
  -H "Access-Control-Request-Method: POST" ^
  -H "Access-Control-Request-Headers: content-type"

That sample endpoint might not exist in your project. CORS middleware still
responds to preflights before API route matching; the key result is HTTP 200
and Access-Control-Allow-Methods containing POST (not just GET).

If the visual overlay fails to mount, the old mission view stays available.
This package is designed as an emergency recovery / homepage bridge, not a
finished unified heliocentric mission solver.
