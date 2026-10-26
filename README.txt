APOGEE v2.5 CORS Hotfix
========================

1. Extract the ZIP to Downloads (not inside your GitHub repo).
2. Open Command Prompt in the extracted folder and run:

py -3 repair_v25_cors.py "C:\Users\Piano\OneDrive\Documents\GitHub\apogee-aerospace-lab"

3. In the APOGEE repo folder run:

   git status --short
   git add apogee/api.py tests/test_v25_cors_preflight.py
   git diff --cached --check
   git commit -m "Fix mission POST CORS preflight and add regression test"
   git push

If Git says nothing to commit, the local API was already fixed; continue to Render deployment.

4. Check GitHub Actions software verification. Then in Render, open the web service
   apogee-physics-api, inspect Events, and select Manual Deploy > Deploy latest commit
   if no successful auto-deploy of the corrected commit exists.

5. When Render says LIVE, run from the repo:

   py -3 scripts\check_live_api.py

Expected: Existing endpoint preflight HTTP 200 and allow-methods lists POST.

Note: This hotfix does not change scripts/v25_render_gate.py or render.yaml. If the
Render build itself fails due to the separate Vallado verification gate, review
its build log rather than weakening scientific validation to force deployment.
