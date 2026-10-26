"""Conservative upgrade from working APOGEE v2.5 to unified interplanetary pilot.

Usage (from extracted ZIP):
    py -3 install_unified_v25.py "C:\\Users\\Piano\\OneDrive\\Documents\\GitHub\\apogee-aerospace-lab"

Does not alter render.yaml, existing v2.4 routes, CORS, or numerical engines.
Makes timestamped backups of changed original files. Aborts on unrecognized
homepage structure rather than injecting an incompatible overlay.
"""
from __future__ import annotations
import shutil
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent


def patch_api(code: str) -> str:
    route_line = 'app.include_router(v25_router)'
    if code.count(route_line) != 1:
        raise RuntimeError('Expected exactly one existing APOGEE v2.5 router integration; API untouched.')
    if 'app.include_router(v25_flight_router)' not in code:
        code = code.replace(route_line,
            route_line+'\n# APOGEE v2.5 interplanetary pilot (additive; original 26 modes untouched)\n'
            'from apogee.v25_interplanetary import router as v25_flight_router\n'
            'app.include_router(v25_flight_router)',1)
    return code


def patch_solar(code: str) -> str:
    orbit_anchor='for(const n of Object.keys(E)) orbit(n);'
    date_anchor='const d=new Date(sim);'
    if code.count(orbit_anchor)!=1 or code.count(date_anchor)!=1:
        raise RuntimeError('Solar renderer differs from expected v2.5; refusing automatic modification')
    if 'APOGEE_V25_FLIGHT.drawTrail' not in code:
        code=code.replace(orbit_anchor, orbit_anchor+
            '\n if(window.APOGEE_V25_FLIGHT) window.APOGEE_V25_FLIGHT.drawTrail({ctx,project,visual});',1)
    if 'APOGEE_V25_FLIGHT.drawCraft' not in code:
        code=code.replace(date_anchor,
            'if(window.APOGEE_V25_FLIGHT) window.APOGEE_V25_FLIGHT.drawCraft({ctx,project,visual,sim});\n '+date_anchor,1)
    if 'src="apogee-v25-flight.js"' not in code:
        if code.count('</body>')!=1:
            raise RuntimeError('Unexpected solar viewer document ending')
        code=code.replace('</body>', '<script src="apogee-v25-flight.js"></script>\n</body>',1)
    return code


def patch_home(code: str) -> str:
    # Remove the installer-generated old floating link if it survived prior fixes.
    import re
    code=re.sub(r'<a\s+[^>]*id=["\']apogee-v25-solar-link["\'][^>]*>.*?</a>',
                '',code, flags=re.IGNORECASE|re.DOTALL)
    # The old bridge is replaced in its existing file; don't add duplicate imports.
    if 'apogee-v25-home.js' not in code:
        if code.count('</body>')!=1:
            raise RuntimeError('No unambiguous index </body> insertion point')
        code=code.replace('</body>','<script defer src="apogee-v25-home.js"></script>\n</body>',1)
    return code


def main(root: Path):
    root=root.expanduser().resolve()
    if not (root/'render.yaml').is_file() or not (root/'apogee/api.py').is_file():
        raise SystemExit(f'Not an APOGEE checkout: {root}')
    if not (root/'site/solar-system.html').is_file() or not (root/'site/index.html').is_file():
        raise SystemExit('Missing v2.5 Solar System viewer or homepage; unchanged')
    edits={root/'apogee/api.py':patch_api,
           root/'site/solar-system.html':patch_solar,
           root/'site/index.html':patch_home}
    staged={}
    for path,update in edits.items():
        staged[path]=update(path.read_text(encoding='utf8'))
    for rel in ('apogee/v25_interplanetary.py','site/apogee-v25-flight.js',
                'site/apogee-v25-home.js','tests/test_v25_interplanetary.py',
                'tests/test_reference_gate_unit.py','scripts/v25_render_gate.py',
                'scripts/check_live_api.py'):
        src=HERE/rel
        if not src.is_file():raise SystemExit(f'Upgrade package incomplete: {rel}')
        dest=root/rel
        # All destination changes staged and validated before first write.
        staged[dest]=src.read_bytes()
    diff={p:contents for p,contents in staged.items()
          if (p.read_bytes() if p.exists() else None) !=
          (contents.encode('utf8') if isinstance(contents,str) else contents)}
    if not diff:
        print('Already installed; no files changed.')
        return
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S')
    backup=root/'.v25-backup'/f'unified-flight-{stamp}'
    for path in diff:
        if path.is_file():
            b=backup/path.relative_to(root)
            b.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(path,b)
    for path,contents in diff.items():
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(contents.encode('utf8') if isinstance(contents,str) else contents)
        print('UPDATED:',path.relative_to(root))
    print('Backups:',backup)
    print('Next: py -3 -m pytest -q tests\\test_v25_interplanetary.py')
    print('Review docs BEFORE changing Render build or deploying.')

if __name__=='__main__':
    if len(sys.argv)!=2:
        raise SystemExit('Usage: py -3 install_unified_v25.py PATH_TO_APOGEE_REPO')
    main(Path(sys.argv[1]))
