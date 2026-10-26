"""Narrow APOGEE v2.5 browser POST/CORS repair.

Run from an extracted release folder:
    py -3 repair_v25_cors.py "C:\\path\\to\\apogee-aerospace-lab"

Edits only the installer-owned _V25CORS middleware line in apogee/api.py.
Adds a regression test, without changing mission physics, site, or render config.
If the line already allows POST, no API edit is needed; redeploy on Render.
"""
from __future__ import annotations

import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

MARKER = '# APOGEE v2.5: public read-only CORS for the Pages frontend'
EXPECTED_METHODS = 'allow_methods=["GET", "POST", "OPTIONS"]'
HERE = Path(__file__).resolve().parent


def main(root: Path) -> None:
    root = root.expanduser().resolve()
    api = root / 'apogee' / 'api.py'
    if not api.is_file() or not (root / 'render.yaml').is_file():
        raise SystemExit(f'Not an APOGEE source checkout: {root}')

    code = api.read_text(encoding='utf-8')
    if code.count(MARKER) != 1:
        raise SystemExit('Expected exactly one v2.5 CORS marker in apogee/api.py; no changes made.')
    marker_pos = code.index(MARKER)
    tail = code[marker_pos:]
    match = re.search(r'(?m)^app\.add_middleware\(\s*_V25CORS\b[^\n]*$', tail)
    if not match or match.start() > 550:
        raise SystemExit('Could not locate the one-line v2.5 CORS middleware; no changes made.')

    old_line = match.group()
    methods = re.search(r'allow_methods\s*=\s*\[[^\]\n]*\]', old_line)
    if not methods:
        raise SystemExit('v2.5 CORS allow_methods setting was not found; no changes made.')
    current = methods.group()
    method_names = re.findall(r'["\']([A-Z]+)["\']', current)
    if method_names not in (["GET"], ["GET", "POST", "OPTIONS"]):
        raise SystemExit(f'Custom methods configuration {current!r}; no changes made.')

    if method_names == ["GET"]:
        new_line = old_line[:methods.start()] + EXPECTED_METHODS + old_line[methods.end():]
        output = code[:marker_pos + match.start()] + new_line + code[marker_pos + match.end():]
        backups = root / '.v25-backup'
        backups.mkdir(exist_ok=True)
        backup = backups / ('api-before-cors-hotfix-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.py')
        shutil.copy2(api, backup)
        api.write_text(output, encoding='utf-8')
        print('UPDATED: apogee/api.py now allows GET, POST, OPTIONS; backup:', backup)
    else:
        print('ALREADY FIXED LOCALLY: v2.5 CORS already allows GET, POST, OPTIONS.')
        print('The HTTP 400 from Render means the live API has not deployed this configuration.')

    source_test = HERE / 'tests' / 'test_v25_cors_preflight.py'
    dest_test = root / 'tests' / 'test_v25_cors_preflight.py'
    if not source_test.is_file():
        raise SystemExit('Missing test in extracted hotfix folder.')
    if dest_test.exists() and dest_test.read_bytes() != source_test.read_bytes():
        raise SystemExit('Existing custom tests/test_v25_cors_preflight.py differs; review it manually.')
    if not dest_test.exists():
        dest_test.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_test, dest_test)
        print('ADDED: tests/test_v25_cors_preflight.py')
    else:
        print('Regression test already present.')
    print('Next: commit updated files, verify GitHub Actions, and deploy latest commit on Render.')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('Usage: py -3 repair_v25_cors.py PATH_TO_APOGEE_REPO')
    main(Path(sys.argv[1]))
