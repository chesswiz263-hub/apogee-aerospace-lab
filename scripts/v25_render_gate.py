"""Execute full-corpus v2.4 audit during the Render v2.5 build.

Use from repository root AFTER installing requirements.txt and requirements-reference.txt.
Never silently convert a PARTIAL/SKIP audit into a PASS.
"""
from __future__ import annotations
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

def run(args: list[str]):
    print('APOGEE gate:', ' '.join(args),flush=True)
    return subprocess.run(args,check=False).returncode

def main():
    root=Path(__file__).resolve().parents[1]
    if not (root/'scripts_fetch_vallado.py').exists():
        sys.exit('Missing existing v2.4 scripts_fetch_vallado.py; run from APOGEE repo')
    if run([sys.executable,str(root/'scripts_fetch_vallado.py'),'--confirm-download']) != 0:
        sys.exit('Vallado reference download failed; aborting build')
    tle=root/'reference_inputs/SGP4-VER.TLE'
    vec=root/'reference_inputs/tcppver.out'
    if not (tle.exists() and vec.exists()):
        sys.exit('Missing Vallado reference files in reference_inputs/')
    report=root/'reports/v25-render-audit.json'
    report.parent.mkdir(parents=True,exist_ok=True)
    for f, env in [(tle, 'APOGEE_V25_TLE_SHA256'), (vec, 'APOGEE_V25_VECTORS_SHA256')]:
        digest = hashlib.sha256(f.read_bytes()).hexdigest()
        print(f'Verification reference {f.name}: sha256={digest}', flush=True)
        if os.environ.get(env) and os.environ[env].lower() != digest:
            sys.exit(f'Pinned reference hash mismatch for {f.name}')
    command=[sys.executable,'-m','apogee.v24_validation',
             '--tle',str(tle),'--vectors',str(vec),'--output',str(report)]
    if os.environ.get('APOGEE_V25_TLE_SHA256'):
        command += ['--expected-tle-sha256',os.environ['APOGEE_V25_TLE_SHA256']]
    if os.environ.get('APOGEE_V25_VECTORS_SHA256'):
        command += ['--expected-vectors-sha256',os.environ['APOGEE_V25_VECTORS_SHA256']]
    code=run(command)
    if not report.exists():
        sys.exit('Validation emitted no audit; aborting build')
    print('Vallado reference audit:',report,'exit code',code,flush=True)
    if code != 0:
        sys.exit('v2.4 audit is FAIL/PARTIAL; do not mark v2.5 verified')
    # The older runner uses its exit code to declare full success. Report is
    # kept for review because source and reference identity still need auditing.
    print('Software audit completed with return code zero. NOT flight-certified.')

if __name__=='__main__':
    main()
