"""Apply the APOGEE v2.5 additive patch to an existing local repo.

Run: python install_v25.py /path/to/apogee-aerospace-lab
Creates backups of patched original API and website index under .v25-backup/.
Does not alter render.yaml, which must be reviewed before changing deployment.
"""
from __future__ import annotations
import shutil
import sys
from pathlib import Path

source=Path(__file__).resolve().parent

def backup_and_write(path: Path, contents: str, root: Path):
    current=path.read_text(encoding='utf8')
    if current==contents:return False
    b=root/'.v25-backup'/path.relative_to(root)
    if not b.exists():
        b.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,b)
    path.write_text(contents,encoding='utf8')
    return True

def main(root: Path):
    root=root.expanduser().resolve()
    if not (root/'apogee/api.py').is_file() or not (root/'render.yaml').is_file():
        sys.exit(f'Not an APOGEE source checkout: {root}')
    files=['apogee/v25_solar.py','apogee/v25_operations.py','site/solar-system.html',
           'scripts/v25_render_gate.py','.github/workflows/apogee-v25-verify.yml',
           'requirements-v25.txt','tests/test_v25_addon.py','tests/test_v25_frames.py','APOGEE_V25_README.md']
    for rel in files:
        from_file=source/rel
        to_file=root/rel
        to_file.parent.mkdir(parents=True,exist_ok=True)
        if not from_file.is_file():
            sys.exit(f'Missing upgrade file: {from_file}. Extract the complete v2.5 ZIP before installing.')
        # The upgrade may already be unpacked inside the target repository.
        # Windows will not allow shutil.copy2() to copy a file onto itself.
        if from_file.resolve() == to_file.resolve():
            continue
        if to_file.is_file() and to_file.read_bytes()!=from_file.read_bytes():
            # Refuse overwrites of pre-existing customized v2.5 files.
            sys.exit(f'Target already has different content: {rel}; resolve conflict manually')
        shutil.copy2(from_file,to_file)
    api=root/'apogee/api.py'
    old=api.read_text('utf8')
    patch='\n# APOGEE v2.5 additive operations routes\nfrom apogee.v25_operations import router as v25_router\napp.include_router(v25_router)\n'
    if 'app.include_router(v25_router)' not in old:
        backup_and_write(api,old.rstrip()+patch,root)
    # Allow the GitHub Pages origin to request GET satellite/planet data.
    # This is an additive, origin-scoped layer; it does not permit credentials.
    api_content=api.read_text('utf8')
    cors_patch=('\n# APOGEE v2.5: public read-only CORS for the Pages frontend\n'
                'from fastapi.middleware.cors import CORSMiddleware as _V25CORS\n'
                'app.add_middleware(_V25CORS, allow_origins=["https://chesswiz263-hub.github.io"], '
                'allow_credentials=False, allow_methods=["GET"], allow_headers=["*"])\n')
    if '# APOGEE v2.5: public read-only CORS' not in api_content:
        backup_and_write(api,api_content.rstrip()+cors_patch,root)
    page=root/'site/index.html' 
    if page.is_file():
        old=page.read_text('utf8')
        link='<a href="solar-system.html" aria-label="Explore the entire Solar System in APOGEE v2.5" id="apogee-v25-solar-link" style="position:fixed;z-index:99999;bottom:16px;right:16px;padding:12px 18px;border:1px solid #4d94c6;background:#0c2845;color:#eef7ff;border-radius:11px;font:600 14px system-ui;text-decoration:none">☀ Solar System v2.5</a>'
        if 'apogee-v25-solar-link' not in old:
            if '</body>' not in old:
                print('WARNING: site/index.html has no </body> tag. Open site/solar-system.html directly.')
            else:
                backup_and_write(page,old.replace('</body>',link+'\n</body>',1),root)
    print('Installed v2.5 additive files into:',root)
    print('Next: pip install -r requirements-v25.txt; pytest -q tests')
    print('See APOGEE_V25_README.md for safe Render build-command update.')

if __name__=='__main__':
    if len(sys.argv)!=2:
        sys.exit('Usage: python install_v25.py /path/to/apogee-aerospace-lab')
    main(Path(sys.argv[1]))