"""One-time APOGEE v2.5 recovery patch.

Repairs GET-only CORS introduced by the add-on, removes its floating homepage
button, and adds the Solar System *inside* the mission visualization. Does not
alter existing mission data handling or physics algorithms.

Usage: py -3 repair_v25_homepage.py "C:\\...\\apogee-aerospace-lab"
"""
from __future__ import annotations
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent

def main(root):
    root=Path(root).expanduser().resolve()
    api=root/'apogee/api.py'
    html=root/'site/index.html'
    solar=root/'site/solar-system.html'
    if not all(p.is_file() for p in (api,html,solar)):
        sys.exit('Missing apogee/api.py, site/index.html or site/solar-system.html in '+str(root))
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S')
    back=root/'.v25-backup'/f'homepage-repair-{stamp}'
    back.mkdir(parents=True,exist_ok=False)
    for path in (api,html,solar):
        dest=back/path.relative_to(root)
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,dest)
    print('Backups saved to',back)

    # A restrictive outer CORSMiddleware blocks browser POST preflights.
    # Broaden only the v2.5 middleware layer to preserve old POST missions.
    code=api.read_text(encoding='utf8')
    marker='# APOGEE v2.5: public read-only CORS for the Pages frontend'
    if marker in code:
        before,after=code.split(marker,1)
        # Only alter the installer-owned layer, not other middleware definitions.
        after, count = re.subn(r'allow_methods=\["GET"\]',
                               'allow_methods=["GET","POST","OPTIONS"]', after, count=1)
        if not count and 'allow_methods=["GET","POST","OPTIONS"]' not in after:
            print('WARNING: custom v2.5 CORS block detected; inspect manually')
        code=before+marker+after
        print('v2.5 browser methods: GET + POST + OPTIONS')
    else:
        print('WARNING: installer-owned CORS not found; verify original CORS permits POST.')
    api.write_text(code,encoding='utf8')

    page=html.read_text(encoding='utf8')
    # Remove only our known injected pop-up link.
    page, removed=re.subn(r'<a\s+[^>]*id=["\']apogee-v25-solar-link["\'][^>]*>.*?</a>', '',page,flags=re.I|re.S)
    # Supports different attribute order (if manually customized).
    if not removed and 'apogee-v25-solar-link' in page:
        print('WARNING: floating link requires manual removal: apogee-v25-solar-link')
    page=page.replace('<script src="apogee-v25-home.js" defer></script>','')
    if '</body>' not in page.lower():
        sys.exit('Cannot safely insert integration script: site/index.html lacks </body>. Restore backups.')
    page=re.sub(r'</body>', '<script src="apogee-v25-home.js" defer></script>\n</body>',page,count=1,flags=re.I)
    html.write_text(page,encoding='utf8')
    js_source=HERE/'site/apogee-v25-home.js'
    js_target=root/'site/apogee-v25-home.js'
    if js_source.resolve()!=js_target.resolve():
        shutil.copy2(js_source,js_target)

    solar_html=solar.read_text(encoding='utf8')
    embedded_css='''<style id="apogee-v25-embedded-css">
body.apogee-v25-embedded .top {top:4px;left:4px;padding:4px 12px;background:transparent}
body.apogee-v25-embedded .top .brand {font-size:9px}
body.apogee-v25-embedded .top .brand strong{font-size:13px;margin:2px 0}
body.apogee-v25-embedded .top .status, body.apogee-v25-embedded .top .readout,
body.apogee-v25-embedded .left, body.apogee-v25-embedded .right{display:none}
body.apogee-v25-embedded .bottom{bottom:6px;left:50%;width:calc(100% - 16px);min-width:0;max-width:800px;padding:6px 10px;border-radius:8px;gap:7px;flex-wrap:wrap}
body.apogee-v25-embedded .bottom label {font-size:10px}
body.apogee-v25-embedded .bottom button,body.apogee-v25-embedded .bottom select,body.apogee-v25-embedded .bottom input {padding:4px 7px;font-size:11px}
body.apogee-v25-embedded .bottom input{width:126px}
@media(max-width:650px){body.apogee-v25-embedded .bottom label,body.apogee-v25-embedded #reset{display:none}}
</style>'''
    if 'apogee-v25-embedded-css' not in solar_html:
        solar_html=solar_html.replace('</head>',embedded_css+'\n</head>',1)
    embed='''<script id="apogee-v25-embedded-init">
if(new URLSearchParams(location.search).has('embedded')) {
 document.body.classList.add('apogee-v25-embedded');
 const chooser=document.createElement('select');
 chooser.title='Focus planet'; chooser.setAttribute('aria-label','Focus planet');
 ['Sun','Mercury','Venus','Earth','Mars','Jupiter','Saturn','Uranus','Neptune','Pluto'].forEach(n=>{
   const opt=document.createElement('option');opt.value=n;opt.textContent=n;
   if(n==='Earth')opt.selected=true;
   chooser.appendChild(opt);
 });
 chooser.addEventListener('change',()=>{
   document.querySelector('.bodyBtn[data-body="'+chooser.value+'"]').click();
 });
 document.querySelector('.bottom').prepend(chooser);
}
</script>'''
    if 'apogee-v25-embedded-init' not in solar_html:
        solar_html=solar_html.replace('</body>',embed+'\n</body>',1)
    solar.write_text(solar_html,encoding='utf8')
    print('Removed floating Solar System link:',bool(removed))
    print('Installed in-panel Solar System and preserved existing mission renderer.')
    print('Review diff; commit only after browser testing. API currently remains v2.4 until Render redeploys.')

if __name__=='__main__':
    if len(sys.argv)!=2:sys.exit('Usage: py -3 repair_v25_homepage.py PATH_TO_REPO')
    main(sys.argv[1])
