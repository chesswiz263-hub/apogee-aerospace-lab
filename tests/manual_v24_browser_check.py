"""Isolated Chromium check using actual FastAPI backend outputs routed in-memory.
Not a genuine browser-to-networked-server test.
"""
import sys, json
from pathlib import Path
from urllib.parse import urlsplit
from fastapi.testclient import TestClient
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root))
from apogee.api import app
from apogee.v23_validation import CURATED
html=(root/'web/index.html').read_text().replace('<script src="/static/app.js" defer></script>','').replace('<link rel="stylesheet" href="/static/style.css" />','')
html=html.replace('<head>','<head><base href="https://apogee.test/" />')
css=(root/'web/style.css').read_text();js=(root/'web/app.js').read_text()
client=TestClient(app)
fixture=json.loads(CURATED.read_text())
tle='\n'.join(s['tle_line1']+'\n'+s['tle_line2']+' 0.0 4320.0 360.0' for s in fixture['satellites'])
vectors='\n'.join(f'{s["catalog_id"]} xx\n'+'\n'.join(' '.join(str(n) for n in (v['minutes'],*v['position_km'],*v['velocity_km_s'])) for v in s['samples']) for s in fixture['satellites'])
with sync_playwright() as pw:
    browser=pw.chromium.launch(headless=True,executable_path='/usr/bin/chromium',args=['--no-sandbox','--disable-dev-shm-usage'])
    for label,viewport in [('desktop',{'width':1440,'height':960}),('mobile',{'width':390,'height':844})]:
        page=browser.new_page(viewport=viewport,device_scale_factor=1)
        errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        def respond(route):
            req=route.request;path=urlsplit(req.url).path
            if not path.startswith('/api/'):
                route.fulfill(status=404,body='unmocked');return
            try:
                res=client.post(path,content=req.post_data,headers={'Content-Type':'application/json'}) if req.method=='POST' else client.get(path)
                route.fulfill(status=res.status_code,body=res.content,headers={'content-type':'application/json','access-control-allow-origin':'*'})
            except Exception as e:route.fulfill(status=500,body=str(e))
        page.route('https://apogee.test/**',respond)
        page.goto('about:blank');page.set_content(html,wait_until='domcontentloaded')
        page.add_style_tag(content=css);page.add_script_tag(content=js)
        page.locator('#mode-v22').click(timeout=15000)
        page.locator('#v22-form').wait_for(state='visible')
        page.wait_for_function("document.querySelector('#vb-gate').textContent==='PARTIAL'",timeout=30000)
        assert page.locator('#vb-analytic-status').inner_text()=='PASS'
        assert page.locator('#vb-convergence').inner_text().endswith('PASS')
        assert page.locator('#vb-coverage').inner_text().endswith('SKIP')
        assert page.locator('#vb-sgp-status').inner_text()=='SKIP'
        assert page.locator('#vb-frames').inner_text()=='SKIP'
        assert not errors,errors
        assert not page.evaluate('document.documentElement.scrollWidth>innerWidth+2'),'horizontal overflow'
        page.screenshot(path=str(root.parent/f'APOGEE-v2.4-{label}-preview.png'),full_page=True)
        if label=='desktop':
            page.locator('#vb-tle-file').set_input_files({'name':'SGP4-VER.TLE','mimeType':'text/plain','buffer':tle.encode()})
            page.locator('#vb-output-file').set_input_files({'name':'tcppver.out','mimeType':'text/plain','buffer':vectors.encode()})
            page.locator('#vb-import').click()
            page.wait_for_function("document.querySelector('#vb-import-status').textContent.includes('28 TEME vectors')",timeout=30000)
            assert page.locator('#vb-gate').inner_text()=='PARTIAL'
            assert 'SKIP' in page.locator('#vb-coverage').inner_text()
        print(label, 'Kepler PASS, convergence PASS, coverage SKIP, strict gate PARTIAL, JS errors',len(errors))
        page.close()
    browser.close()
