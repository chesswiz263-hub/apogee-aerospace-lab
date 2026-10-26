"""Isolated Chromium smoke test using actual backend-computed API results.
The page runs without a networked browser-to-server session.
"""
import sys
from pathlib import Path
from urllib.parse import urlsplit
from fastapi.testclient import TestClient
from playwright.sync_api import sync_playwright
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root))
from apogee.api import app
html=(root/'web/index.html').read_text().replace('<script src="/static/app.js" defer></script>','').replace('<link rel="stylesheet" href="/static/style.css" />','')
html=html.replace('<head>','<head><base href="https://apogee.test/" />')
css=(root/'web/style.css').read_text();js=(root/'web/app.js').read_text()
client=TestClient(app)
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
        page.wait_for_function("document.querySelector('#vb-analytic-status').textContent==='PASS'",timeout=30000)
        status=page.locator('#vb-overall').inner_text()
        other=page.locator('#vb-sgp-status').inner_text()
        print(label,'Kepler',page.locator('#vb-analytic-status').inner_text(),'SGP4',other,'overall',status,
              'errors',errors,'horizontal overflow',page.evaluate('document.documentElement.scrollWidth>innerWidth+2'))
        assert status in {'PARTIAL','PASS'} and other in {'SKIP','PASS'}
        assert not errors and not page.evaluate('document.documentElement.scrollWidth>innerWidth+2')
        page.screenshot(path=str(root.parent/f'APOGEE-v2.2-{label}-preview.png'),full_page=True)
        if label=='desktop':
            page.locator('#vb-steps').select_option('480')
            page.locator('#vb-run').click()
            page.wait_for_function("document.querySelector('#vb-position').textContent.includes('m') && !document.querySelector('#vb-run').disabled",timeout=30000)
            value=page.locator('#vb-position').inner_text();print('finer-resolution rerun',value)
            assert float(value.split(' ')[0]) < 0.25
        page.close()
    browser.close()
