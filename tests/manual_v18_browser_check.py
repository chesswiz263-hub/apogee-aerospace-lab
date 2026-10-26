"""Standalone offline Chromium interaction test. No live HTTP end-to-end claim."""
import json,sys
from pathlib import Path
from fastapi.testclient import TestClient
from playwright.sync_api import sync_playwright
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from apogee.api import app
root=Path(__file__).resolve().parents[1]
client=TestClient(app)
normal=client.post('/api/propagate',json={}).json()
calibrated=client.post('/api/navigation/adaptive',json={}).json()
uncalibrated=client.post('/api/navigation/adaptive',json={'estimate_thrust_scale':False}).json()
presets=client.get('/api/presets').json()
html=(root/'web/index.html').read_text().replace('<link rel="stylesheet" href="/static/style.css" />','').replace('<script src="/static/app.js" defer></script>','')
css=(root/'web/style.css').read_text()
js_source=(root/'web/app.js').read_text()
def fake_request(url,body=None):
    if '/api/navigation/adaptive' in url:
        return calibrated if json.loads(body or '{}').get('estimate_thrust_scale',True) else uncalibrated
    return presets if '/api/presets' in url else normal
with sync_playwright() as pw:
    browser=pw.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
    for label,width,height in [('desktop',1440,940),('mobile',390,844)]:
        page=browser.new_page(viewport={'width':width,'height':height},device_scale_factor=1)
        errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        page.set_content(html,wait_until='load',timeout=20000)
        page.add_style_tag(content=css)
        page.expose_function('mockFetch',fake_request)
        page.evaluate("window.fetch=async (url,opts={})=>new Response(JSON.stringify(await window.mockFetch(String(url),opts.body||null)),{status:200,headers:{'Content-Type':'application/json'}})")
        page.add_script_tag(content=js_source)
        page.locator('#mode-adaptive').click()
        page.wait_for_function("document.querySelector('#view-title').textContent.includes('Adaptive navigation')",timeout=20000)
        page.wait_for_function("!document.querySelector('#adaptive-submit').disabled",timeout=20000)
        metrics=page.locator('#analysis-rows').inner_text()
        print(label,'title',page.locator('#view-title').inner_text())
        print(label,'metrics',metrics[:280].replace('\n',' | '))
        print(label,'errors',errors,'overflow',page.evaluate('document.documentElement.scrollWidth>innerWidth+2'))
        assert 'engine scale' in metrics.lower()
        assert not errors
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+2')
        page.screenshot(path=f'/mnt/data/APOGEE-v1.8-{label}-preview.png',full_page=True)
        if label=='desktop':
            page.locator('#av-calibrate').uncheck()
            page.locator('#adaptive-submit').click()
            page.wait_for_function("!document.querySelector('#adaptive-submit').disabled",timeout=20000)
            assert '1.0000' in page.locator('#analysis-rows').inner_text()
            print('calibration-off rerun','passed')
        page.close()
    browser.close()
