"""Developer-only offline Chromium smoke check, not a live end-to-end test."""
import json,sys
from pathlib import Path
from fastapi.testclient import TestClient
from playwright.sync_api import sync_playwright
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from apogee.api import app
root=Path(__file__).resolve().parents[1]
client=TestClient(app)
normal=client.post('/api/propagate',json={}).json()
fault=client.post('/api/navigation/joint-integrity',json={'monte_carlo_runs':1}).json()
clean=client.post('/api/navigation/joint-integrity',json={'monte_carlo_runs':1,'fault_channel':'none'}).json()
presets=client.get('/api/presets').json()
responses={'/api/propagate':normal,'/api/presets':presets}
html=(root/'web/index.html').read_text().replace('<link rel="stylesheet" href="/static/style.css" />','').replace('<script src="/static/app.js" defer></script>','')
js_source=(root/'web/app.js').read_text()
css=(root/'web/style.css').read_text()
def fake_request(url,body=None):
    if '/api/navigation/joint-integrity' in url:
        try: selection=json.loads(body or '{}').get('fault_channel')
        except Exception:selection=None
        return clean if selection=='none' else fault
    return presets if '/api/presets' in url else normal
try:
 with sync_playwright() as pw:
  browser=pw.chromium.launch(executable_path='/usr/bin/chromium',headless=True,args=['--no-sandbox'])
  for label,width,height in [('desktop',1440,940),('mobile',390,844)]:
   page=browser.new_page(viewport={'width':width,'height':height},device_scale_factor=1)
   errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
   page.set_content(html,wait_until='load',timeout=20000)
   page.add_style_tag(content=css)
   page.expose_function('mockFetch',fake_request)
   page.evaluate("window.fetch=async (url, opts={})=>new Response(JSON.stringify(await window.mockFetch(String(url),opts.body||null)),{status:200,headers:{'Content-Type':'application/json'}})")
   page.add_script_tag(content=js_source)
   page.locator('#mode-joint-integrity').click(timeout=15000)
   page.wait_for_function("document.querySelector('#view-title').textContent.includes('fault isolation')",timeout=20000)
   page.wait_for_function("!document.querySelector('#joint-integrity-submit').disabled",timeout=20000)
   title=page.locator('#view-title').inner_text()
   print(label,'title',title,'errors',errors,'overflow',page.evaluate('document.documentElement.scrollWidth>innerWidth+2'))
   print('analysis',page.locator('#analysis-rows').inner_text()[:250].replace('\n',' | '))
   assert not errors
   assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+2')
   assert 'Joint navigation' in title
   page.screenshot(path=f'/mnt/data/APOGEE-v1.7-{label}-preview.png',full_page=True)
   if label=='desktop':
    page.locator('#ji-channel').select_option('none')
    page.locator('#joint-integrity-submit').click()
    page.wait_for_function("!document.querySelector('#joint-integrity-submit').disabled",timeout=20000)
    assert 'no alarms' in page.locator('#analysis-rows').inner_text().lower()
    print('clean rerun',page.locator('#analysis-rows').inner_text()[:200].replace('\n',' | '))
   page.close()
  browser.close()
finally:
 pass
