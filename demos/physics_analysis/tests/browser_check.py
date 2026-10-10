"""Optional browser acceptance check against a running demo server.

python tests/browser_check.py --url http://127.0.0.1:8080 --output /tmp/view.png
"""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright

parser=argparse.ArgumentParser()
parser.add_argument('--url',default='http://127.0.0.1:8080')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True,args=['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1440,'height':1100})
    errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    # Offline means every HTTP request must stay on the local server.
    remote=[]
    page.on('request',lambda r:remote.append(r.url) if r.url.startswith('http') and not r.url.startswith(args.url+'/') else None)
    page.goto(args.url+'/replay/')
    page.wait_for_function('window.sceneReady === true',timeout=60000)
    initial=page.locator('#clock').inner_text()
    page.locator('#camera').select_option('overview')
    page.locator('#trail').select_option('all')
    page.evaluate('window.renderFrame(Number(document.getElementById("time").max))')
    page.locator('#reset').click()
    assert page.locator('#clock').inner_text() != initial
    page.locator('#play').click()
    page.wait_for_timeout(300)
    page.locator('#play').click()
    selects=page.locator('.pane select')
    if selects.count()>1:
        options=selects.nth(1).locator('option').all_text_contents()
        if 'Zigzag / GPS spoofing' in options:
            selects.nth(1).select_option(label='Zigzag / GPS spoofing')
    page.locator('#camera').select_option('overview')
    page.evaluate('window.renderFrame(165)')
    assert page.locator('.plot canvas').count() == 4
    if 'Zigzag / GPS spoofing' in page.locator('.pane-top').all_inner_texts()[-1]:
        assert 'ATTACK ACTIVE' in page.locator('.overlay').nth(1).inner_text()
        assert 'Truth dwell 4/10' in page.locator('.metrics').nth(1).inner_text()
    page.screenshot(path=str(args.output))
    assert not errors,errors
    assert not remote,remote
    # Real Rumoca viewer shell, dynamically loaded route overlay and live WebSocket.
    page.goto(args.url+'/')
    if '/replay/' not in page.url:
        page.wait_for_function('document.querySelector("canvas") !== null')
        page.wait_for_timeout(1500)
        response=page.request.get(args.url+'/api/scenario')
        assert response.ok and len(response.json()['waypoints_ne_m']) == 10
        assert not errors,errors
        assert not remote,remote
    browser.close()
print('Browser replay, attack comparison, live Rumoca viewer and local-only assets: passed')
