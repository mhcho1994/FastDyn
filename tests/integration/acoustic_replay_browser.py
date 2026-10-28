#!/usr/bin/env python3
"""Smoke-check a running acoustic replay in Chromium and save a comparison screenshot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit


def check(url: str, output: Path) -> dict:
    """Verify the real Rumoca assets, playback controls, two poses and local-only requests."""
    # Playwright is an optional browser-test dependency, not a replay dependency.
    from playwright.sync_api import sync_playwright

    origin = urlsplit(url)
    errors, failures, external = [], [], []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True, args=[
            '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--disable-dev-shm-usage',
        ])
        try:
            page = browser.new_page(viewport={'width': 1440, 'height': 1120}, device_scale_factor=1)
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.on('response', lambda response: failures.append(f'{response.status} {response.url}')
                    if response.status >= 400 else None)
            page.on('requestfailed', lambda request: failures.append(
                f'{request.failure} {request.url}'))
            page.on('request', lambda request: external.append(request.url)
                    if urlsplit(request.url).scheme in ('http', 'https')
                    and urlsplit(request.url).netloc != origin.netloc else None)
            page.goto(url, wait_until='networkidle')
            page.wait_for_function('window.sceneReady === true', timeout=60000)
            assert page.locator('.view canvas').count() == 2
            page.get_by_role('button', name='Resonance crossing', exact=True).click()
            assert abs(float(page.locator('#time').input_value()) - 10) < .01
            page.locator('#axis').select_option('1')
            page.locator('#axis').select_option('0')
            page.locator('#speed').select_option('2')
            page.get_by_role('button', name='Play', exact=True).click()
            page.wait_for_function('Number(document.getElementById("time").value) > 10.1')
            page.get_by_role('button', name='Pause', exact=True).click()
            page.get_by_role('button', name='Early ground contact', exact=True).click()
            assert 'CONTACT' in page.locator('.badge').nth(1).inner_text()
            page.locator('#camera').select_option('overview')
            page.locator('#camera').select_option('top')
            page.locator('#range').select_option('full')
            assert float(page.locator('#time').get_attribute('min')) < -20
            page.locator('#range').select_option('sweep')
            page.locator('#camera').select_option('scene')
            page.locator('#trail').select_option('0')
            page.locator('#trail').select_option('all')
            page.evaluate('window.renderFrame(12)')
            page.wait_for_timeout(300)
            output.parent.mkdir(parents=True, exist_ok=True)
            page.screenshot(path=str(output), full_page=True)
            readouts = page.locator('.readout').all_inner_texts()
            assert readouts[0] != readouts[1]
            assert not errors, errors
            assert not failures, failures
            assert not external, external
            return dict(url=url, screenshot=str(output), readouts=readouts,
                        page_errors=errors, failed_requests=failures, external_requests=external)
        finally:
            browser.close()


def main() -> None:
    """Run the optional browser check against an already running replay server."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8080/', help='Running local replay URL')
    parser.add_argument('--output', required=True, type=Path, help='Comparison screenshot path')
    args = parser.parse_args()
    print(json.dumps(check(args.url, args.output), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
