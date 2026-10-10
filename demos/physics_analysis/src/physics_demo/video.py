"""Optional deterministic-time capture of the local replay page."""
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from urllib.parse import urlparse


def render_video(url, output, start, end, fps=24):
    from playwright.sync_api import sync_playwright
    if urlparse(url).hostname not in ('localhost', '127.0.0.1', '::1'):
        raise ValueError('video capture expects the local demo replay server')
    if not all(math.isfinite(x) for x in (start, end, fps)) or end <= start or not 1 <= fps <= 60:
        raise ValueError('require finite start < end and 1 <= fps <= 60')
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    if not shutil.which('ffmpeg'):
        raise FileNotFoundError('ffmpeg is required for video export')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='physics-video-', dir=output.parent) as temp:
        movie = Path(temp)/'movie.mp4'
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
            try:
                page = browser.new_page(viewport={'width':1440, 'height':1080}, device_scale_factor=1)
                errors = []
                page.on('pageerror', lambda e: errors.append(str(e)))
                page.goto(url)
                page.wait_for_function('window.sceneReady === true', timeout=60000)
                limits = page.evaluate('[Number(document.getElementById("time").min), Number(document.getElementById("time").max)]')
                if start < limits[0] or end > limits[1]:
                    raise ValueError(f'video time must stay inside replay range {limits}')
                process = subprocess.Popen(['ffmpeg', '-v', 'error', '-n', '-f', 'image2pipe', '-vcodec', 'png',
                    '-framerate', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(movie)], stdin=subprocess.PIPE)
                try:
                    for frame in range(math.ceil((end-start)*fps)):
                        page.evaluate('(t) => window.renderFrame(t)', start+frame/fps)
                        process.stdin.write(page.screenshot(type='png'))
                    process.stdin.close()
                    if process.wait(timeout=60):
                        raise RuntimeError('ffmpeg failed')
                    if errors:
                        raise RuntimeError('; '.join(errors))
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill(); process.wait()
            finally:
                browser.close()
        # Exclusive creation preserves pre-existing files, even after a concurrent export.
        with output.open('xb') as dest, movie.open('rb') as src:
            shutil.copyfileobj(src, dest)
    print(output)
