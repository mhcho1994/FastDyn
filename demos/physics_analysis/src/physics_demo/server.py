"""Local HTTP/WS bridge. The simulator writes a file, never waits for a browser."""
from __future__ import annotations

import asyncio
import contextlib
import csv
import json
from pathlib import Path
import time
from urllib.parse import quote

from .common import DEMO, RUMOCA
from .data import decode_truth, live_frame

IMPORTMAP = '<script type="importmap">{"imports":{"three":"/vendor/three.module.js"}}</script>'


class TruthTail:
    def __init__(self, path):
        self.path = Path(path) if path else None
        self.position = 0
        self.partial = ''
        self.header = None
        self.latest = None
        self.sequence = 0
        self.updated = None
        self.error = None

    def poll(self):
        if self.path is None or not self.path.exists():
            return
        if self.path.stat().st_size < self.position:
            self.position, self.partial, self.header = 0, '', None
            self.latest = None
        with self.path.open() as f:
            f.seek(self.position)
            chunk = f.read(1024*1024)
            self.position = f.tell()
        lines = (self.partial+chunk).split('\n')
        self.partial = lines.pop()
        for line in lines:
            if not line:
                continue
            values = next(csv.reader([line]))
            if self.header is None:
                self.header = values
                continue
            if len(values) != len(self.header):
                raise ValueError('incomplete/corrupt truth record')
            frame = decode_truth(dict(zip(self.header, values)))
            if self.latest and frame['t'] <= self.latest['t']:
                raise ValueError('live truth clock moved backwards; start a new bridge for a new run')
            self.sequence += 1
            self.latest = live_frame(frame, self.sequence)
            self.updated = time.monotonic()


async def app_for(truth=None, replay=None, ws_port=8081):
    from aiohttp import web
    app = web.Application()
    tail = TruthTail(truth)
    clients = set()

    async def index(request):
        if truth is None:
            raise web.HTTPFound('/replay/')
        source = (RUMOCA / 'packages/rumoca-web/viz/viewer.html').read_text()
        config = dict(status_title='FastDyn FMU truth (read only)', show_armed=False,
                      controls={'keyboard': [{'keys': 'Mouse', 'action': 'Orbit / wheel zoom'}],
                                'gamepad': [{'keys': '—', 'action': 'Observer only'}]})
        source = source.replace('__WS_PORT__', str(ws_port)).replace('__VIEWER_CONFIG__', json.dumps(config)).replace('__DEBUG__', 'false')
        source = source.replace('<head>', '<head>'+IMPORTMAP)
        source = source.replace('<body>', '<body><a style="position:fixed;right:16px;top:8px;z-index:10;color:white" href="/replay/">Replay / analysis</a>')
        return web.Response(text=source, content_type='text/html')

    async def scene(request):
        base = RUMOCA / 'examples/interactive/quadrotor/quadrotor_scene.js'
        extension = DEMO / 'web/scene_extension.js'
        return web.Response(text=base.read_text()+'\n'+extension.read_text(), content_type='text/javascript')

    async def replay_page(request):
        page = (DEMO/'web/index.html').read_text()
        if truth is None:
            page = page.replace('<a href="/">Live Rumoca viewer ↗</a>', '<span>Offline recording</span>')
        return web.Response(text=page, content_type='text/html', headers={'Cache-Control': 'no-store'})

    async def dataset(request):
        if replay is None:
            raise web.HTTPNotFound(text='No replay selected. Export a recording and restart with --replay FILE.')
        path = Path(replay).resolve()
        return web.FileResponse(path, headers={
            'Cache-Control': 'no-store',
            'X-Physics-Replay-Source': quote(f'{path.parent.name}/{path.name}'),
        })

    async def scenario_info(request):
        path = Path(truth).parent / 'scenario.json' if truth else None
        return web.json_response(json.loads(path.read_text()) if path and path.exists() else {})

    async def status(request):
        return web.json_response(dict(sequence=tail.sequence, t=tail.latest['t'] if tail.latest else None,
            age_wall_s=time.monotonic()-tail.updated if tail.updated else None, error=tail.error))

    async def websocket(request):
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4096)
        await ws.prepare(request)
        clients.add(ws)

        async def send_latest():
            try:
                while not ws.closed:
                    if tail.latest is not None:
                        value = dict(tail.latest)
                        age = time.monotonic()-tail.updated
                        value['input_message'] = tail.error or f'Read-only FMU truth · last sample {age:.1f}s ago'
                        await asyncio.wait_for(ws.send_json(value), timeout=1)
                    await asyncio.sleep(.05)
            except (ConnectionError, TimeoutError):
                await ws.close()
        sender = asyncio.create_task(send_latest())
        try:
            # Intentional read-only protocol: discard all viewer control messages.
            async for _ in ws:
                pass
        finally:
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError, ConnectionError, TimeoutError):
                await sender
            clients.discard(ws)
        return ws

    async def poll_tail():
        while True:
            try:
                previous = tail.sequence
                tail.poll()
                if tail.sequence > previous:
                    tail.error = None
            except (ValueError, OSError) as exc:
                tail.error = str(exc)
            await asyncio.sleep(.05)

    async def lifecycle(_app):
        task = asyncio.create_task(poll_tail())
        yield
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        for ws in list(clients):
            await ws.close()

    app.cleanup_ctx.append(lifecycle)
    app.router.add_get('/', index)
    app.router.add_get('/scene.js', scene)
    app.router.add_get('/replay/', replay_page)
    app.router.add_get('/api/replay', dataset)
    app.router.add_get('/api/status', status)
    app.router.add_get('/api/scenario', scenario_info)
    app.router.add_static('/web/', DEMO/'web', follow_symlinks=False)
    app.router.add_static('/vendor/', DEMO/'web/vendor', follow_symlinks=False)
    app.router.add_static('/assets/', RUMOCA/'examples/assets', follow_symlinks=False)
    # Same application on a second port: Rumoca's unmodified shell connects to WS /.
    ws_app = web.Application()
    ws_app.router.add_get('/', websocket)
    return app, ws_app


def serve(truth=None, replay=None, port=8080, ws_port=8081):
    from aiohttp import web
    if truth is None and replay is None:
        raise ValueError('serve requires --truth and/or --replay')
    if replay is not None:
        replay = Path(replay).resolve()
        if not replay.is_file():
            raise FileNotFoundError(f'Replay file does not exist: {replay}')
    if port == ws_port:
        raise ValueError('HTTP and WebSocket ports must differ')

    async def start():
        app, ws_app = await app_for(truth, replay, ws_port)
        runners = [web.AppRunner(app), web.AppRunner(ws_app)]
        try:
            for runner, p in zip(runners, (port, ws_port)):
                await runner.setup()
                await web.TCPSite(runner, '127.0.0.1', p).start()
            if truth is not None:
                print(f'Live Rumoca viewer: http://127.0.0.1:{port}/', flush=True)
            if replay is not None:
                print(f'Replay file: {Path(replay).resolve()}', flush=True)
                print(f'Offline replay: http://127.0.0.1:{port}/replay/', flush=True)
                print('Reload an already-open replay tab after changing the recording.', flush=True)
            await asyncio.Event().wait()
        finally:
            for runner in runners:
                await runner.cleanup()
    try:
        asyncio.run(start())
    except KeyboardInterrupt:
        pass
