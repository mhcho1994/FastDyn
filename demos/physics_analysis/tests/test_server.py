import asyncio
from pathlib import Path
from aiohttp import web, ClientSession
from physics_demo.server import app_for
from test_data import truth_file
from physics_demo.common import write_json


def test_live_socket_ignores_control_and_replay_is_local(tmp_path):
    async def check():
        path=tmp_path/'truth.csv'
        truth_file(path,[(0,3,1)])
        write_json(tmp_path/'scenario.json',dict(waypoints_ne_m=[[10,-10],[20,10]]))
        app,ws_app=await app_for(path,None)
        runners=[web.AppRunner(app),web.AppRunner(ws_app)]
        sites=[]
        try:
            for runner in runners:
                await runner.setup()
                site=web.TCPSite(runner,'127.0.0.1',0)
                await site.start();sites.append(site)
            http_port=sites[0]._server.sockets[0].getsockname()[1]
            ws_port=sites[1]._server.sockets[0].getsockname()[1]
            async with ClientSession() as client:
                async with client.get(f'http://127.0.0.1:{http_port}/') as response:
                    html=await response.text()
                    assert '__VIEWER_CONFIG__' not in html
                    assert 'importmap' in html
                async with client.get(f'http://127.0.0.1:{http_port}/api/scenario') as response:
                    assert (await response.json())['waypoints_ne_m'] == [[10,-10],[20,10]]
                async with client.get(f'http://127.0.0.1:{http_port}/api/replay') as response:
                    assert response.status == 404
                async with client.ws_connect(f'http://127.0.0.1:{ws_port}/') as ws:
                    first=await ws.receive_json(timeout=2)
                    assert first['t'] == 0 and first['pz'] == 3
                    before=path.read_bytes()
                    await ws.send_json({'type':'reset','armed':True,'throttle':1})
                    again=await ws.receive_json(timeout=2)
                    assert again['t'] == 0
                    assert path.read_bytes() == before
        finally:
            for runner in runners:await runner.cleanup()
    asyncio.run(check())


def test_replay_replacement_is_uncached_and_reports_source(tmp_path):
    async def check():
        case = tmp_path/'case-003'
        case.mkdir()
        path = case/'replay.json'
        write_json(path, {'cases': [{'name': 'Zigzag / gyro bias and false rate'}]})
        app, _ = await app_for(replay=path)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '127.0.0.1', 0)
        try:
            await site.start()
            url = f'http://127.0.0.1:{site._server.sockets[0].getsockname()[1]}'
            async with ClientSession() as client:
                async with client.get(url+'/replay/') as response:
                    assert response.headers['Cache-Control'] == 'no-store'
                async with client.get(url+'/api/replay') as response:
                    assert (await response.json())['cases'][0]['name'] == 'Zigzag / gyro bias and false rate'
                    assert response.headers['Cache-Control'] == 'no-store'
                    assert response.headers['X-Physics-Replay-Source'] == 'case-003/replay.json'
                write_json(path, {'cases': [{'name': 'Zigzag / GPS spoofing'}]})
                async with client.get(url+'/api/replay') as response:
                    assert (await response.json())['cases'][0]['name'] == 'Zigzag / GPS spoofing'
                    assert response.headers['Cache-Control'] == 'no-store'
        finally:
            await runner.cleanup()
    asyncio.run(check())


def test_missing_replay_is_rejected_before_server_start(tmp_path):
    import pytest
    from physics_demo.server import serve
    with pytest.raises(FileNotFoundError, match='Replay file does not exist'):
        serve(replay=tmp_path/'0eplay.json')
