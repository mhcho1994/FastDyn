#!/usr/bin/env python3
"""Export and serve paired acoustic recordings using Rumoca's quadrotor scene."""

from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from urllib.parse import unquote, urlsplit

from fastdyn import paths


SCHEMA = 'fastdyn-acoustic-replay-v1'


def _asset_path(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise FileNotFoundError(relative)
    return path


class ReplayHandler(SimpleHTTPRequestHandler):
    """Serve only the selected recording, viewer code and Rumoca scene assets."""

    def __init__(self, *args, replay: Path, web: Path, rumoca: Path, **kwargs):
        self.replay, self.web, self.rumoca = replay, web, rumoca
        super().__init__(*args, **kwargs)

    def _resolve(self) -> Path:
        route = unquote(urlsplit(self.path).path)
        if route in ('/', '/replay/', '/index.html'):
            return self.web / 'index.html'
        if route == '/api/replay':
            return self.replay
        if route == '/scene.js':
            return self.rumoca / 'examples/interactive/quadrotor/quadrotor_scene.js'
        for prefix, root in [('/web/', self.web), ('/vendor/', self.web / 'vendor'),
                             ('/assets/', self.rumoca / 'examples/assets')]:
            if route.startswith(prefix):
                return _asset_path(root, route[len(prefix):])
        raise FileNotFoundError(route)

    def send_head(self):
        """Return an opened allowlisted file; never expose directory listings."""
        try:
            path = self._resolve()
            handle = path.open('rb')
        except (OSError, ValueError):
            self.send_error(404, 'Replay resource not found')
            return None
        self.send_response(200)
        mime = 'text/javascript' if path.suffix == '.js' else self.guess_type(str(path))
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(path.stat().st_size))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        return handle

    def log_message(self, format, *args):
        """Keep normal asset requests quiet; report HTTP errors on stderr."""
        if len(args) > 1 and str(args[1]).startswith(('4', '5')):
            super().log_message(format, *args)


def create_server(replay: Path, port: int, rumoca: Path | None = None) -> ThreadingHTTPServer:
    """Bind a read-only localhost replay server after checking its inputs."""
    replay = replay.resolve()
    with replay.open(encoding='utf-8') as handle:
        data = json.load(handle)
    if data.get('schema') != SCHEMA or len(data.get('cases', [])) != 2:
        raise ValueError('Expected a paired acoustic replay; run the export command first')
    if rumoca is None:
        root = paths.repo_root()
        if root is None:
            raise ValueError('Set FASTDYN_REPO_ROOT or pass --rumoca-root')
        rumoca = root / 'third_party/common/rumoca'
    rumoca = rumoca.resolve()
    for relative in ['examples/interactive/quadrotor/quadrotor_scene.js',
                     'examples/assets/models/drone.glb']:
        if not (rumoca / relative).is_file():
            raise FileNotFoundError(f'Missing Rumoca scene asset: {rumoca / relative}')
    web = Path(__file__).with_name('replay_web')
    handler = partial(ReplayHandler, replay=replay, web=web, rumoca=rumoca)
    return ThreadingHTTPServer(('127.0.0.1', port), handler)


def main() -> None:
    """Run the export or localhost playback command."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('export', help='Convert baseline/exposure run directories')
    export.add_argument('baseline', type=Path, help='Zero-pressure run directory')
    export.add_argument('exposure', type=Path, help='Positive-pressure run directory')
    export.add_argument('--output', required=True, type=Path, help='New replay JSON file')
    export.add_argument('--hz', type=float, default=50, help='Pose replay rate (10–200; default 50)')
    export.add_argument('--no-telemetry', action='store_true', help='Omit optional MAVLink overlays')
    serve = commands.add_parser('serve', help='Open a local Rumoca replay server')
    serve.add_argument('--replay', required=True, type=Path, help='Exported replay JSON file')
    serve.add_argument('--port', type=int, default=8080, help='Local HTTP port (default 8080)')
    serve.add_argument('--rumoca-root', type=Path, help='Override the checkout\'s Rumoca directory')
    args = parser.parse_args()
    try:
        if args.command == 'export':
            # NumPy and optional pymavlink are only needed when converting recordings.
            from replay_data import export_replay
            data = export_replay(args.baseline, args.exposure, args.output,
                                 args.hz, not args.no_telemetry)
            print(f'Replay: {args.output.resolve()}')
            for case in data['cases']:
                print(f"  {case['label']}: {case['metrics']['source_samples']} source samples"
                      f" → {len(case['frames'])} replay poses")
                for notice in case['notices']:
                    print(f'  {notice}')
        else:
            with create_server(args.replay, args.port, args.rumoca_root) as server:
                print(f'Recording: {args.replay.resolve()}', flush=True)
                print(f'Rumoca acoustic replay: http://127.0.0.1:{server.server_port}/', flush=True)
                server.serve_forever()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as exc:
        parser.exit(1, f'Acoustic replay: {exc}\n')


if __name__ == '__main__':
    main()
