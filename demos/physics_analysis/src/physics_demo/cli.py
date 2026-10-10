from __future__ import annotations
import argparse
from pathlib import Path

from .common import OUT, scenario, write_json


def main():
    parser = argparse.ArgumentParser(description='FastDyn physics analysis demo (optional tooling)')
    sub = parser.add_subparsers(dest='command', required=True)
    b = sub.add_parser('build', help='build pristine and observed demo FMUs in a fresh directory')
    b.add_argument('--output', type=Path, default=OUT/'waypoint-attacks/build')
    b.add_argument('--compiler', type=Path)
    r = sub.add_parser('run', help='run existing Copter 4.6.2 against the demo FMU')
    r.add_argument('scenario', help='baseline, magnetic, wind, gps-spoof, gyro or a TOML file')
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--build-dir', type=Path, default=OUT/'waypoint-attacks/build')
    r.add_argument('--port-base', type=int, default=15550)
    r.add_argument('--timeout', type=float, default=900, help='wall-clock watchdog seconds')
    r.add_argument('--boot-offset-s', type=float, default=0)
    r.add_argument('--duration', type=float, help='override waypoint mission timeout in simulation seconds')
    r.add_argument('--prepare-only', action='store_true')
    c = sub.add_parser('campaign', help='sequential paired or seeded runs with common-clock replay')
    c.add_argument('kind', choices=['attacks', 'monte-carlo'])
    c.add_argument('--output', type=Path, required=True)
    c.add_argument('--build-dir', type=Path, default=OUT/'waypoint-attacks/build')
    c.add_argument('--seed', type=int, default=42)
    c.add_argument('--count', type=int, default=3)
    c.add_argument('--port-base', type=int, default=15550)
    c.add_argument('--prepare-only', action='store_true')
    e = sub.add_parser('export', help='export run directories or original-study truth CSV files')
    e.add_argument('inputs', nargs='+', type=Path)
    e.add_argument('--output', required=True, type=Path)
    s = sub.add_parser('serve', help='localhost live Rumoca viewer and offline Three.js replay')
    s.add_argument('--truth', type=Path, help='tail a running demo truth.csv, may not exist yet')
    s.add_argument('--replay', type=Path, help='exported replay.json')
    s.add_argument('--port', type=int, default=8080)
    s.add_argument('--ws-port', type=int, default=8081)
    v = sub.add_parser('video', help='capture the local replay to MP4 (optional Playwright + ffmpeg)')
    v.add_argument('--url', default='http://127.0.0.1:8080/replay/')
    v.add_argument('--output', required=True, type=Path)
    v.add_argument('--start', required=True, type=float)
    v.add_argument('--end', required=True, type=float)
    v.add_argument('--fps', default=24, type=int)
    m = sub.add_parser('_mission', help='internal mission helper')
    m.add_argument('run', type=Path)
    m.add_argument('--connect', required=True)
    args = parser.parse_args()
    if args.command == 'build':
        from .build import build
        build(args.output, args.compiler)
    elif args.command == 'run':
        from .runner import run, prepare
        spec = scenario(args.scenario)
        if args.duration is not None:
            spec['duration_s'] = args.duration
        if args.prepare_only:
            prepare(spec, args.output, args.build_dir, args.port_base, args.boot_offset_s)
        else:
            raise SystemExit(0 if run(spec, args.output, args.build_dir, args.port_base, args.timeout, args.boot_offset_s) else 1)
    elif args.command == 'campaign':
        from .runner import campaign_cases, run, prepare
        cases = campaign_cases(args.kind, args.seed, args.count)
        args.output.mkdir(parents=True, exist_ok=False)
        write_json(args.output/'campaign.json', dict(kind=args.kind, seed=args.seed, cases=cases))
        paths, passed = [], True
        for i, spec in enumerate(cases):
            path = args.output / f'case-{i:03d}'
            if args.prepare_only:
                prepare(spec, path, args.build_dir, args.port_base)
            else:
                passed = run(spec, path, args.build_dir, args.port_base) and passed
                if (path/'truth.csv').exists():
                    paths.append(path)
        if paths:
            from .data import export
            export(paths, args.output/'replay.json')
        raise SystemExit(0 if passed else 1)
    elif args.command == 'export':
        from .data import export
        export(args.inputs, args.output)
    elif args.command == 'serve':
        from .server import serve
        serve(args.truth, args.replay, args.port, args.ws_port)
    elif args.command == 'video':
        from .video import render_video
        render_video(args.url, args.output, args.start, args.end, args.fps)
    elif args.command == '_mission':
        from .mission import execute
        raise SystemExit(execute(args.run, args.connect))
