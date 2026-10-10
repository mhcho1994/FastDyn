from __future__ import annotations

from datetime import datetime, timezone
import math
import os
from pathlib import Path
import random
import shutil
import signal
import subprocess
import sys
import tomllib

from .common import DEMO, ROOT, OUT, read_json, write_json, sha256, revision, scenario, validate_scenario, attack_parameters


def prepare(spec, output, build_dir, port_base=15550, boot_offset_s=0.0):
    import tomli_w
    validate_scenario(spec)
    if not math.isfinite(boot_offset_s):
        raise ValueError('boot offset must be finite')
    if not 1024 <= port_base <= 65530:
        raise ValueError('port base must be between 1024 and 65530')
    output = Path(output).resolve()
    build_dir = Path(build_dir).resolve()
    fmu = build_dir / 'observed/PhysicsAnalysis_Copter.fmu'
    build_meta = read_json(build_dir / 'build.json')
    if build_meta.get('profile') != 'waypoint-attacks-v1':
        raise ValueError('build a fresh waypoint attack FMU; legacy builds are incompatible')
    if sha256(fmu) != build_meta['observed_fmu_sha256']:
        raise ValueError('observed FMU does not match build provenance')
    # Refuse reuse even if the previous run failed.
    output.mkdir(parents=True, exist_ok=False)
    with (ROOT / 'configs/copter462.toml').open('rb') as f:
        config = tomllib.load(f)
    model = config['FMU']['models']['quadrotor']
    model.update(model='PhysicsAnalysis.Copter', model_file=str(DEMO / 'models/PhysicsAnalysis/Copter.mo'),
                 source_roots=[str(DEMO / 'models')], output=str(fmu.parent), auto_build=False)
    config['FMU']['auto_build'] = False
    model['parameters'].update(attack_parameters(spec))
    config['Rumoca']['enabled'] = False
    machine = config['Machine']
    machine.update(monitor_port=port_base, qmp_socket=str(output / 'qmp.sock'), log_file=str(output / 'qemu.log'))
    # Core runtime also understands these environment variables; both are set.
    banks = [config['Memory']['main'], *config['Memory'].get('ram1', [])]
    for bank in banks:
        bank['memory_file'] = str(output / (bank['id'] + '.ram'))
    config['Run']['cwd'] = str(ROOT)
    mavproxy = ROOT / 'fastdyn-env/bin/mavproxy.py'
    if not mavproxy.is_file():
        raise FileNotFoundError(f'MAVProxy unavailable: {mavproxy}')
    config['Run']['processes'] = {
        'mavproxy': dict(enabled=True, quiet=True, cwd=str(output),
                        command=[str(mavproxy), '--daemon', f'--master=udpout:127.0.0.1:{port_base+1}',
                                 f'--out=udpout:127.0.0.1:{port_base+2}', '--logfile=' + str(output/'mavlink.tlog')]),
        'mission': dict(enabled=True, start_delay_sec=0, terminate_run_on_exit=True,
                        command=[sys.executable, str(DEMO / 'demo.py'), '_mission', str(output),
                                 '--connect', f'udpin:127.0.0.1:{port_base+2}']),
    }
    config_path = output / 'fastdyn.toml'
    config_path.write_text(tomli_w.dumps(config))
    parameter_source = ROOT / 'virtuals/physics/flight_controllers/courbet/mavlink/copter_init.param'
    shutil.copyfile(parameter_source, output / 'parameters.param')
    write_json(output / 'scenario.json', spec)
    write_json(output / 'run.json', dict(schema_version=1, name=spec['name'], scenario=spec,
        created_utc=datetime.now(timezone.utc).isoformat(), fastdyn_commit=revision(ROOT),
        firmware='ArduCopter 4.6.2', firmware_sha256=sha256(ROOT / config['CPU']['cpu0'][0]['binary']),
        build=build_meta, parameters_sha256=sha256(output / 'parameters.param'),
        demo_source_sha256={str(p.relative_to(DEMO)): sha256(p)
                            for p in sorted((DEMO / 'src/physics_demo').glob('*.py'))},
        clock='FMU/QEMU simulation seconds', boot_offset_s=boot_offset_s,
        clock_mapping='truth_sim_s = time_boot_s + boot_offset_s; verify for other firmware',
        position_frame='NWU', quaternion='body FLU to world NWU; wxyz',
        electrical_model=False, current_proxy="0.42 + 42.5908 * mean normalized squared rotor speed",
        gps_yaw="unavailable; no ground-truth heading supplied to GPS_INPUT"))
    env = os.environ.copy()
    # Do not let a previous shell FMU override replace the recorded artifact.
    for key in tuple(env):
        if key.startswith('FASTDYN_FMI_'):
            env.pop(key)
    env.update(PHYSICS_DEMO_TRUTH_PATH=str(output / 'truth.csv'),
               FASTDYN_MAVLINK_FIRMWARE_PORT=str(port_base+1), FASTDYN_MAVLINK_GCS_PORT=str(port_base+2),
               GPS_INPUT_PORT=str(port_base+3), FASTDYN_MONITOR_PORT=str(port_base),
               FASTDYN_QMP_SOCKET=str(output / 'qmp.sock'),
               FASTDYN_QEMU_MEMORY_DIR=str(output / 'memory'), FASTDYN_REPO_ROOT=str(ROOT))
    env['PATH'] = str(ROOT / 'fastdyn-env/bin') + os.pathsep + env.get('PATH', '')
    command = [str(ROOT / 'fastdyn-env/bin/fastdyn'), 'run', '-c', str(config_path),
               '-o', str(output / 'work'), '--no-build-fmu']
    write_json(output / 'command.json', {'command': command, 'ports': {'base': port_base}})
    return output, command, env


def descendants(pid):
    children = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            if int(stat[1]) == pid:
                child = int(entry.name)
                children.extend(descendants(child))
                children.append(child)
        except (OSError, ValueError, IndexError):
            continue
    return children


def run(spec, output, build_dir, port_base=15550, timeout=900, boot_offset_s=0.0):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be finite and positive')
    output, command, env = prepare(spec, output, build_dir, port_base, boot_offset_s)
    print(f'Run: {output}\nLog: {output / "fastdyn.log"}', flush=True)
    process = None
    child_pids = []
    result = dict(returncode=None, error=None)
    try:
        with (output / 'fastdyn.log').open('x') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            result['returncode'] = process.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
        result['error'] = type(exc).__name__
        if process is not None:
            child_pids = descendants(process.pid)
    finally:
        if process is not None:
            for pid in child_pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            # Clean remaining descendants even if the FastDyn parent already exited.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        mission_path = output / 'mission.json'
        mission = read_json(mission_path) if mission_path.exists() else {'passed': False, 'error': 'mission did not finish'}
        result['passed'] = result['returncode'] == 0 and bool(mission['passed']) and result['error'] is None
        write_json(output / 'result.json', result)
        if (output / 'truth.csv').exists():
            from .data import export
            export([output], output / 'replay.json')
    print(f'Passed: {result["passed"]}', flush=True)
    return result['passed']


def campaign_cases(kind, seed=42, count=3):
    if count < 1:
        raise ValueError('campaign count must be positive')
    if kind == 'attacks':
        return [scenario(name) for name in ('baseline', 'magnetic', 'wind', 'gps-spoof', 'gyro')]
    if kind != 'monte-carlo':
        raise ValueError('campaign must be attacks or monte-carlo')
    rng = random.Random(seed)
    cases = [scenario('baseline')]
    for i in range(count):
        case = scenario('wind')
        case.update(name=f'Zigzag / wind sample {i:03d}', seed=seed, sample_index=i)
        case['attack'].update(wind_mean_mps=rng.uniform(3, 7), wind_gust_mps=rng.uniform(5, 12),
            wind_from_deg=rng.uniform(210, 300), wind_phase_rad=rng.uniform(0, 2*math.pi),
            wind_frequency_hz=rng.uniform(0.05, 0.2))
        cases.append(case)
    return cases
