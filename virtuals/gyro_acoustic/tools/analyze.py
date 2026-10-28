#!/usr/bin/env python3
"""Compare paired clean-FMU acoustic traces and save metrics plus a plot."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re


def read_run(directory: Path) -> tuple[list[dict[str, float]], dict]:
    """Read finite trace rows and the exact runtime exposure settings."""
    artifacts = directory / 'run-artifacts/gyro_acoustic'
    settings = json.loads((artifacts / 'settings.json').read_text(encoding='utf-8'))
    with (artifacts / 'trace.csv').open(encoding='utf-8') as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    if not rows or not all(math.isfinite(value) for row in rows for value in row.values()):
        raise ValueError(f'{directory}: empty or nonfinite trace')
    if any(a['time_s'] >= b['time_s'] for a, b in zip(rows, rows[1:])):
        raise ValueError(f'{directory}: non-increasing trace time')
    if rows[-1]['sweep_elapsed_s'] < settings['duration_s'] + 10:
        raise ValueError(f'{directory}: sweep/recovery observation was not completed')
    north = east = 0.
    previous = rows[0]
    for row in rows:
        dt = row['time_s'] - previous['time_s']
        north += dt * (row['truth_vn'] + previous['truth_vn']) / 2
        east += dt * (row['truth_ve'] + previous['truth_ve']) / 2
        row['horizontal_distance_m'] = math.hypot(north, east)
        previous = row
    return rows, settings


def metrics(rows: list[dict[str, float]], settings: dict) -> dict:
    """Measure the sweep and 10 s recovery window before the scheduled landing."""
    window = [row for row in rows if 0 <= row['sweep_elapsed_s'] <= settings['duration_s'] + 10]
    ground = [row for row in window if row['truth_altitude_m'] <= 0.3]
    return {
        'samples': len(rows),
        'last_sim_time_s': rows[-1]['time_s'],
        'sweep_start_time_s': window[0]['time_s'] - window[0]['sweep_elapsed_s'],
        'max_false_rate_rad_s': max(math.sqrt(sum(row[f'false_{a}']**2 for a in 'xyz'))
                                  for row in rows),
        'window_min_altitude_m': min(row['truth_altitude_m'] for row in window),
        'window_max_altitude_m': max(row['truth_altitude_m'] for row in window),
        'window_max_down_speed_m_s': max(row['truth_vd'] for row in window),
        'window_max_horizontal_distance_m': max(row['horizontal_distance_m'] for row in window),
        'early_ground_contact': bool(ground),
        'ground_contact_sweep_elapsed_s': ground[0]['sweep_elapsed_s'] if ground else None,
        'max_sensor_sum_residual_rad_s': max(
            abs(row[f'measured_gyro_{a}']-row[f'truth_gyro_{a}']-row[f'false_{a}'])
            for row in rows for a in 'xyz'),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('sweep', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline, clean_settings = read_run(args.baseline)
    sweep, settings = read_run(args.sweep)
    if clean_settings['rms_pressure_pa'] != 0 or settings['rms_pressure_pa'] <= 0:
        raise ValueError('Expected zero-pressure control and positive-pressure sweep')
    if any(clean_settings[k] != v for k, v in settings.items() if k != 'rms_pressure_pa'):
        raise ValueError('Control and sweep must differ only in pressure')
    report = {'baseline': metrics(baseline, clean_settings), 'sweep': metrics(sweep, settings),
              'settings': settings, 'window': 'sweep start through 10 s after sweep end',
              'model_scope': 'Illustrative reduced gyro response on clean P2 sensors; no hardware calibration'}
    for name, directory in [('baseline', args.baseline), ('sweep', args.sweep)]:
        console = (directory/'console.log').read_text(encoding='utf-8', errors='replace')
        for marker in ['PX4 git-hash: 0b6e4687defb353a34201951809efd3f0040a9ba',
                       'arming confirmed', 'mission start accepted', 'observation period ended']:
            if marker not in console:
                raise ValueError(f'{directory}: missing completed-run evidence: {marker}')
        if 'FMU fmi3DoStep failed' in console or 'Nonfinite acoustic response' in console:
            raise ValueError(f'{directory}: numerical runtime failure')
        if name == 'baseline' and not all(marker in console for marker in
                ['Mission finished, landed', 'Disarmed by landing']):
            raise ValueError(f'{directory}: control did not finish with normal landing/disarming')
        if report[name]['max_sensor_sum_residual_rad_s'] > 1e-5:
            raise ValueError(f'{directory}: sensor trace does not match clean gyro + false rate')
        libraries = list(directory.glob('fmu-*/binaries/*/*.so'))
        if len(libraries) != 1:
            raise ValueError(f'{directory}: expected exactly one isolated FMU library')
        report[name]['fmu_binary_sha256'] = hashlib.sha256(libraries[0].read_bytes()).hexdigest()
        report[name]['px4_commit'] = re.search(r'PX4 git-hash: (\w+)', console).group(1)
        report[name]['px4_roll_failure_reported'] = 'Attitude failure (roll)' in console
    if report['baseline']['fmu_binary_sha256'] != report['sweep']['fmu_binary_sha256']:
        raise ValueError('The paired runs used different FMU binaries')
    if report['baseline']['max_false_rate_rad_s'] != 0:
        raise ValueError('Control has a nonzero injected rate')
    if report['baseline']['early_ground_contact']:
        raise ValueError('Control also contacted ground before the scheduled landing')
    args.output.mkdir(parents=True, exist_ok=True)
    for name, directory in [('baseline', args.baseline), ('sweep', args.sweep)]:
        report[name]['trace_sha256'] = hashlib.sha256(
            (directory/'run-artifacts/gyro_acoustic/trace.csv').read_bytes()).hexdigest()
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    # Optional plotting dependency, only loaded when this analysis command is run.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    for key, name, rows in [('baseline', '0 Pa control', baseline),
                            ('sweep', f"{settings['rms_pressure_pa']:g} Pa sweep", sweep)]:
        rows = [r for r in rows if -3 <= r['sweep_elapsed_s'] <= settings['duration_s'] + 10
                and r['time_s'] >= report[key]['sweep_start_time_s'] - 3]
        x = [r['sweep_elapsed_s'] for r in rows]
        for ax, column in zip(axes[1:], ['false_x', 'truth_altitude_m', 'horizontal_distance_m']):
            ax.plot(x, [r[column] for r in rows], label=name, linewidth=1)
    selected = [r for r in sweep if 0 <= r['sweep_elapsed_s'] <= settings['duration_s']]
    axes[0].plot([r['sweep_elapsed_s'] for r in selected],
                 [r['frequency_hz']/1000 for r in selected], color='tab:orange')
    axes[0].set_ylabel('Frequency [kHz]')
    axes[1].set_ylabel('False roll rate [rad/s]')
    axes[2].set_ylabel('FMU altitude [m]')
    axes[3].set_ylabel('FMU horizontal\ndistance [m]')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.axvline(settings['duration_s']/2, color='gray', linestyle=':', linewidth=.8)
    axes[2].legend()
    axes[-1].set_xlabel('Time since sweep start [simulated s]')
    fig.suptitle('PX4 v1.17.0-rc2 / P2 quadrotor / illustrative acoustic response')
    fig.tight_layout()
    fig.savefig(args.output/'comparison.png', dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
