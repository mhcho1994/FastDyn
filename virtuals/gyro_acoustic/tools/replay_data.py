"""Export acoustic recordings as bounded-size Rumoca replay data."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from plot_flight import read_case, read_px4


SCHEMA = 'fastdyn-acoustic-replay-v1'


def to_rumoca_pose(position: np.ndarray, quaternion: np.ndarray) -> tuple:
    """Convert NED / body-FRD poses to NWU / body-FLU, with wxyz quaternions."""
    norms = np.linalg.norm(quaternion, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(np.abs(norms - 1) > 0.1):
        raise ValueError('Invalid recorded truth quaternion norm')
    p = position * [1, -1, -1]
    # Change BOTH world and body bases: R_nwu_flu = S R_ned_frd S.
    q = quaternion / norms[:, None] * [1, 1, -1, -1]
    signs = np.ones(len(q))
    signs[1:] = np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1, 1))
    return p, q * signs[:, None]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _indices(case: dict, hz: float) -> np.ndarray:
    time = case['time']
    regular = np.searchsorted(time, np.arange(time[0], time[-1], 1 / hz))
    extra = [0, len(time) - 1]
    for channel in ('false_x', 'false_y', 'false_z'):
        extra.extend((int(np.argmin(case['raw'][channel])),
                      int(np.argmax(case['raw'][channel]))))
    events = [0, case['settings']['duration_s'], _resonance_time(case['settings'])]
    for event in events:
        if event is not None:
            extra.append(int(np.argmin(np.abs(time - event))))
    if case['contact'] is not None:
        extra.append(case['contact'])
    # Preserve both sides of missing data so the player cannot bridge a gap.
    gaps = np.flatnonzero(np.diff(time) > 0.25)
    return np.unique(np.concatenate((regular, extra, gaps, gaps + 1))).astype(int)


def _resonance_time(settings: dict) -> float | None:
    delta = settings['end_frequency_hz'] - settings['start_frequency_hz']
    if not delta:
        return None
    t = settings['duration_s'] * (
        settings['sense_frequency_hz'] - settings['start_frequency_hz']) / delta
    return t if 0 <= t <= settings['duration_s'] else None


def _metrics(case: dict) -> dict:
    raw, time = case['raw'], case['time']
    mask = (time >= 0) & (time <= case['settings']['duration_s'] + 10)
    false = np.column_stack([raw[f'false_{a}'] for a in 'xyz'])
    return {
        'source_samples': len(time),
        'source_median_interval_s': case['median_dt'],
        'window_s': [0, case['settings']['duration_s'] + 10],
        'max_false_rate_rad_s': float(np.max(np.linalg.norm(false, axis=1))),
        'min_altitude_m': float(np.min(-case['position'][mask, 2])),
        'max_altitude_m': float(np.max(-case['position'][mask, 2])),
        'max_horizontal_distance_m': float(np.max(
            np.linalg.norm(case['position'][mask, :2], axis=1))),
        'max_down_speed_m_s': float(np.max(case['velocity'][mask, 2])),
        'early_contact_s': (float(time[case['contact']])
                            if case['contact'] is not None else None),
    }


def _telemetry(case: dict, enabled: bool) -> tuple[dict, list[str]]:
    if not enabled:
        return {}, ['PX4 overlay disabled.']
    if not (case['path'] / 'mav.tlog').is_file():
        return {}, ['No mav.tlog; displaying FMU truth only.']
    try:
        values = read_px4(case)
    except (ImportError, ValueError) as exc:
        return {}, [f'PX4 overlay unavailable: {exc}']
    return {key: [dict(t=float(t), value=list(map(float, value)))
                  for t, value in zip(times, series)]
            for key, (times, series) in values.items()}, []


def _export_case(path: Path, key: str, hz: float, telemetry: bool) -> dict:
    case = read_case(path)
    p, q = to_rumoca_pose(case['position'], case['quaternion'])
    frames = []
    for i in _indices(case, hz):
        row = case['raw'][i]
        frames.append({
            't': round(float(case['time'][i]), 9),
            'p': p[i].round(7).tolist(),
            'q': q[i].round(10).tolist(),
            'attitude_deg': case['attitude'][i].round(6).tolist(),
            'rate_deg_s': case['rate'][i].round(6).tolist(),
            'gyro_deg_s': case['gyro'][i].round(6).tolist(),
            'clean_gyro_deg_s': case['clean_gyro'][i].round(6).tolist(),
            'velocity_ned': case['velocity'][i].round(7).tolist(),
            'false_rate_rad_s': [float(row[f'false_{a}']) for a in 'xyz'],
            'frequency_hz': float(row['frequency_hz']),
            'pressure_pa': float(row['rms_pressure_pa']),
            'pwm_us': [float(row[f'pwm{n}']) for n in range(1, 5)],
        })
    estimated, notices = _telemetry(case, telemetry)
    folder = path / 'run-artifacts/gyro_acoustic'
    libraries = list(path.glob('fmu-*/binaries/*/*.so'))
    return {
        'id': key,
        'label': f"{case['settings']['rms_pressure_pa']:g} Pa "
                 + ('control' if key == 'baseline' else 'sweep'),
        'source': path.name,
        'sweep_start_sim_s': case['start'],
        'resonance_s': _resonance_time(case['settings']),
        'settings': case['settings'],
        'metrics': _metrics(case),
        'frames': frames,
        'telemetry': estimated,
        'notices': notices,
        'provenance': {
            'trace_sha256': _sha256(folder / 'trace.csv'),
            'settings_sha256': _sha256(folder / 'settings.json'),
            'fmu_sha256': _sha256(libraries[0]) if len(libraries) == 1 else None,
        },
    }


def export_replay(baseline: Path, exposure: Path, output: Path,
                  hz: float = 50, telemetry: bool = True) -> dict:
    """Write a new replay, preserving full-resolution metrics and simulation timestamps."""
    if not math.isfinite(hz) or not 10 <= hz <= 200:
        raise ValueError('Replay sample rate must be between 10 and 200 Hz')
    if output.exists():
        raise FileExistsError(f'Choose a new output file: {output}')
    cases = [_export_case(Path(path), key, hz, telemetry)
             for key, path in [('baseline', baseline), ('exposure', exposure)]]
    clean, exposed = [case['settings'] for case in cases]
    if clean['rms_pressure_pa'] != 0 or exposed['rms_pressure_pa'] <= 0:
        raise ValueError('Expected a 0 Pa baseline and positive-pressure exposure')
    if {k: v for k, v in clean.items() if k != 'rms_pressure_pa'} != {
            k: v for k, v in exposed.items() if k != 'rms_pressure_pa'}:
        raise ValueError('Paired acoustic settings must differ only in rms_pressure_pa')
    hashes = [case['provenance']['fmu_sha256'] for case in cases]
    if all(hashes) and hashes[0] != hashes[1]:
        raise ValueError('Paired recordings used different FMU binaries')
    if cases[0]['metrics']['max_false_rate_rad_s'] > 1e-9:
        raise ValueError('The zero-pressure baseline contains a nonzero injection')
    value = {
        'schema': SCHEMA,
        'time_axis': 'Seconds from each run\'s recorded sweep onset',
        'absolute_time': 'sim_time_s = t + sweep_start_sim_s',
        'position_frame': 'NWU',
        'quaternion': 'Hamilton wxyz, body FLU to world NWU',
        'signal_frame': 'Body FRD rates/gyro; world NED velocity; angles in degrees',
        'pose_sample_hz': hz,
        'max_interpolation_gap_s': 0.25,
        'metrics_source': 'All original trace samples, before replay subsampling',
        'exporter_sha256': _sha256(Path(__file__)),
        'cases': cases,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        handle.write('\n')
    return value
