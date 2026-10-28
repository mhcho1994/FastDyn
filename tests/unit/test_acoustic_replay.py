"""Check acoustic replay coordinates, source fidelity and read-only asset boundaries."""

from __future__ import annotations

import csv
import json

import pytest

from fastdyn import paths


np = pytest.importorskip('numpy')
with pytest.MonkeyPatch.context() as imports:
    imports.syspath_prepend(str(paths.virtuals_dir() / 'gyro_acoustic/tools'))
    import replay as server
    import replay_data as replay


def _recording(directory, pressure, *, invalid_truth=False):
    folder = directory / 'run-artifacts/gyro_acoustic'
    folder.mkdir(parents=True)
    settings = dict(rms_pressure_pa=pressure, duration_s=1, start_frequency_hz=23000,
                    end_frequency_hz=27000, sense_frequency_hz=25000)
    (folder / 'settings.json').write_text(json.dumps(settings), encoding='utf-8')
    rows = []
    for i in range(1201):
        t = i / 100
        # The isolated peak is between the regular 50 Hz pose samples.
        false = 3.5 if pressure and i == 133 else 0.0
        row = dict(time_s=t, sweep_elapsed_s=t - 1, frequency_hz=25000,
                   rms_pressure_pa=pressure if 1 <= t <= 2 else 0,
                   truth_valid=0 if invalid_truth else 1,
                   truth_n=t, truth_e=2, truth_d=-10, truth_altitude_m=10,
                   truth_qw=np.sqrt(.5), truth_qx=0, truth_qy=0, truth_qz=np.sqrt(.5),
                   truth_p=0, truth_q=0, truth_r=0, truth_vn=1, truth_ve=0, truth_vd=0)
        for axis in 'xyz':
            row.update({f'imu_accel_{axis}': 0, f'truth_gyro_{axis}': 0,
                        f'measured_gyro_{axis}': false if axis == 'x' else 0,
                        f'false_{axis}': false if axis == 'x' else 0})
        row.update({f'pwm{n}': 1500 for n in range(1, 5)})
        rows.append(row)
    with (folder / 'trace.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return directory


def test_pose_changes_both_world_and_body_bases():
    c = np.sqrt(.5)
    q = np.array([[1, 0, 0, 0], [c, c, 0, 0], [c, 0, c, 0], [c, 0, 0, c]])
    p, actual = replay.to_rumoca_pose(np.tile([1, 2, 3], (4, 1)), q)
    np.testing.assert_allclose(p, np.tile([1, -2, -3], (4, 1)))
    np.testing.assert_allclose(actual, [[1, 0, 0, 0], [c, c, 0, 0],
                                       [c, 0, -c, 0], [c, 0, 0, -c]])
    _, continuous = replay.to_rumoca_pose(np.zeros((3, 3)), np.array([q[0], -q[0], q[0]]))
    np.testing.assert_allclose(continuous, np.tile(q[0], (3, 1)))
    with pytest.raises(ValueError, match='quaternion'):
        replay.to_rumoca_pose(np.zeros((1, 3)), np.zeros((1, 4)))


def test_export_preserves_off_grid_peak_and_absolute_clock(tmp_path):
    baseline = _recording(tmp_path / 'baseline', 0)
    exposure = _recording(tmp_path / 'exposure', 20)
    output = tmp_path / 'replay.json'
    value = replay.export_replay(baseline, exposure, output, hz=50, telemetry=False)
    recorded = json.loads(output.read_text(encoding='utf-8'))
    assert recorded['schema'] == replay.SCHEMA
    case = value['cases'][1]
    assert case['sweep_start_sim_s'] == 1
    assert case['metrics']['source_samples'] == 1201
    assert case['metrics']['max_false_rate_rad_s'] == 3.5
    assert len(case['frames']) < case['metrics']['source_samples']
    peak = next(f for f in case['frames'] if f['false_rate_rad_s'][0] == 3.5)
    assert peak['t'] == pytest.approx(.33)
    assert peak['p'] == pytest.approx([1.33, -2, 10])
    assert peak['t'] + case['sweep_start_sim_s'] == pytest.approx(1.33)
    assert case['telemetry'] == {}
    assert case['notices']
    assert case['metrics']['early_contact_s'] is None
    with pytest.raises(FileExistsError):
        replay.export_replay(baseline, exposure, output)


def test_invalid_truth_and_mismatched_conditions_are_rejected(tmp_path):
    baseline = _recording(tmp_path / 'baseline', 0, invalid_truth=True)
    exposure = _recording(tmp_path / 'exposure', 20)
    with pytest.raises(ValueError, match='diagnostics'):
        replay.export_replay(baseline, exposure, tmp_path / 'bad.json', telemetry=False)
    good = _recording(tmp_path / 'good', 0)
    settings = exposure / 'run-artifacts/gyro_acoustic/settings.json'
    value = json.loads(settings.read_text(encoding='utf-8'))
    value['sense_frequency_hz'] = 26000
    settings.write_text(json.dumps(value), encoding='utf-8')
    with pytest.raises(ValueError, match='settings must differ only'):
        replay.export_replay(good, exposure, tmp_path / 'mismatch.json', telemetry=False)
    assert not (tmp_path / 'mismatch.json').exists()


def test_asset_resolution_cannot_escape_allowed_directory(tmp_path):
    assets = tmp_path / 'assets'
    assets.mkdir()
    private = tmp_path / 'private.txt'
    private.write_text('not a viewer asset', encoding='utf-8')
    (assets / 'link.txt').symlink_to(private)
    for name in ['../private.txt', 'link.txt', str(private), '.']:
        with pytest.raises(FileNotFoundError):
            server._asset_path(assets, name)
    valid = assets / 'scene.js'
    valid.write_text('// scene', encoding='utf-8')
    assert server._asset_path(assets, 'scene.js') == valid
