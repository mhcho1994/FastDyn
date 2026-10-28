#!/usr/bin/env python3
"""Plot full acoustic-control flights from direct FMU state and IMU diagnostics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


COLORS = {'baseline': '#0072B2', 'exposure': '#D55E00'}
LABELS = {'baseline': '0 Pa control', 'exposure': '20 Pa sweep'}


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """Return roll/pitch/yaw radians for Hamilton wxyz body-FRD to world-NED q."""
    norm = np.linalg.norm(q, axis=1)
    if np.any(norm < 1e-10):
        raise ValueError('Zero quaternion in FMU diagnostics')
    w, x, y, z = (q / norm[:, None]).T
    return np.column_stack((
        np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
        np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    ))


def break_angle_wraps(values: np.ndarray) -> np.ndarray:
    """Break plotted lines at wrapped Euler-angle discontinuities in degrees."""
    result = values.copy()
    result[np.r_[False, np.abs(np.diff(values)) > 180]] = np.nan
    return result


def read_case(path: Path) -> dict:
    """Load direct state/IMU signals, requiring completed and valid diagnostics."""
    folder = path/'run-artifacts/gyro_acoustic'
    data = np.genfromtxt(folder/'trace.csv', delimiter=',', names=True)
    needed = {'truth_valid', 'truth_n', 'truth_e', 'truth_d', 'truth_qw', 'truth_qx',
              'truth_qy', 'truth_qz', 'truth_p', 'truth_q', 'truth_r', 'imu_accel_x',
              'imu_accel_y', 'imu_accel_z'}
    if not needed <= set(data.dtype.names) or np.any(data['truth_valid'] != 1):
        raise ValueError(f'{path}: direct state/IMU diagnostics absent or invalid; rerun with tracing')
    if not all(np.all(np.isfinite(data[name])) for name in data.dtype.names):
        raise ValueError(f'{path}: nonfinite diagnostics')
    dt = np.diff(data['time_s'])
    if np.any(dt <= 0):
        raise ValueError(f'{path}: non-increasing FMU time')
    settings = json.loads((folder/'settings.json').read_text(encoding='utf-8'))
    started = np.flatnonzero(data['sweep_elapsed_s'] >= 0)
    if not len(started):
        raise ValueError(f'{path}: sweep never triggered')
    index = started[0]
    start = float(data['time_s'][index]-data['sweep_elapsed_s'][index])
    time = data['time_s'] - start
    if time[-1] < settings['duration_s'] + 10:
        raise ValueError(f'{path}: incomplete sweep/recovery observation')
    def columns(names):
        return np.column_stack([data[name] for name in names])
    position = columns(['truth_n', 'truth_e', 'truth_d'])
    quaternion = columns(['truth_qw', 'truth_qx', 'truth_qy', 'truth_qz'])
    contact = np.flatnonzero((time >= 0) & (time <= settings['duration_s']+10)
                             & (-position[:, 2] <= 0.3))
    return dict(path=path, raw=data, settings=settings, start=start, time=time,
                position=position, quaternion=quaternion,
                attitude=np.rad2deg(quaternion_to_euler(quaternion)),
                velocity=columns(['truth_vn', 'truth_ve', 'truth_vd']),
                rate=np.rad2deg(columns(['truth_p', 'truth_q', 'truth_r'])),
                accel=columns(['imu_accel_x', 'imu_accel_y', 'imu_accel_z']),
                gyro=np.rad2deg(columns(['measured_gyro_x', 'measured_gyro_y', 'measured_gyro_z'])),
                clean_gyro=np.rad2deg(columns(['truth_gyro_x', 'truth_gyro_y', 'truth_gyro_z'])),
                contact=int(contact[0]) if len(contact) else None,
                median_dt=float(np.median(dt)), max_dt=float(np.max(dt)))


def read_px4(case: dict) -> dict:
    """Read estimates using PX4 boot timestamps, which share QEMU's FMU time base."""
    # Optional telemetry dependency is only needed by this plotting command.
    from pymavlink import mavutil
    messages = {'ATTITUDE': [], 'LOCAL_POSITION_NED': []}
    link = mavutil.mavlink_connection(str(case['path']/'mav.tlog'))
    try:
        while (message := link.recv_match()) is not None:
            kind = message.get_type()
            if kind in messages and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                messages[kind].append(message.to_dict())
    finally:
        link.close()
    result = {}
    for kind, rows in messages.items():
        if not rows:
            raise ValueError(f"{case['path']}: missing PX4 {kind} telemetry")
        # Messages can be duplicated by a telemetry router; keep one per timestamp.
        rows = list({r['time_boot_ms']: r for r in rows}.values())
        rows.sort(key=lambda row: row['time_boot_ms'])
        time = np.array([r['time_boot_ms']/1000-case['start'] for r in rows])
        if kind == 'ATTITUDE':
            result['attitude'] = (time, np.rad2deg([[r[k] for k in ('roll', 'pitch', 'yaw')]
                                                   for r in rows]))
            result['rate'] = (time, np.rad2deg([[r[k] for k in ('rollspeed', 'pitchspeed', 'yawspeed')]
                                               for r in rows]))
        else:
            result['velocity'] = (time, np.array([[r[k] for k in ('vx', 'vy', 'vz')]
                                                   for r in rows]))
    return result


def save_figure(fig, output: Path, name: str) -> None:
    """Write readable raster and vector artifacts."""
    fig.savefig(output/f'{name}.png', dpi=170)
    fig.savefig(output/f'{name}.pdf')


def plot_signal(cases: dict, output: Path, field: str, title: str,
                labels: list[str], name: str, *, px4: bool = False) -> None:
    """Plot the complete recordings and an expanded disturbance window separately."""
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(15, 8), sharex='col', layout='constrained')
    focus = (8, 12) if field in ('accel', 'gyro', 'rate') else (-3, 25)
    for row, label in enumerate(labels):
        for col in range(2):
            ax = axes[row, col]
            for key, case in cases.items():
                t = case['time']
                mask = (np.ones(len(t), dtype=bool) if col == 0
                        else (t >= focus[0]) & (t <= focus[1]))
                y = case[field][:, row]
                if field == 'attitude':
                    y = break_angle_wraps(y)
                ax.plot(t[mask], y[mask], color=COLORS[key], linewidth=.8,
                        label=LABELS[key]+(' FMU truth' if px4 else ' IMU output'))
                if field == 'gyro' and key == 'exposure':
                    ax.plot(t[mask], case['clean_gyro'][mask, row], color='#333333',
                            linestyle='--', linewidth=.8, label='Sweep: clean gyro')
                if px4:
                    et, estimate = case['px4'][field]
                    emask = (np.ones(len(et), dtype=bool) if col == 0
                             else (et >= focus[0]) & (et <= focus[1]))
                    ey = estimate[:, row]
                    if field == 'attitude':
                        ey = break_angle_wraps(ey)
                    ax.plot(et[emask], ey[emask], color=COLORS[key], linestyle='--',
                            linewidth=1, alpha=.7, label=LABELS[key]+' PX4')
            ax.axvspan(0, cases['exposure']['settings']['duration_s'], color='#E69F00', alpha=.08)
            ax.axvline(10, color='#666666', linestyle=':', linewidth=.8)
            contact = cases['exposure']['contact']
            if contact is not None:
                ax.axvline(cases['exposure']['time'][contact], color='#CC0077', linestyle=':', linewidth=.8)
            if col == 1:
                ax.set_xlim(*focus)
            if field == 'accel' and col == 1:
                peak = max(np.max(np.abs(case[field][
                    (case['time'] >= focus[0]) & (case['time'] <= focus[1]), row]))
                    for case in cases.values())
                if peak < 1:
                    ax.set_ylim(-1, 1)  # Do not amplify floating-point zero into visible motion.
            ax.grid(alpha=.25)
            if col == 0:
                ax.set_ylabel(label)
    axes[0, 0].set_title('Complete recorded flight')
    axes[0, 1].set_title('25 kHz crossing detail' if focus == (8, 12)
                         else 'Sweep / loss-of-control detail')
    axes[0, 0].legend(fontsize=8, ncol=2, loc='upper right')
    for ax in axes[-1]:
        ax.set_xlabel('Time from sweep start [simulated s]')
    fig.suptitle(title+'\nShading: sweep; gray: 25 kHz crossing; magenta: early ground contact', fontsize=13)
    save_figure(fig, output, name)
    plt.close(fig)


def plot_trajectory(cases: dict, output: Path) -> None:
    """Compare full direct CG trajectories with physical axis scaling and projections."""
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    fig = plt.figure(figsize=(15, 10), layout='constrained')
    ax3d = fig.add_subplot(2, 2, 1, projection='3d')
    projections = [fig.add_subplot(2, 2, i) for i in (2, 3, 4)]
    positions = []
    for key, case in cases.items():
        p = case['position'].copy()
        p[:, 2] *= -1  # Display North/East/Up, while retaining NED in data exports.
        positions.append(p)
        ax3d.plot(*p.T, color=COLORS[key], linewidth=1.4, label=LABELS[key])
        for ax, i, j in zip(projections, (0, 0, 1), (1, 2, 2)):
            ax.plot(p[:, i], p[:, j], color=COLORS[key], linewidth=1.1, label=LABELS[key])
        # Show complete run start/end, exposure onset and the first early ground contact.
        markers = [(0, 'o', 'Start'), (len(p)-1, 'X', 'End'),
                   (int(np.argmin(np.abs(case['time']))), '^', 'Sweep onset')]
        if case['contact'] is not None:
            markers.append((case['contact'], '*', 'Early contact'))
        for index, marker, label in markers:
            ax3d.scatter(*p[index], color=COLORS[key], marker=marker, s=50,
                         label=f'{LABELS[key]}: {label}')
            for ax, i, j in zip(projections, (0, 0, 1), (1, 2, 2)):
                ax.scatter(p[index, i], p[index, j], color=COLORS[key], marker=marker, s=30)
    combined = np.vstack(positions)
    low, high = combined.min(axis=0), combined.max(axis=0)
    span = np.maximum(high-low, 1)
    for setter, i in zip([ax3d.set_xlim, ax3d.set_ylim, ax3d.set_zlim], range(3)):
        setter(low[i]-.04*span[i], high[i]+.04*span[i])
    ax3d.set_box_aspect(span)
    ax3d.set(xlabel='North [m]', ylabel='East [m]', zlabel='Up [m]', title='3D CG trajectory (equal metric scale)')
    ax3d.view_init(elev=24, azim=-55)
    for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        axis.set_major_locator(MaxNLocator(3))
    ax3d.legend(fontsize=7, loc='upper left')
    for ax, xlabel, ylabel in zip(projections, ['North [m]', 'North [m]', 'East [m]'],
                                 ['East [m]', 'Up [m]', 'Up [m]']):
        ax.set(xlabel=xlabel, ylabel=ylabel)
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.25)
    projections[0].set_title('Top view')
    projections[1].set_title('North / altitude')
    projections[2].set_title('East / altitude')
    fig.suptitle('Full 3D trajectories | direct FMU position | PX4 v1.17.0-rc2', fontsize=15)
    save_figure(fig, output, 'trajectory_3d')
    plt.close(fig)


def write_csv(case: dict, output: Path, name: str) -> None:
    """Export plotted FMU series in SI units with Euler angles additionally in degrees."""
    data = np.column_stack([case['time'], case['position'], case['velocity'], case['attitude'],
                            np.deg2rad(case['rate']), case['accel'], np.deg2rad(case['gyro'])])
    header = ('time_from_sweep_s,north_m,east_m,down_m,vn_m_s,ve_m_s,vd_m_s,'
              'roll_deg,pitch_deg,yaw_deg,p_rad_s,q_rad_s,r_rad_s,'
              'imu_ax_m_s2,imu_ay_m_s2,imu_az_m_s2,imu_gx_rad_s,imu_gy_rad_s,imu_gz_rad_s')
    np.savetxt(output/f'{name}_signals.csv', data, delimiter=',', header=header, comments='', fmt='%.10g')
    for field, (time, values) in case['px4'].items():
        labels = {'attitude': 'roll_deg,pitch_deg,yaw_deg',
                  'rate': 'p_deg_s,q_deg_s,r_deg_s',
                  'velocity': 'vn_m_s,ve_m_s,vd_m_s'}[field]
        np.savetxt(output/f'{name}_px4_{field}.csv', np.column_stack([time, values]),
                   delimiter=',', header='time_from_sweep_s,' + labels,
                   comments='', fmt='%.10g')


def write_interactive(cases: dict, output: Path) -> None:
    """Save an offline, rotatable view of the complete recorded trajectories."""
    flights = {}
    for key, case in cases.items():
        p = case['position'].copy()
        p[:, 2] *= -1
        indices = np.unique(np.linspace(0, len(p)-1, min(3000, len(p)), dtype=int))
        marks = [(0, '●', 'Start'), (len(p)-1, '×', 'End'),
                 (int(np.argmin(np.abs(case['time']))), '▲', 'Sweep onset')]
        if case['contact'] is not None:
            marks.append((case['contact'], '★', 'Early contact'))
        flights[key] = dict(points=p[indices].round(5).tolist(), color=COLORS[key],
                            label=LABELS[key], samples=len(p),
                            recorded=float(case['raw']['time_s'][-1]),
                            markers=[dict(position=p[i].tolist(), time=float(case['time'][i]),
                                          symbol=s, label=label) for i, s, label in marks])
    template = Path(__file__).with_name('trajectory_viewer.html').read_text(encoding='utf-8')
    html = template.replace('__TRAJECTORY_DATA__', json.dumps(flights, separators=(',', ':')))
    (output/'trajectory_3d.html').write_text(html, encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('exposure', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = {'baseline': read_case(args.baseline), 'exposure': read_case(args.exposure)}
    for case in cases.values():
        case['px4'] = read_px4(case)
    # Matplotlib is optional until plotting is requested.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'path.simplify': False})
    args.output.mkdir(parents=True, exist_ok=True)
    plot_trajectory(cases, args.output)
    write_interactive(cases, args.output)
    specifications = [
        ('accel', 'IMU acceleration output: body-FRD specific force (includes gravity response)',
         ['Ax [m/s2]', 'Ay [m/s2]', 'Az [m/s2]'], 'imu_acceleration', False),
        ('gyro', 'IMU angular-velocity output: clean gyro + injected false rate',
         ['Gyro X [deg/s]', 'Gyro Y [deg/s]', 'Gyro Z [deg/s]'], 'imu_angular_velocity', False),
        ('velocity', 'Velocity: world-NED FMU truth and PX4 estimate',
         ['North [m/s]', 'East [m/s]', 'Down [m/s]'], 'velocity', True),
        ('attitude', 'Attitude: body-FRD to world-NED Euler angles (wrapped)',
         ['Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'], 'attitude', True),
        ('rate', 'Body angular rate p/q/r: FMU truth and PX4 filtered rate (not Euler derivatives)',
         ['p [deg/s]', 'q [deg/s]', 'r [deg/s]'], 'body_rate', True),
    ]
    for field, title, labels, name, px4 in specifications:
        plot_signal(cases, args.output, field, title, labels, name, px4=px4)
    report = {}
    for key, case in cases.items():
        write_csv(case, args.output, key)
        report[key] = {
            'samples': len(case['time']), 'recorded_sim_time_s': float(case['raw']['time_s'][-1]),
            'sweep_start_s': case['start'], 'median_sample_interval_s': case['median_dt'],
            'maximum_sample_interval_s': case['max_dt'],
            'max_quaternion_norm_error': float(np.max(np.abs(np.linalg.norm(case['quaternion'], axis=1)-1))),
            'max_horizontal_distance_m': float(np.max(np.linalg.norm(case['position'][:, :2], axis=1))),
            'max_altitude_m': float(np.max(-case['position'][:, 2])),
            'max_speed_m_s': float(np.max(np.linalg.norm(case['velocity'], axis=1))),
            'early_contact_after_sweep_s': (float(case['time'][case['contact']])
                                             if case['contact'] is not None else None),
            'imu_accel_abs_max_m_s2': np.max(np.abs(case['accel']), axis=0).tolist(),
            'imu_gyro_abs_max_deg_s': np.max(np.abs(case['gyro']), axis=0).tolist(),
            'body_rate_abs_max_deg_s': np.max(np.abs(case['rate']), axis=0).tolist(),
            'trace_sha256': hashlib.sha256((case['path']/'run-artifacts/gyro_acoustic/trace.csv').read_bytes()).hexdigest(),
            'px4_samples': {name: len(data[0]) for name, data in case['px4'].items()},
        }
    (args.output/'flight_metrics.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
