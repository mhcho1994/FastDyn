"""Versioned replay data and metrics. All comparisons use simulation seconds."""
from __future__ import annotations

from bisect import bisect_right
import csv
import math
from pathlib import Path

from .common import heading, wrap_deg, read_json, write_json, sha256


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def decode_truth(row):
    t = number(row.get('sim_time_s'))
    p = [number(row.get(k)) for k in ('north_m', 'west_m', 'alt_m')]
    q = [number(row.get(k)) for k in ('qw', 'qx', 'qy', 'qz')]
    if t is None or None in p or None in q:
        raise ValueError('truth needs finite sim_time_s, NWU position and wxyz quaternion')
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-8 or abs(norm-1) > 0.1:
        raise ValueError(f'invalid truth quaternion norm at t={t}: {norm}')
    q = [x/norm for x in q]
    return dict(t=t, p=p, q=q, omega=[number(row.get(f'omega{i}')) for i in range(1, 5)],
                wind=[number(row.get(k)) for k in ('wind_n_ms', 'wind_w_ms', 'wind_u_ms')],
                attack_gate=number(row.get('attack_gate')), model_current_a=number(row.get('model_current_a')),
                mag_delta_gauss=[number(row.get(f'mag_delta_{a}_gauss')) for a in 'xyz'],
                gps_offset_neu=[number(row.get(f'gps_offset_{a}_m')) for a in 'neu'],
                gyro_bias_rps=[number(row.get(f'gyro_bias_{a}_rps')) for a in 'xyz'],
                gyro_false_rps=[number(row.get(f'gyro_false_{a}_rps')) for a in 'xyz'],
                current_a=number(row.get('current_a')), voltage_v=number(row.get('voltage_v', row.get('battery_voltage_v'))),
                true_heading_deg=heading(q))


def load_truth(path):
    with Path(path).open(newline='') as f:
        frames = [decode_truth(row) for row in csv.DictReader(f)]
    if not frames:
        raise ValueError(f'no truth samples in {path}')
    for a, b in zip(frames, frames[1:]):
        if b['t'] <= a['t']:
            raise ValueError('truth timestamps must be strictly increasing (do not concatenate reboots)')
        # q and -q represent the same attitude; retain a continuous representative.
        if sum(x*y for x, y in zip(a['q'], b['q'])) < 0:
            b['q'] = [-x for x in b['q']]
    return frames


def csv_rows(path):
    if not path.exists():
        return []
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    k = (len(values)-1)*fraction
    lo, hi = math.floor(k), math.ceil(k)
    return values[lo] + (values[hi]-values[lo])*(k-lo)


def stats(values):
    return dict(count=len(values), rms=math.sqrt(sum(x*x for x in values)/len(values)) if values else None,
                p95_abs=percentile([abs(x) for x in values], .95),
                max_abs=max(map(abs, values)) if values else None)


def segment_distance(n, e, sn, se, tn, te):
    dn, de = tn-sn, te-se
    size = dn*dn+de*de
    u = max(0, min(1, ((n-sn)*dn+(e-se)*de)/size)) if size else 0
    return math.hypot(n-sn-u*dn, e-se-u*de)


def interpolate_heading(t, samples, times, max_gap=.5):
    index = bisect_right(times, t)-1
    if index < 0:
        return None
    a = samples[index]
    if t == a[0]:
        return a[1]
    if index+1 >= len(samples):
        return None
    b = samples[index+1]
    if b[0]-a[0] > max_gap:
        return None
    return wrap_deg(a[1] + (t-a[0])/(b[0]-a[0])*wrap_deg(b[1]-a[1]))


def interpolate_position(t, samples, times, max_gap=.5):
    i = bisect_right(times, t)-1
    if i < 0:
        return None
    ta, a = samples[i]
    if t == ta:
        return a
    if i+1 >= len(samples):
        return None
    tb, b = samples[i+1]
    if tb-ta > max_gap:
        return None
    return [x+(y-x)*(t-ta)/(tb-ta) for x, y in zip(a, b)]


def waypoint_audit(frames, spec, mission, offset):
    """Require contiguous truth dwell at the CURRENT commanded waypoint.

    Allow one 20 Hz sampling interval of boundary uncertainty. Never infer truth
    arrival from a GPS/FC arrival or from passing a future waypoint out of order.
    """
    arrivals = {r['index']: r['time_boot_s']+offset for r in mission.get('reached', [])}
    audit = []
    times = [f['t'] for f in frames]
    for index, (n, e) in enumerate(spec.get('waypoints_ne_m', [])):
        first = last = None
        dwell = 0.0
        distances = []
        for frame in frames:
            if not frame['analysis_valid'] or frame.get('waypoint_index') != index:
                continue
            distance = math.hypot(frame['p'][0]-n, -frame['p'][1]-e)
            distances.append(distance)
            inside = (distance <= spec.get('acceptance_radius_m', 2)
                      and abs(frame['p'][2]-spec['altitude_m']) <= spec.get('altitude_tolerance_m', 1))
            if inside:
                if first is None or frame['t']-last > .1:
                    first = frame['t']
                last = frame['t']
                dwell = max(dwell, last-first)
            else:
                first = last = None
        arrival = arrivals.get(index)
        pos = interpolate_position(arrival, [(f['t'], f['p']) for f in frames], times) if arrival is not None else None
        audit.append(dict(index=index, number=index+1, target_ne_m=[n,e],
            fc_reached=arrival is not None, fc_arrival_s=arrival,
            truth_reached=bool(distances) and dwell+0.051 >= spec.get('hold_s', 1),
            longest_truth_dwell_s=dwell, minimum_truth_distance_m=min(distances) if distances else None,
            truth_distance_at_fc_arrival_m=math.hypot(pos[0]-n,-pos[1]-e) if pos else None))
    return audit


def make_case(path):
    path = Path(path).resolve()
    raw = path if path.is_file() else path / 'truth.csv'
    directory = raw.parent
    meta = read_json(directory/'run.json') if (directory/'run.json').exists() else dict(name=path.stem, imported=True)
    frames = load_truth(raw)
    offset = float(meta.get('boot_offset_s', 0))
    telemetry = csv_rows(directory / 'telemetry.csv')
    headings, positions = {}, {}
    for row in telemetry:
        t, yaw = number(row.get('time_boot_s')), number(row.get('yaw_deg'))
        position = [number(row.get(key)) for key in ('north_m','east_m','alt_m')]
        if t is not None and None not in position:
            positions[t+offset] = [position[0], -position[1], position[2]]
        if t is not None and yaw is not None:
            headings[t+offset] = yaw
    samples = sorted(headings.items())
    times = [s[0] for s in samples]
    position_samples = sorted(positions.items())
    position_times = [s[0] for s in position_samples]
    spec = meta.get('scenario', {})
    attack = spec.get('attack', {})
    commands = []
    for row in csv_rows(directory / 'commands.csv'):
        command = {k: (row[k] if k == 'phase' else number(row[k])) for k in row}
        if command.get('time_boot_s') is None:
            raise ValueError('invalid command timestamp')
        command['t'] = command.pop('time_boot_s') + offset
        commands.append(command)
    if any(a['t'] > b['t'] for a, b in zip(commands, commands[1:])):
        raise ValueError('command time moved backwards')
    cmd_times = [c['t'] for c in commands]
    mission = read_json(directory/'mission.json') if (directory/'mission.json').exists() else {}
    final = read_json(directory/'result.json') if (directory/'result.json').exists() else {}
    end = mission.get('analysis_end_boot_s', mission.get('last_boot_s'))
    analysis_end = end+offset if end is not None else None
    contact_t = None
    airborne = False
    heading_errors, horizontal, altitude, hold = [], [], [], []
    valid_window = []
    for frame in frames:
        t, p = frame['t'], frame['p']
        estimated = interpolate_heading(t, samples, times)
        frame['estimated_p'] = interpolate_position(t, position_samples, position_times)
        frame['position_estimate_error_m'] = (math.hypot(frame['estimated_p'][0]-p[0], frame['estimated_p'][1]-p[1])
                                               if frame['estimated_p'] is not None else None)
        frame['attack_active'] = attack.get('kind', 'none') != 'none' and (frame['attack_gate'] or 0) > 1e-9
        frame['estimated_heading_deg'] = estimated
        frame['heading_error_deg'] = wrap_deg(estimated-frame['true_heading_deg']) if estimated is not None else None
        i = bisect_right(cmd_times, t)-1
        command = commands[i] if i >= 0 else None
        # Commands beyond the final mission timestamp are never extrapolated.
        active = command is not None and command['phase'] == 'analysis' and analysis_end is not None and t <= analysis_end
        frame['target'] = [command['target_n'], -command['target_e'], command['target_alt']] if active else None
        airborne = airborne or p[2] > 2
        if active and airborne and p[2] <= .3 and contact_t is None:
            contact_t = t
        valid = active and (contact_t is None or t < contact_t)
        frame['analysis_valid'] = valid
        frame['waypoint_index'] = command.get('waypoint_index') if active else None
        frame.update(segment_error_m=None, target_distance_m=None, altitude_error_m=None)
        if valid:
            n, e, alt = p[0], -p[1], p[2]
            horizontal.append(segment_distance(n, e, command['start_n'], command['start_e'], command['target_n'], command['target_e']))
            hold.append(math.hypot(n-command['target_n'], e-command['target_e']))
            altitude.append(alt-command['target_alt'])
            frame.update(segment_error_m=horizontal[-1], target_distance_m=hold[-1], altitude_error_m=altitude[-1])
            if frame['heading_error_deg'] is not None:
                heading_errors.append(frame['heading_error_deg'])
            valid_window.append(t)
    metrics = dict(completed=final.get('passed', mission.get('passed')), mission_error=mission.get('error'),
                   contact_during_analysis_s=contact_t, contact_definition='truth altitude <= 0.3 m after exceeding 2 m, during analysis only',
                   valid_window_s=[valid_window[0], valid_window[-1]] if valid_window else None,
                   heading_error_deg=stats(heading_errors), segment_error_m=stats(horizontal),
                   target_distance_m=stats(hold), altitude_error_m=stats(altitude),
                   sample_weighting='uniform 20 Hz truth samples; imported recordings are sample-weighted',
                   telemetry_max_interpolation_gap_s=.5)
    audit = waypoint_audit(frames, spec, mission, offset)
    active_frames = [f for f in frames if f['attack_active']]
    def vector_peak(key):
        values = [math.sqrt(sum(x*x for x in f[key])) for f in frames if None not in f[key]]
        return max(values) if values else None
    peaks = {k: vector_peak(k) for k in ('mag_delta_gauss', 'gps_offset_neu', 'gyro_bias_rps', 'gyro_false_rps', 'wind')}
    peak_keys = {'magnetic': ('mag_delta_gauss',), 'gps': ('gps_offset_neu',),
                 'gyro': ('gyro_bias_rps', 'gyro_false_rps'), 'wind': ('wind',)}
    applied = any((peaks[k] or 0) > 1e-9 for k in peak_keys.get(attack.get('kind'), ()))
    window_stats = {}
    if attack:
        start, end = attack['start_s'], attack['start_s']+attack['duration_s']
        for name, pred in [('before', lambda t: t < start), ('during', lambda t: start <= t <= end), ('after', lambda t: t > end)]:
            subset = [f for f in frames if f['analysis_valid'] and pred(f['t'])]
            window_stats[name] = {key: stats([f[key] for f in subset if f[key] is not None])
                for key in ('heading_error_deg', 'segment_error_m', 'altitude_error_m', 'position_estimate_error_m')}
    metrics.update(waypoints=audit, fc_waypoints_reached=sum(w['fc_reached'] for w in audit),
        truth_waypoints_reached=sum(w['truth_reached'] for w in audit), planned_waypoints=len(audit),
        truth_route_completed=all(w['truth_reached'] for w in audit) if audit else None,
        rtl_home_error_m=math.hypot(frames[-1]['p'][0],frames[-1]['p'][1]) if mission.get('landed_boot_s') else None,
        position_estimate_error_m=stats([f['position_estimate_error_m'] for f in frames
                                       if f['analysis_valid'] and f['position_estimate_error_m'] is not None]),
        attack_kind=attack.get('kind'), attack_applied=applied,
        observed_attack_window_s=[active_frames[0]['t'],active_frames[-1]['t']] if active_frames else None,
        attack_schedule_fully_recorded=(frames[-1]['t'] >= attack['start_s']+attack['duration_s']) if attack else None,
        injection_peak_norms=peaks, attack_window_metrics=window_stats)
    return dict(name=meta.get('name', directory.name), metadata=meta,
                source_sha256=sha256(raw), frames=frames, commands=commands, metrics=metrics)


def export(paths, output):
    cases = [make_case(path) for path in paths]
    analysis_hash = sha256(Path(__file__))
    value = dict(schema_version=1, analysis_source_sha256=analysis_hash, time_axis='absolute simulation seconds; no per-case time warping',
                 position_frame='NWU', quaternion='body FLU to world NWU, wxyz', cases=cases)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_json(output, value)
    windows = [c['metrics']['valid_window_s'] for c in cases]
    common = [max(w[0] for w in windows), min(w[1] for w in windows)] if all(windows) else None
    if common and common[0] > common[1]:
        common = None
    comparable = []
    for case in cases:
        frames = [f for f in case['frames'] if common and common[0] <= f['t'] <= common[1] and f['analysis_valid']]
        comparable.append(dict(name=case['name'], **{key: stats([f[key] for f in frames if f[key] is not None])
            for key in ('heading_error_deg', 'segment_error_m', 'target_distance_m', 'altitude_error_m')}))
    completed = [c['metrics']['completed'] for c in cases]
    report = dict(schema_version=1, analysis_source_sha256=analysis_hash, cases=[{'name': c['name'], **c['metrics']} for c in cases],
                  completion_counts=dict(passed=sum(x is True for x in completed),
                                         failed=sum(x is False for x in completed),
                                         unknown=sum(x is None for x in completed)),
                  common_comparison_window_s=common, common_window_metrics=comparable)
    with output.with_name(output.stem + '.metrics.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['case', 'completed', 'heading_p95_deg', 'segment_rms_m', 'altitude_rms_m', 'analysis_samples', 'fc_waypoints', 'truth_waypoints', 'rtl_home_error_m', 'attack_applied', 'position_estimate_rms_m'])
        for case in cases:
            m = case['metrics']
            writer.writerow([case['name'], m['completed'], m['heading_error_deg']['p95_abs'],
                             m['segment_error_m']['rms'], m['altitude_error_m']['rms'], m['segment_error_m']['count'],
                             m['fc_waypoints_reached'],m['truth_waypoints_reached'],m['rtl_home_error_m'],
                             m['attack_applied'],m['position_estimate_error_m']['rms']])
    write_json(output.with_name(output.stem + '.metrics.json'), report)
    return value


def live_frame(frame, sequence=0):
    return dict(t=frame['t'], px=frame['p'][0], py=frame['p'][1], pz=frame['p'][2],
                **{f'q{i}': value for i, value in enumerate(frame['q'])},
                **{f'omega_m{i+1}': (value or 0) for i, value in enumerate(frame['omega'])},
                frame=sequence, wall_ms=0, input_mode='observer',
                attack_gate=frame.get('attack_gate') or 0,
                input_message='Read-only FMU truth; mission process controls flight')
