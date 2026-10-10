"""GUIDED flight scenarios on the existing ArduCopter firmware.

Only the demo process sends commands. The viewer is an observer.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path
import sys
import time

from .common import ROOT, read_json, write_json


def execute(run: Path, endpoint: str):
    from pymavlink import mavutil
    sys.path.insert(0, str(ROOT / 'virtuals/physics/flight_controllers/courbet/mavlink'))
    import mav_command_and_control as helper

    spec = read_json(run / 'scenario.json')
    boot_offset = read_json(run / 'run.json').get('boot_offset_s', 0)
    state = dict(t=None, north=None, east=None, alt=None, armed=False, mode='UNKNOWN')
    origin = (40.414929, -86.932387)
    radius = 6378137.0
    phase = 'initializing'
    started = time.monotonic()
    files = [open(run / name, 'x', buffering=1) for name in
             ('telemetry.csv', 'commands.csv', 'events.jsonl')]
    telemetry = csv.writer(files[0])
    telemetry.writerow(['time_boot_s', 'message', 'yaw_deg', 'north_m', 'east_m', 'alt_m', 'armed', 'mode'])
    commands = csv.writer(files[1])
    commands.writerow(['time_boot_s', 'phase', 'start_n', 'start_e', 'target_n', 'target_e', 'target_alt', 'waypoint_index'])
    mav = None
    result = {'passed': False, 'reached': [], 'error': 'mission interrupted'}

    def event(name, **details):
        import json
        files[2].write(json.dumps(dict(t=state['t'], wall_s=time.monotonic()-started,
                                      phase=phase, event=name, **details)) + '\n')
        print(f'[physics-demo] {name} {details}', flush=True)

    def receive_hook(_mav, msg):
        kind = msg.get_type()
        if kind == 'HEARTBEAT' and msg.get_srcSystem() == _mav.target_system:
            state.update(armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                         mode=mavutil.mode_string_v10(msg))
        elif kind == 'GLOBAL_POSITION_INT':
            t = msg.time_boot_ms / 1000.0
            if state['t'] is not None and t < state['t']:
                raise RuntimeError('firmware boot clock moved backwards')
            state.update(t=t, north=math.radians(msg.lat/1e7-origin[0])*radius,
                         east=math.radians(msg.lon/1e7-origin[1])*radius*math.cos(math.radians(origin[0])),
                         alt=msg.relative_alt/1000)
            telemetry.writerow([t, kind, '', state['north'], state['east'], state['alt'],
                                int(state['armed']), state['mode']])
        elif kind == 'ATTITUDE':
            telemetry.writerow([msg.time_boot_ms/1000.0, kind, math.degrees(msg.yaw),
                                '', '', '', int(state['armed']), state['mode']])
        elif kind == 'STATUSTEXT':
            event('status', text=str(msg.text))

    def wait_until(predicate, wall_timeout, action=None):
        deadline = time.monotonic() + wall_timeout
        next_action = 0
        while time.monotonic() < deadline:
            if action and time.monotonic() >= next_action:
                action()
                next_action = time.monotonic() + 2
            mav.recv_match(blocking=True, timeout=0.2)
            if predicate():
                return
        raise TimeoutError(f'timed out in {phase}')

    def mode(name):
        wait_until(lambda: state['mode'] == name, 40, lambda: helper.set_mode(mav, name))

    def target(north, east, altitude, segment_start):
        lat = origin[0] + math.degrees(north/radius)
        lon = origin[1] + math.degrees(east/(radius*math.cos(math.radians(origin[0]))))
        mask = sum(1 << bit for bit in (3, 4, 5, 6, 7, 8, 10, 11))
        mav.mav.set_position_target_global_int_send(
            int(state['t']*1000), mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
            round(lat*1e7), round(lon*1e7), altitude, 0, 0, 0, 0, 0, 0, 0, 0)
        commands.writerow([state['t'], phase, *segment_start, north, east, altitude, index])

    try:
        mav = helper.connect(endpoint, 180)
        mav.message_hooks.append(receive_hook)
        helper.wait_for_autopilot_ready(mav, 60)
        helper.load_params(mav, run / 'parameters.param')
        for message_id, hz in ((33, 10), (30, 20)):
            mav.mav.command_long_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                     0, message_id, 1e6/hz, 0, 0, 0, 0, 0)
        phase = 'arming'
        mode('STABILIZE')

        def arm():
            mav.mav.command_long_send(mav.target_system, mav.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE,
                                     0, 1, 0, 0, 0, 0, 0, 0)
            helper.request_arm(mav, force=True)
        wait_until(lambda: state['armed'], 60, arm)
        helper.wait_for_gps_fusion(mav, 0, 60)
        mode('GUIDED')
        if not state['armed']:
            raise RuntimeError('firmware disarmed before takeoff')
        phase = 'takeoff'
        altitude = spec['altitude_m']
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                 0, 0, 0, 0, 0, 0, 0, altitude)
        wait_until(lambda: state['alt'] is not None and state['alt'] >= altitude-1, 150)
        event('takeoff_complete')
        if spec['attack']['kind'] != 'none' and state['t'] + boot_offset >= spec['attack']['start_s']:
            raise RuntimeError('attack started before waypoint flight; increase attack.start_s')
        result['analysis_start_boot_s'] = state['t']
        start = state['t']
        wall_deadline = time.monotonic() + max(180, spec['duration_s']*10)
        phase = 'analysis'
        points = spec['waypoints_ne_m']
        index, hold_start, last_send = 0, None, -math.inf
        segment_start = [0, 0]
        while state['t']-start < spec['duration_s'] and time.monotonic() < wall_deadline:
            mav.recv_match(blocking=True, timeout=0.2)
            t = state['t']
            if not state['armed']:
                raise RuntimeError('unexpected disarm during analysis')
            north, east = points[index]
            if t-last_send >= 0.2:
                target(north, east, altitude, segment_start)
                last_send = t
            arrived = (math.hypot(state['north']-north, state['east']-east) <= spec['acceptance_radius_m']
                       and abs(state['alt']-altitude) <= spec['altitude_tolerance_m'])
            hold_start = (t if hold_start is None else hold_start) if arrived else None
            if hold_start is not None and t-hold_start >= spec['hold_s']:
                result['reached'].append({'index': index, 'time_boot_s': t})
                event('waypoint_reached', index=index, number=index+1)
                segment_start = list(points[index])
                index += 1
                hold_start = None
                if index == len(points):
                    break
        if time.monotonic() >= wall_deadline:
            raise TimeoutError('analysis wall watchdog expired')
        if index != len(points):
            raise TimeoutError('route did not complete within simulation-time budget')
        result['analysis_end_boot_s'] = state['t']
        event('analysis_complete')
        phase = 'rtl'
        # A final marker prevents carrying a stale flight target into landing metrics.
        commands.writerow([state['t'], phase, *points[-1], 0, 0, altitude, -1])
        event('rtl_requested')
        mode('RTL')
        ground_since = [None]
        def disarm_when_landed():
            if state['alt'] is not None and state['alt'] <= .3:
                if ground_since[0] is None:
                    ground_since[0] = state['t']
                if state['t']-ground_since[0] >= 5:
                    # Manual mode permits an ordinary disarm after a sustained landing.
                    # The original parameters disable automatic disarm while waiting for GPS.
                    if state['mode'] != 'STABILIZE':
                        helper.set_mode(mav, 'STABILIZE')
                        return
                    # Normal disarm request only; firmware may reject it in flight.
                    mav.mav.command_long_send(mav.target_system, mav.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            else:
                ground_since[0] = None
        wait_until(lambda: not state['armed'] and state['alt'] <= 0.5, 240, disarm_when_landed)
        result.update(passed=True, error=None, landed_boot_s=state['t'])
        event('landed')
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        event('failed', error=result['error'])
        if mav is not None:
            # Best effort; process shutdown still follows when the helper exits.
            helper.set_mode(mav, 'LAND')
    finally:
        result['last_boot_s'] = state['t']
        write_json(run / 'mission.json', result)
        if mav is not None:
            mav.close()
        for f in files:
            f.close()
    return 0 if result['passed'] else 1
