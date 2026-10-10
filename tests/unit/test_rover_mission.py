"""Rover completion and persistent readiness telemetry regression tests."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

PATH = Path(__file__).resolve().parents[2] / 'virtuals/physics/flight_controllers/courbet/mavlink/mav_command_and_control.py'
spec = importlib.util.spec_from_file_location('rover_mission_helper', PATH)
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)


def message(kind, system=1, **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system, **fields)


class Connection:
    target_system = 1

    def __init__(self, messages=()):
        self.messages = iter(messages)

    def recv_match(self, **kwargs):
        return next(self.messages, None)


def test_final_target_selection_is_not_arrival():
    state = {}
    connection = Connection()
    mission.handle_runtime_message(connection, message('MISSION_CURRENT', seq=4), count=5, state=state)
    assert state['final_item_seen']
    assert not state.get('final_item_reached')
    mission.handle_runtime_message(connection, message('MISSION_ITEM_REACHED', seq=3), count=5, state=state)
    assert not state.get('final_item_reached')
    mission.handle_runtime_message(connection, message('MISSION_ITEM_REACHED', system=2, seq=4), count=5, state=state)
    assert not state.get('final_item_reached')
    mission.handle_runtime_message(connection, message('MISSION_ITEM_REACHED', seq=4), count=5, state=state)
    assert state['final_item_reached']


def test_waypoint_completion_ignores_ground_altitude():
    connection = Connection([message('GLOBAL_POSITION_INT', relative_alt=149000),
                             message('MISSION_ITEM_REACHED', seq=4)])
    mission.monitor(connection, 5, 1, exit_on_complete=True, completion='waypoints')


def test_waypoint_timeout_is_failure():
    with pytest.raises(RuntimeError, match='final waypoint not reached'):
        mission.monitor(Connection(), 5, 0, exit_on_complete=True, completion='waypoints')


def test_ekf_report_can_confirm_fusion_without_startup_text():
    flags = mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_VELOCITY_HORIZ | mavutil.mavlink.EKF_POS_HORIZ_ABS
    connection = Connection([message('EKF_STATUS_REPORT', flags=flags)])
    mission.wait_for_gps_fusion(connection, 5, 1)
    state = {}
    mission.handle_runtime_message(connection, message('EKF_STATUS_REPORT', flags=mavutil.mavlink.EKF_ATTITUDE), state=state)
    assert state['ekf_ready'] is False
