import csv
import math
import pytest

from physics_demo.common import heading, write_json
from physics_demo.data import load_truth, make_case, segment_distance, interpolate_heading
from physics_demo.runner import campaign_cases
from physics_demo.server import TruthTail


def truth_file(path, rows):
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sim_time_s','north_m','west_m','alt_m','qw','qx','qy','qz','current_a','voltage_v'])
        for t, altitude, sign in rows:
            w.writerow([t, 1, 2, altitude, sign, 0, 0, 0, '', ''])


def test_original_csv_optional_electrical_and_quaternion_sign(tmp_path):
    path=tmp_path/'truth.csv'
    truth_file(path, [(0, 2, 1), (.05, 3, -1)])
    frames=load_truth(path)
    assert frames[1]['q'] == frames[0]['q']
    assert frames[0]['current_a'] is frames[0]['voltage_v'] is None
    assert frames[0]['p'] == [1,2,2]


def test_heading_frame_and_wrap():
    assert heading([math.sqrt(.5),0,0,math.sqrt(.5)]) == pytest.approx(-90)
    samples=[(0,179),(.1,-179)]
    assert abs(interpolate_heading(.05,samples,[0,.1])) == 180
    assert interpolate_heading(-.01,samples,[0,.1]) is None
    assert interpolate_heading(.2,samples,[0,.1]) is None
    assert interpolate_heading(.5,[(0,0),(1,90)],[0,1]) is None


def test_active_segment_and_ground_contact(tmp_path):
    truth_file(tmp_path/'truth.csv',[(0,3,1),(.05,3,1),(.1,.2,1),(.15,4,1)])
    (tmp_path/'commands.csv').write_text('time_boot_s,phase,start_n,start_e,target_n,target_e,target_alt\n0,analysis,0,0,10,0,3\n')
    write_json(tmp_path/'mission.json',dict(passed=False,last_boot_s=.15,error='interrupted'))
    case=make_case(tmp_path)
    assert case['metrics']['segment_error_m']['count'] == 2
    assert case['metrics']['contact_during_analysis_s'] == .1
    assert not case['frames'][-1]['analysis_valid']
    assert segment_distance(5,3,0,0,10,0) == 3
    assert segment_distance(12,0,0,0,10,0) == 2
    assert segment_distance(3,4,0,0,0,0) == 5


def test_import_has_no_invented_completion_or_metrics(tmp_path):
    truth_file(tmp_path/'truth.csv',[(0,3,1),(.05,3,1)])
    case=make_case(tmp_path/'truth.csv')
    assert case['metrics']['completed'] is None
    assert case['metrics']['heading_error_deg']['rms'] is None
    assert case['metrics']['segment_error_m']['rms'] is None


def test_duplicate_and_invalid_truth_rejected(tmp_path):
    path=tmp_path/'truth.csv'
    truth_file(path,[(0,2,1),(0,2,1)])
    with pytest.raises(ValueError,match='strictly increasing'):load_truth(path)
    truth_file(path,[(0,2,0)])
    with pytest.raises(ValueError,match='quaternion norm'):load_truth(path)


def test_tail_buffers_partial_lines(tmp_path):
    path=tmp_path/'truth.csv'
    truth_file(path,[(0,2,1)])
    text=path.read_text(); path.write_text(text[:-1])
    tail=TruthTail(path);tail.poll()
    assert tail.latest is None
    with path.open('a') as f:f.write('\n')
    tail.poll()
    assert tail.latest['pz'] == 2
    assert tail.latest['frame'] == 1
    tail.poll();assert tail.sequence == 1


def test_campaign_is_reproducible_and_paired():
    a=campaign_cases('monte-carlo',42,3)
    assert a == campaign_cases('monte-carlo',42,3)
    assert a != campaign_cases('monte-carlo',43,3)
    cases=campaign_cases('attacks')
    assert len(cases) == 5
    assert {c['attack']['kind'] for c in cases} == {'none','magnetic','wind','gps','gyro'}
    assert all(c['waypoints_ne_m'] == cases[0]['waypoints_ne_m'] and len(c['waypoints_ne_m']) == 10 for c in cases)
    assert all(c['kind'] == 'waypoints' for c in cases+a)


def test_comparison_metrics_use_common_absolute_window(tmp_path):
    from physics_demo.data import export
    paths=[]
    for i, start in enumerate((0, .05)):
        path=tmp_path/str(i);path.mkdir();paths.append(path)
        truth_file(path/'truth.csv',[(0,3,1),(.05,3,1),(.1,3,1)])
        (path/'commands.csv').write_text('time_boot_s,phase,start_n,start_e,target_n,target_e,target_alt\n'
                                       f'{start},analysis,0,0,0,0,3\n')
        write_json(path/'mission.json',dict(passed=True,analysis_end_boot_s=.1,last_boot_s=.1))
    output=tmp_path/'replay.json'
    export(paths,output)
    from physics_demo.common import read_json
    report=read_json(tmp_path/'replay.metrics.json')
    assert report['common_comparison_window_s'] == [.05,.1]
    assert report['completion_counts'] == dict(passed=2,failed=0,unknown=0)
    assert all(m['segment_error_m']['count'] == 2 for m in report['common_window_metrics'])


def test_scenario_rejects_cross_case_parameter_and_invalid_envelope():
    from physics_demo.common import scenario, validate_scenario
    case=scenario('wind');case['attack']['gps_north_m']=2
    with pytest.raises(ValueError,match='unexpected'):validate_scenario(case)
    case=scenario('gps-spoof');case['attack']['ramp_s']=0
    with pytest.raises(ValueError,match='ramp'):validate_scenario(case)


def test_spoofed_arrival_is_not_truth_arrival(tmp_path):
    truth_file(tmp_path/'truth.csv',[(i*.05,3,1) for i in range(25)])
    spec=dict(waypoints_ne_m=[[10,0]],altitude_m=3,acceptance_radius_m=2,hold_s=1)
    write_json(tmp_path/'run.json',dict(name='spoof',scenario=spec))
    write_json(tmp_path/'mission.json',dict(passed=True,analysis_end_boot_s=1.2,
        reached=[dict(index=0,time_boot_s=1.1)],landed_boot_s=1.2))
    (tmp_path/'commands.csv').write_text('time_boot_s,phase,start_n,start_e,target_n,target_e,target_alt,waypoint_index\n0,analysis,0,0,10,0,3,0\n')
    case=make_case(tmp_path)
    assert case['metrics']['fc_waypoints_reached'] == 1
    assert case['metrics']['truth_waypoints_reached'] == 0
    assert case['metrics']['completed'] is True
    assert case['metrics']['truth_route_completed'] is False
    assert case['metrics']['waypoints'][0]['truth_distance_at_fc_arrival_m'] == pytest.approx(math.hypot(9,2))


def test_truth_dwell_is_contiguous_and_in_target_order():
    from physics_demo.data import waypoint_audit
    spec=dict(waypoints_ne_m=[[10,0],[20,0]],altitude_m=3,acceptance_radius_m=2,hold_s=1)
    frames=[dict(t=i*.05,p=[10,0,3],analysis_valid=True,waypoint_index=0) for i in range(30)]
    a=waypoint_audit(frames,spec,{},0)
    assert a[0]['truth_reached'] and not a[1]['truth_reached']
    frames[15]['p']=[20,0,3] # splits dwell; future waypoint cannot be counted
    a=waypoint_audit(frames,spec,{},0)
    assert not any(w['truth_reached'] for w in a)
