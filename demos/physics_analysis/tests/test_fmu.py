"""Real FMI3 ABI test; opt in with PHYSICS_DEMO_TEST_BUILD=/path/to/build."""
import ctypes as c
import csv
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

BUILD=os.environ.get('PHYSICS_DEMO_TEST_BUILD')
pytestmark=pytest.mark.skipif(not BUILD,reason='set PHYSICS_DEMO_TEST_BUILD to test a compiled FMU')


class Fmu:
    def __init__(self, folder):
        root=Path(BUILD)/folder/'PhysicsAnalysis_Copter'
        meta=ET.parse(root/'modelDescription.xml').getroot()
        self.refs={x.get('name'):int(x.get('valueReference')) for x in meta.findall('ModelVariables/Float64')}
        identifier=meta.find('CoSimulation').get('modelIdentifier')
        self.lib=c.CDLL(str(next((root/'binaries').glob(f'*/{identifier}.so'))))
        def bind(name,restype,args):
            f=getattr(self.lib,name);f.restype=restype;f.argtypes=args;return f
        P,B,D,S=c.c_void_p,c.c_bool,c.c_double,c.c_size_t
        U=c.c_uint32
        self.instantiate=bind('fmi3InstantiateCoSimulation',P,[c.c_char_p,c.c_char_p,c.c_char_p,B,B,B,B,c.POINTER(U),S,P,P,P])
        self.init=bind('fmi3EnterInitializationMode',c.c_int,[P,B,D,D,B,D])
        self.exitinit=bind('fmi3ExitInitializationMode',c.c_int,[P])
        self.step=bind('fmi3DoStep',c.c_int,[P,D,D,B,c.POINTER(B),c.POINTER(B),c.POINTER(B),c.POINTER(D)])
        self.set=bind('fmi3SetFloat64',c.c_int,[P,c.POINTER(U),S,c.POINTER(D),S])
        self.get=bind('fmi3GetFloat64',c.c_int,[P,c.POINTER(U),S,c.POINTER(D),S])
        self.free=bind('fmi3FreeInstance',None,[P])
        self.instance=self.instantiate(b'test',meta.get('instantiationToken').encode(),None,False,False,False,False,None,0,None,None,None)
        assert self.instance

    def value(self,name,values=None,count=1):
        vr=c.c_uint32(self.refs[name])
        array=(c.c_double*(len(values) if values is not None else count))(*(values or []))
        if values is None:assert self.get(self.instance,c.byref(vr),1,array,count) == 0
        else:assert self.set(self.instance,c.byref(vr),1,array,len(values)) == 0
        return list(array)

    def start(self, parameters=None):
        assert self.init(self.instance,False,0,0,False,0) == 0
        for name,value in (parameters or {}).items():
            self.value(name,[value])
        self.value('pwm',[1450]*4)
        assert self.exitinit(self.instance) == 0

    def tick(self, index, dt=.001):
        event,terminate,early=c.c_bool(),c.c_bool(),c.c_bool()
        last=c.c_double()
        assert self.step(self.instance,index*dt,dt,True,c.byref(event),c.byref(terminate),c.byref(early),c.byref(last)) == 0

    def simulate(self, parameters=None):
        self.start(parameters)
        states=[]
        for i in range(1000):
            self.tick(i)
            if i%50==0:states.append(self.value('vehicle.plant.p',count=3)+self.value('vehicle.plant.q',count=4))
        self.free(self.instance)
        return states


def test_observer_preserves_physics_and_optional_values(tmp_path,monkeypatch):
    monkeypatch.delenv('PHYSICS_DEMO_TRUTH_PATH',raising=False)
    pristine=Fmu('pristine').simulate()
    assert pristine == Fmu('observed').simulate()
    path=tmp_path/'truth.csv'
    monkeypatch.setenv('PHYSICS_DEMO_TRUTH_PATH',str(path))
    assert pristine == Fmu('observed').simulate()
    with path.open() as f:rows=list(csv.DictReader(f))
    assert len(rows) == 20
    assert all(r['current_a']==r['voltage_v']=='' for r in rows)
    assert all(float(r['model_current_a']) > 0 for r in rows)
    assert all(float(b['sim_time_s'])>float(a['sim_time_s']) for a,b in zip(rows,rows[1:]))
    monkeypatch.delenv('PHYSICS_DEMO_TRUTH_PATH')
    assert Fmu('pristine').simulate({'wind_mean_mps':10,'attack_start_s':0}) != pristine


@pytest.mark.parametrize('parameters', [
    {'magnetic_scale':1}, {'gps_north_m':12,'gps_east_m':-8,'gps_up_m':2},
    {'gyro_bias_yaw_dps':3,'gyro_false_yaw_dps':8,'gyro_false_roll_dps':2},
])
def test_sensor_attacks_leave_open_loop_truth_unchanged(parameters,monkeypatch):
    monkeypatch.delenv('PHYSICS_DEMO_TRUTH_PATH',raising=False)
    base=Fmu('pristine').simulate()
    parameters={**parameters,'attack_start_s':.1,'attack_duration_s':.6,'attack_ramp_s':.1}
    assert Fmu('pristine').simulate(parameters) == base


def test_injections_units_frames_and_smooth_gps_velocity(monkeypatch):
    import math
    monkeypatch.delenv('PHYSICS_DEMO_TRUTH_PATH',raising=False)
    f=Fmu('observed')
    f.start(dict(attack_start_s=.1,attack_duration_s=.6,attack_ramp_s=.1,
        magnetic_scale=1,gps_north_m=12,gps_east_m=-8,gps_up_m=2,
        gyro_bias_yaw_dps=3,gyro_false_yaw_dps=8,gyro_false_roll_dps=2))
    previous=None
    gates=[]
    for i in range(900):
        f.tick(i)
        t=(i+1)*.001
        gate=f.value('attack.gate')[0];gates.append(gate)
        assert 0 <= gate <= 1
        for output,base,delta in [('mag','vehicle.mag','attack.mag_delta_frd')]:
            assert f.value(output,count=3) == pytest.approx([a+b for a,b in zip(f.value(base,count=3),f.value(delta,count=3))])
        bias=f.value('attack.gyro_bias_frd',count=3)
        false=f.value('attack.gyro_false_frd',count=3)
        assert bias == pytest.approx([0,0,gate*3*math.pi/180])
        assert false == pytest.approx([gate*math.sin(2*math.pi*.25*(t-.1))*v*math.pi/180 for v in [2,0,8]],abs=1e-10)
        assert f.value('gyro',count=3) == pytest.approx([a+b+d for a,b,d in zip(f.value('vehicle.gyro',count=3),bias,false)])
        gps=f.value('gps',count=3);real=f.value('vehicle.gps',count=3)
        offset=f.value('attack.gps_offset_neu',count=3)
        velocity=f.value('attack.gps_velocity_neu',count=3)
        assert [(gps[0]-real[0])*math.pi/180*6378137,
                (gps[1]-real[1])*math.pi/180*6378137*math.cos(40.414929*math.pi/180),gps[2]-real[2]] == pytest.approx(offset,abs=2e-8)
        assert f.value('vel_ned',count=3) == pytest.approx([a+b for a,b in zip(f.value('vehicle.vel_ned',count=3),[velocity[0],velocity[1],-velocity[2]])])
        if previous is not None and .102 < t < .198:
            # Trapezoidal integration of the analytic derivative agrees with position.
            assert [(a-b)/.001 for a,b in zip(offset,previous[0])] == pytest.approx([(a+b)/2 for a,b in zip(velocity,previous[1])],abs=.025)
        previous=(offset,velocity)
        assert f.value('yaw_deg') == [0]
        assert f.value('accel',count=3) == f.value('vehicle.accel',count=3)
        assert f.value('baro_altitude_m') == f.value('vehicle.baro_altitude_m')
    assert max(gates) == 1 and gates[0] == gates[-1] == 0
    f.free(f.instance)
