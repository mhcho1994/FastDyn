"""Check the reduced acoustic response against analytic and converged solutions."""

from __future__ import annotations

import ctypes
from pathlib import Path
import subprocess
import sys

import pytest

from fastdyn import paths
from fastdyn.virtual_preprocessing import RunContext, VirtualPreparationError, _load_run_plugins


ROOT = paths.repo_root()
_load_run_plugins()
# Use the registered module loaded by the normal discovery mechanism.
HOST = next(module for name, module in sys.modules.items()
            if name.endswith('gyro_acoustic.host.preprocessor'))


@pytest.fixture(scope='module')
def response(tmp_path_factory):
    directory = tmp_path_factory.mktemp('acoustic-native')
    source = directory/'probe.c'
    source.write_text('''
#include "response.h"
static AcousticResponse s;
void setup(const double *p) {
    s = (AcousticResponse){0};
    for (int i=0;i<ACOUSTIC_FIELDS;i++) s.p[i]=p[i];
}
double raw(double t) { return acoustic_raw(&s,t); }
double step(double t, double dt) { acoustic_integrate(&s,t,dt); return s.filter2; }
double frequency(double t) { return acoustic_frequency(&s,t); }
''', encoding='utf-8')
    library = directory/'probe.so'
    subprocess.run(['cc', '-std=c11', '-O2', '-shared', '-fPIC', str(source),
                    '-I', str(ROOT/'virtuals/gyro_acoustic/runtime'), '-lm', '-o', str(library)],
                   check=True)
    lib = ctypes.CDLL(str(library))
    lib.setup.argtypes = [ctypes.POINTER(ctypes.c_double)]
    for name, arguments in [('raw', 1), ('step', 2), ('frequency', 1)]:
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_double]*arguments
        fn.restype = ctypes.c_double
    return lib


def setup(lib, **overrides):
    p = HOST.resolve_settings({'enabled': True, **overrides})
    values = [p[key] for key in HOST.DEFAULTS] + p['axis_coupling']
    lib.setup((ctypes.c_double*len(values))(*values))
    return p


def test_resonant_tone_and_zero_pressure(response):
    import math
    p = setup(response, start_frequency_hz=25000., end_frequency_hz=25000., ramp_s=0.)
    expected = -math.sqrt(2)*p['rms_pressure_pa']*p['effective_area_m2'] / (
        2*p['mems_mass_kg']*p['drive_amplitude_m']*2*math.pi*p['reference_frequency_hz'])
    assert response.raw(0.1) == pytest.approx(expected, rel=1e-12)
    assert response.step(0., 0.1) == pytest.approx(expected, rel=1e-10)
    setup(response, rms_pressure_pa=0.)
    assert response.step(0., 20.1) == 0.


def test_chirp_phase_convergence_and_filter_decay(response):
    setup(response)
    assert response.frequency(0.) == 23000.
    assert response.frequency(10.) == 25000.
    assert response.frequency(20.) == 27000.
    # At t=10 the integrated beat is -10000 cycles, yielding negative DC at resonance.
    assert response.raw(10.) < -17.9
    coarse = []
    setup(response)
    for k in range(2000):
        coarse.append(response.step(k*0.01, 0.01))
    setup(response, integration_step_s=5e-6)
    fine = [response.step(k*0.01, 0.01) for k in range(2000)]
    assert max(abs(a-b) for a,b in zip(coarse, fine)) < 0.0001
    assert abs(response.step(20., 0.15)) < 1e-10


@pytest.mark.parametrize('setting', [
    {'rms_pressure_pa': -1}, {'duration_s': 0}, {'phase_rad': float('nan')},
    {'axis_coupling': [1, 0]}, {'integration_step_s': 0.001}, {'enabled': 'false'},
    {'end_frequncy_hz': 27000}, {'ramp_s': 11},
])
def test_invalid_settings(setting):
    with pytest.raises(VirtualPreparationError):
        HOST.resolve_settings(setting)


def test_serialized_configuration(tmp_path):
    ctx = RunContext(binary=Path('unused.elf'), architecture='arm', machine='cortexm',
                     cpu='cortex-m7', workdir=tmp_path, plugin_name='gyro_acoustic',
                     settings={'enabled': True, 'rms_pressure_pa': 0.})
    artifacts = HOST.AcousticPreprocessor().prepare(ctx).artifacts
    assert len(artifacts[0].read_text().splitlines()) == 22
    assert float(artifacts[0].read_text().splitlines()[1]) == 0.
