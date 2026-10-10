"""The same physical magnetic vector must reach firmware for FRD and FLU plants."""
import ctypes
from pathlib import Path
import shutil
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[2]


class Vector(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ('x', 'y', 'z')]


class Raw(ctypes.Structure):
    _fields_ = [(name, ctypes.c_int16) for name in ('x', 'y', 'z')]


def test_magnetometer_backend_frames_produce_identical_wire_data(tmp_path):
    if shutil.which('cc') is None:
        pytest.skip('requires a C compiler')
    # Compile the production conversion and packing routines, without QEMU hooks.
    source = (REPO / 'virtuals/physics/flight_controllers/ardupilot/ardupilot.c').read_text()
    start = source.index('typedef struct\n{\n    float mag_x_gauss;')
    end = source.index('typedef struct\n{\n    const float diagonals[3];', start)
    harness = tmp_path / 'mag.c'
    harness.write_text('#include <stdint.h>\n'
                       '#define PHY_BODY_FRAME_FLU 0\n'
                       'int frame = 0;\n'
                       'int phy_get_imu_frame(void) { return frame; }\n'
                       + source[start:end])
    library = tmp_path / 'mag.so'
    subprocess.run(['cc', '-shared', '-fPIC', str(harness), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.convert_to_hmc5843.argtypes = [Vector]
    lib.convert_to_hmc5843.restype = Raw
    lib.pack_to_hmc5843_wire_format.argtypes = [Raw, ctypes.POINTER(ctypes.c_uint8)]
    frames = ctypes.c_int.in_dll(lib, 'frame')
    frames.value = 1  # FRD
    frd = lib.convert_to_hmc5843(Vector(0.21, 0.1, 0.45))
    frames.value = 0  # FLU, same physical vector
    flu = lib.convert_to_hmc5843(Vector(0.21, -0.1, -0.45))
    a, b = (ctypes.c_uint8 * 6)(), (ctypes.c_uint8 * 6)()
    lib.pack_to_hmc5843_wire_format(frd, a)
    lib.pack_to_hmc5843_wire_format(flu, b)
    assert bytes(a) == bytes(b)
    # Decode the register vector in firmware's body axes (sensor XY mounting).
    assert -frd.y / 1090 == pytest.approx(0.21, abs=0.001)
    assert frd.x / 1090 == pytest.approx(0.1, abs=0.001)
    assert frd.z / 1090 == pytest.approx(0.45, abs=0.001)
