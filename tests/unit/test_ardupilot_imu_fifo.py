"""Exercise the production IMU FIFO hook against a virtual clock and memory sink."""
import ctypes
from pathlib import Path
import shutil
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def fifo(tmp_path):
    if shutil.which('cc') is None:
        pytest.skip('requires a C compiler')
    source = (REPO / 'virtuals/physics/flight_controllers/ardupilot/ardupilot.c').read_text()
    start = source.index('void ins_block_read(')
    end = source.index('\n}\n', start) + 3
    harness = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "phy.h"
#define ARM_V7M_R0 0
#define ARM_V7M_R1 1
#define ARM_V7M_R2 2
#define ARM_V7M_R3 3
#define ARM_V7M_LR 14
#define ARM_V7M_PC 15
#define TEMP_ZERO_C_INV 0
uint32_t registers[16];
uint64_t now_ns;
uint8_t memory[512];
uint32_t written;
int fmu_active = 1;
uint32_t qemu_get_register(int r) { return registers[r]; }
void qemu_set_register(uint32_t v, int r) { registers[r] = v; }
uint64_t qemu_plugin_get_virtual_timer(void) { return now_ns; }
void qemu_plugin_write_memory(uint32_t p, const void *v, uint32_t n) {
    (void)p; written = n; memcpy(memory, v, n);
}
int phy_backend_is(const char *name) { (void)name; return fmu_active; }
phy_body_frame_t phy_get_imu_frame(void) { return PHY_BODY_FRAME_FRD; }
int phy_get_imu_batch(imu_batch_t *batch) { memset(batch, 0, sizeof(*batch)); return 1; }
void fifo_count_to_bytes(uint16_t n, uint8_t *v) { v[0] = n >> 8; v[1] = n & 255; }
void convert_to_invensense(float *v, int16_t *o) {
    (void)v; for (int i=0; i<7; i++) o[i]=i+1;
}
void convert_int16_array_to_be_bytes(int16_t *v, int n, uint8_t *o) {
    for (int i=0; i<n; i++) { o[2*i]=v[i]>>8; o[2*i+1]=v[i]&255; }
}
'''
    harness += source[start:end]
    for name in ('micros32', 'chibiOS_tick_handler'):
        a = source.index(f'void {name}(')
        b = source.index('\n}\n', a) + 3
        harness += source[a:b]
    harness += r'''
int poll(uint32_t device, uint64_t t, uint32_t reg, uint32_t size) {
    registers[0]=device; registers[1]=reg; registers[2]=0; registers[3]=size;
    now_ns=t; written=0; memset(memory, 0xCD, sizeof(memory));
    ins_block_read(0, NULL);
    if (!registers[0]) return -1;
    return reg == 0x72 ? ((memory[0]<<8)|memory[1]) : (int)written;
}
'''
    c = tmp_path / 'fifo.c'
    c.write_text(harness)
    so = tmp_path / 'fifo.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-I', str(REPO / 'virtuals/physics'),
                    str(c), '-o', str(so)], check=True)
    lib = ctypes.CDLL(str(so))
    lib.poll.argtypes = [ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint32, ctypes.c_uint32]
    return lib


def test_fifo_counts_follow_elapsed_time_per_device(fifo):
    assert fifo.poll(1, 1_000_000, 0x72, 2) == 0
    assert fifo.poll(2, 1_000_000, 0x72, 2) == 0
    assert fifo.poll(1, 2_000_000, 0x72, 2) == 8 * 14
    assert fifo.poll(2, 2_000_000, 0x72, 2) == 8 * 14
    assert fifo.poll(1, 2_000_000, 0x72, 2) == 0
    assert fifo.poll(1, 2_062_500, 0x72, 2) == 0
    assert fifo.poll(1, 2_125_000, 0x72, 2) == 14
    assert fifo.poll(1, 100_000_000, 0x72, 2) == 32 * 14
    assert fifo.poll(1, 1_000_000, 0x72, 2) == 0


def test_fifo_reads_fill_exact_requested_size_and_reject_invalid_sizes(fifo):
    for count in (1, 8, 24, 32):
        assert fifo.poll(1, 1_000_000, 0x74, count * 14) == count * 14
        memory = (ctypes.c_uint8 * 512).in_dll(fifo, 'memory')
        assert bytes(memory[:count*14]) == bytes([0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7]) * count
        assert memory[count*14] == 0xCD
    for size in (0, 13, 15, 449):
        assert fifo.poll(1, 1_000_000, 0x74, size) == -1
        assert ctypes.c_uint32.in_dll(fifo, 'written').value == 0


def test_legacy_backend_keeps_its_fifo_count(fifo):
    ctypes.c_int.in_dll(fifo, 'fmu_active').value = 0
    assert fifo.poll(1, 1_000_000, 0x72, 2) == 24 * 14


def test_chibios_counter_matches_microseconds_and_wraps_at_32_bits(fifo):
    clock = ctypes.c_uint64.in_dll(fifo, 'now_ns')
    registers = (ctypes.c_uint32 * 16).in_dll(fifo, 'registers')
    for nanoseconds in (1_234_567_890, (2**32 + 123) * 1000):
        clock.value = nanoseconds
        fifo.chibiOS_tick_handler(0, None)
        assert registers[0] == (nanoseconds // 1000) & 0xFFFFFFFF
