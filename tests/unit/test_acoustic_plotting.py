"""Check the body-to-world quaternion and wrapped-angle plotting conventions."""

from __future__ import annotations

import importlib.util

import pytest

from fastdyn import paths

np = pytest.importorskip('numpy')
source = paths.virtuals_dir()/'gyro_acoustic/tools/plot_flight.py'
spec = importlib.util.spec_from_file_location('acoustic_plot_flight', source)
plots = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plots)


def test_hamilton_body_to_ned_axis_rotations():
    c = np.sqrt(.5)
    # Pitch +/-90 is a nonunique Euler representation; use a nonsingular pitch.
    q = np.array([[1, 0, 0, 0], [c, c, 0, 0],
                  [np.cos(np.pi/6), 0, np.sin(np.pi/6), 0], [c, 0, 0, c]])
    actual = np.rad2deg(plots.quaternion_to_euler(q))
    expected = [[0, 0, 0], [90, 0, 0], [0, 60, 0], [0, 0, 90]]
    np.testing.assert_allclose(actual, expected, atol=1e-6)
    np.testing.assert_allclose(plots.quaternion_to_euler(-2*q),
                               plots.quaternion_to_euler(q), atol=1e-12)


def test_wrap_breaks_are_not_drawn_as_angular_excursions():
    result = plots.break_angle_wraps(np.array([177., 179., -179., -177.]))
    assert np.isnan(result[2])
    np.testing.assert_equal(result[[0, 1, 3]], [177, 179, -177])


def test_zero_quaternion_is_not_presented_as_attitude():
    with pytest.raises(ValueError, match='Zero quaternion'):
        plots.quaternion_to_euler(np.zeros((1, 4)))
