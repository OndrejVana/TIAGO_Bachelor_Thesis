# -*- coding: utf-8 -*-

from __future__ import print_function, division

import os
import numpy as np
import tempfile

from tiago_door_planning.utils import angle_wrap as wrap_angle_rad
from tiago_door_planning.feasibility import OfflineReachabilityMap


def make_test_map():
    x_bins = np.array([0.2, 0.4, 0.6], dtype=float)
    y_bins = np.array([-0.2, 0.0, 0.2], dtype=float)
    yaw_bins_rad = np.radians(np.array([-180.0, -90.0, 0.0, 90.0, 180.0], dtype=float))

    reachable = np.zeros((len(x_bins), len(y_bins), len(yaw_bins_rad)), dtype=np.uint8)

    # Mark a few cells reachable
    # x=0.4, y=0.0, yaw=0
    reachable[1, 1, 2] = 1
    # x=0.6, y=0.2, yaw=90
    reachable[2, 2, 3] = 1
    # x=0.2, y=-0.2, yaw=-90
    reachable[0, 0, 1] = 1

    m = OfflineReachabilityMap(
        x_bins=x_bins,
        y_bins=y_bins,
        yaw_bins_rad=yaw_bins_rad,
        reachable=reachable,
        fixed_z=1.0,
    )
    return m


def test_wrap_angle_rad_basic():
    assert np.isclose(wrap_angle_rad(0.0), 0.0)
    assert np.isclose(wrap_angle_rad(np.pi), -np.pi)
    assert np.isclose(wrap_angle_rad(-np.pi), -np.pi)
    assert np.isclose(wrap_angle_rad(3.0 * np.pi), -np.pi)


def test_nearest_index_query_hit():
    m = make_test_map()
    ok = m.query(0.4, 0.0, 0.0)
    assert ok is True


def test_nearest_index_query_miss():
    m = make_test_map()
    ok = m.query(0.4, 0.0, np.radians(90.0))
    assert ok is False


def test_query_nearest_bin_selection():
    m = make_test_map()
    ok = m.query(0.41, 0.02, np.radians(3.0))
    assert ok is True


def test_query_yaw_wraparound():
    m = make_test_map()

    # 360 deg should wrap to 0 deg
    ok = m.query(0.4, 0.0, np.radians(360.0))
    assert ok is True

    # -270 deg should wrap to 90 deg
    ok2 = m.query(0.6, 0.2, np.radians(-270.0))
    assert ok2 is True


def test_query_x_out_of_bounds_rejects():
    m = make_test_map()
    assert m.query(0.1, 0.0, 0.0) is False
    assert m.query(0.7, 0.0, 0.0) is False


def test_query_y_out_of_bounds_rejects():
    m = make_test_map()
    assert m.query(0.4, -0.5, 0.0) is False
    assert m.query(0.4, 0.5, 0.0) is False


def test_load_npz_roundtrip():
    m = make_test_map()

    fd, path = tempfile.mkstemp(suffix=".npz")
    os.close(fd)
    try:
        np.savez_compressed(
            path,
            x_bins=m.x_bins,
            y_bins=m.y_bins,
            yaw_bins_rad=m.yaw_bins_rad,
            reachable=m.reachable,
            fixed_z=np.array(1.0, dtype=float),
        )

        loaded = OfflineReachabilityMap.load_npz(path)

        assert np.allclose(loaded.x_bins, m.x_bins)
        assert np.allclose(loaded.y_bins, m.y_bins)
        assert np.allclose(loaded.yaw_bins_rad, m.yaw_bins_rad)
        assert np.array_equal(loaded.reachable, m.reachable)
        assert np.isclose(loaded.fixed_z, 1.0)

        assert loaded.query(0.4, 0.0, 0.0) is True
        assert loaded.query(0.4, 0.0, np.radians(90.0)) is False
    finally:
        if os.path.exists(path):
            os.remove(path)