# -*- coding: utf-8 -*-

from __future__ import print_function, division

import math
import numpy as np

from tiago_door_planning.feasibility import FeasConfig, LambdaComputer


def make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0):
    cfg = FeasConfig(
        angle_step_deg=angle_step_deg,
        open_angle_rad=np.radians(open_angle_deg),
        robot_radius=0.30,
        reach_min=0.35,
        reach_max=0.75,
        handle_height=1.0,
    )
    return LambdaComputer(cfg)


def test_components_1d_empty():
    lc = make_lambda_computer()
    comps = lc._components_1d([False, False, False])
    assert comps == []


def test_components_1d_single_component():
    lc = make_lambda_computer()
    comps = lc._components_1d([False, True, True, True, False])
    assert comps == [[1, 2, 3]]


def test_components_1d_multiple_components():
    lc = make_lambda_computer()
    comps = lc._components_1d([True, True, False, True, False, True, True])
    assert comps == [[0, 1], [3], [5, 6]]


def test_classify_intervals_component_touches_closed_only():
    lc = make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0)
    angles_rad = np.radians(np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90], dtype=float))
    comps = [[0, 1, 2]]

    angles0, angles1 = lc._classify_intervals_from_components(comps, angles_rad)

    assert np.allclose(angles0, np.radians([0, 10, 20]))
    assert angles1 == []


def test_classify_intervals_component_touches_open_only():
    lc = make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0)
    angles_rad = np.radians(np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90], dtype=float))
    comps = [[7, 8, 9]]

    angles0, angles1 = lc._classify_intervals_from_components(comps, angles_rad)

    assert angles0 == []
    assert np.allclose(angles1, np.radians([70, 80, 90]))


def test_classify_intervals_component_touches_neither_boundary():
    lc = make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0)
    angles_rad = np.radians(np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90], dtype=float))
    comps = [[3, 4, 5]]

    angles0, angles1 = lc._classify_intervals_from_components(comps, angles_rad)

    assert angles0 == []
    assert angles1 == []


def test_classify_intervals_one_component_spans_both_boundaries():
    lc = make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0)
    angles_rad = np.radians(np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90], dtype=float))
    comps = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]]

    angles0, angles1 = lc._classify_intervals_from_components(comps, angles_rad)

    assert np.allclose(angles0, angles_rad)
    assert np.allclose(angles1, angles_rad)


def test_classify_intervals_two_boundary_components():
    lc = make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0)
    angles_rad = np.radians(np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90], dtype=float))
    comps = [[0, 1], [8, 9]]

    angles0, angles1 = lc._classify_intervals_from_components(comps, angles_rad)

    assert np.allclose(angles0, np.radians([0, 10]))
    assert np.allclose(angles1, np.radians([80, 90]))


def test_angle_sampling_includes_exact_open_endpoint_even_if_not_multiple():
    lc = make_lambda_computer(angle_step_deg=7.0, open_angle_deg=90.0)

    step = np.radians(lc.cfg.angle_step_deg)
    angles_rad = np.arange(0.0, lc.cfg.open_angle_rad + 0.5 * step, step, dtype=float)
    if len(angles_rad) == 0 or not np.isclose(angles_rad[-1], lc.cfg.open_angle_rad):
        if angles_rad[-1] > lc.cfg.open_angle_rad:
            angles_rad[-1] = lc.cfg.open_angle_rad
        else:
            angles_rad = np.append(angles_rad, lc.cfg.open_angle_rad)

    assert np.isclose(angles_rad[0], 0.0)
    assert np.isclose(angles_rad[-1], lc.cfg.open_angle_rad)


def test_classify_intervals_uses_boundary_tolerance():
    lc = make_lambda_computer(angle_step_deg=10.0, open_angle_deg=90.0)

    angles_rad = np.array([
        1e-6,
        np.radians(10),
        np.radians(20),
        np.radians(70),
        np.radians(80),
        np.radians(90) - 1e-6,
    ], dtype=float)

    comps = [[0, 1, 2], [3, 4, 5]]
    angles0, angles1 = lc._classify_intervals_from_components(comps, angles_rad)

    assert len(angles0) == 3
    assert len(angles1) == 3