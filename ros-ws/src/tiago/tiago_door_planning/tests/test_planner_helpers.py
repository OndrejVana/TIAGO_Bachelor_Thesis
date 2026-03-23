# -*- coding: utf-8 -*-

from __future__ import print_function, division

import math
import pytest

from tiago_door_planning.planner_core import (
    lambda_intersection_nonempty,
    intersect_angle_sets,
    propagate_interval_feasibility,
)


def test_lambda_intersection_nonempty_exact_true():
    a = [0.0, 0.1, 0.2]
    b = [0.3, 0.2, 0.4]
    assert lambda_intersection_nonempty(a, b, tol=0.0) is True


def test_lambda_intersection_nonempty_exact_false():
    a = [0.0, 0.1, 0.2]
    b = [0.3, 0.4, 0.5]
    assert lambda_intersection_nonempty(a, b, tol=0.0) is False


def test_lambda_intersection_nonempty_tolerant_true():
    a = [0.0, 0.1, 0.2]
    b = [0.300, 0.401, 0.205]
    assert lambda_intersection_nonempty(a, b, tol=0.01) is True


def test_lambda_intersection_nonempty_empty_false():
    assert lambda_intersection_nonempty([], [0.1], tol=0.01) is False
    assert lambda_intersection_nonempty([0.1], [], tol=0.01) is False


def test_intersect_angle_sets_exact():
    a = [0.0, 0.1, 0.2, 0.3]
    b = [0.2, 0.3, 0.4]
    out = intersect_angle_sets(a, b, tol=0.0)
    assert out == [0.2, 0.3]


def test_intersect_angle_sets_tolerant():
    a = [0.0, 0.1, 0.2, 0.3]
    b = [0.201, 0.299]
    out = intersect_angle_sets(a, b, tol=0.01)
    assert out == [0.2, 0.3]


def test_intersect_angle_sets_empty():
    assert intersect_angle_sets([], [0.1], tol=0.01) == []
    assert intersect_angle_sets([0.1], [], tol=0.01) == []


def test_propagate_interval_feasibility_simple_success():
    angle_sets = [
        [0.0, 0.1, 0.2],
        [0.1, 0.2, 0.3],
        [0.2, 0.3, 0.4],
    ]
    ok, running, fail_idx = propagate_interval_feasibility(angle_sets, tol=0.0)

    assert ok is True
    assert running == [0.2]
    assert fail_idx is None


def test_propagate_interval_feasibility_pairwise_but_not_global():
    # Pairwise overlap exists:
    # [0.10, 0.20] ∩ [0.20, 0.30] = [0.20]
    # [0.20, 0.30] ∩ [0.30, 0.40] = [0.30]
    # But globally nothing survives through all three.
    angle_sets = [
        [0.10, 0.20],
        [0.20, 0.30],
        [0.30, 0.40],
    ]
    ok, running, fail_idx = propagate_interval_feasibility(angle_sets, tol=0.0)

    assert ok is False
    assert running == []
    assert fail_idx == 2


def test_propagate_interval_feasibility_tolerant_success():
    angle_sets = [
        [0.100, 0.200],
        [0.205, 0.300],
        [0.209, 0.350],
    ]
    ok, running, fail_idx = propagate_interval_feasibility(angle_sets, tol=0.01)

    assert ok is True
    assert len(running) >= 1
    assert fail_idx is None


def test_propagate_interval_feasibility_empty_input():
    ok, running, fail_idx = propagate_interval_feasibility([], tol=0.01)
    assert ok is False
    assert running == []
    assert fail_idx == 0


def test_propagate_interval_feasibility_empty_first_set():
    ok, running, fail_idx = propagate_interval_feasibility([[], [0.1]], tol=0.01)
    assert ok is False
    assert running == []
    assert fail_idx == 0