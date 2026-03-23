# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np

from .costs import pick_best_angle


def lambda_intersection_nonempty(La, Lb, tol):
    """
    Return True if the two angle sets have at least one pair within tol.
    """
    if not La or not Lb:
        return False
    if tol <= 0.0:
        return bool(set(La) & set(Lb))
    a_arr = np.array(La)
    b_arr = np.array(Lb)
    return bool(np.any(np.abs(a_arr[:, np.newaxis] - b_arr) <= tol))


def intersect_angle_sets(La, Lb, tol):
    """
    Return angles from La that have a match in Lb within tol.
    """
    if not La or not Lb:
        return []
    if tol <= 0.0:
        return sorted(list(set(La) & set(Lb)))

    b_arr = np.asarray(Lb, dtype=float)
    return [
        float(a) for a in La
        if np.any(np.abs(b_arr - float(a)) <= tol)
    ]


def propagate_interval_feasibility(angle_sets_per_pose, tol=0.02):
    """
    Propagate a running feasible angle set along a motion primitive.

    Returns:
        (feasible, surviving_angles, fail_index)
        - feasible: True if at least one angle survives all poses
        - surviving_angles: angles that survive (empty on failure)
        - fail_index: index where propagation failed (None on success)
    """
    if not angle_sets_per_pose:
        return False, [], 0

    running = list(angle_sets_per_pose[0])
    if not running:
        return False, [], 0

    for i in range(1, len(angle_sets_per_pose)):
        running = intersect_angle_sets(running, angle_sets_per_pose[i], tol=tol)
        if not running:
            return False, [], i

    return True, running, None


def interval_angles(lam, d):
    """Return the feasible angle list for interval d from a LambdaResult."""
    return lam.angles0 if int(d) == 0 else lam.angles1


def can_switch_intervals(lam, tol=1e-6):
    """
    True if intervals 0 and 1 overlap at this pose — meaning a zero-cost
    interval switch is valid here.
    """
    return lambda_intersection_nonempty(lam.angles0, lam.angles1, tol=tol)


def pick_best_monotonic_angle(base_pose_map, hinge_pose_map, handle_radius,
                              hinge_yaw, angles, handle_pose_from_angle_fn,
                              cfg, prev_angle, monotonic_tol=1e-6):
    """
    Choose the best feasible angle from 'angles' subject to monotonic opening.

    Returns:
        (best_angle, best_cost, filtered_angles)
        - best_angle: None if no angle satisfies the monotonicity constraint
    """
    if not angles:
        return None, None, []

    filtered = [
        float(a) for a in angles
        if float(a) >= float(prev_angle) - float(monotonic_tol)
    ]
    if not filtered:
        return None, None, []

    best_a, best_cost, best_pose = pick_best_angle(
        base_pose_map=base_pose_map,
        hinge_pose_map=hinge_pose_map,
        handle_radius=handle_radius,
        hinge_yaw=hinge_yaw,
        angles=filtered,
        handle_pose_from_angle_fn=handle_pose_from_angle_fn,
        cfg=cfg,
    )
    return best_a, best_cost, filtered