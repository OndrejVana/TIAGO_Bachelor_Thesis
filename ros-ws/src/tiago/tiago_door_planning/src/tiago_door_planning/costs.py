# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np

from geometry_msgs.msg import PoseStamped

from .utils import yaw_from_quat


class CostConfig(object):
    def __init__(self,
                 occ_threshold=50,
                 unknown_cost=5.0,
                 occ_cost=1e6,
                 w_costmap=1.0,
                 w_arm=1.0,
                 arm_nominal_dist=0.55,
                 arm_sigma=0.15,
                 arm_min_dist=0.35,
                 arm_max_dist=0.85,
                 arm_hard_penalty=1e3,
                 arm_centerline_danger_m=0.0,
                 arm_centerline_penalty=0.0):
        self.occ_threshold = occ_threshold
        self.unknown_cost = unknown_cost
        self.occ_cost = occ_cost
        self.w_costmap = w_costmap
        self.w_arm = w_arm
        self.arm_nominal_dist = arm_nominal_dist
        self.arm_sigma = arm_sigma
        self.arm_min_dist = arm_min_dist
        self.arm_max_dist = arm_max_dist
        self.arm_hard_penalty = arm_hard_penalty
        self.arm_centerline_danger_m = float(arm_centerline_danger_m)
        self.arm_centerline_penalty = float(arm_centerline_penalty)


# =========================
# --- GRID / COSTMAP ---
# =========================

def _world_to_grid(occ, x, y):
    """
    Convert world coordinates to grid indices.
    """
    res = occ.info.resolution
    ox = occ.info.origin.position.x
    oy = occ.info.origin.position.y

    ix = int((x - ox) / res)
    iy = int((y - oy) / res)
    return ix, iy


def _is_inside_grid(ix, iy, width, height):
    """
    Check if grid indices are inside bounds.
    """
    return (0 <= ix < width) and (0 <= iy < height)


def grid_value_at(occ, x, y):
    """Return occupancy value at world (x,y) or None if outside grid."""
    ix, iy = _world_to_grid(occ, x, y)

    w = occ.info.width
    h = occ.info.height

    if not _is_inside_grid(ix, iy, w, h):
        return None

    return int(occ.data[iy * w + ix])


def _interpret_occ_value(v, cfg):
    """
    Convert raw occupancy value into cost.
    """
    if v is None:
        return cfg.unknown_cost
    if v < 0:
        return cfg.unknown_cost
    if v >= cfg.occ_threshold:
        return cfg.occ_cost

    return float(v) / float(max(1, cfg.occ_threshold - 1))


def cost_costmap_pose(occ, base_pose, cfg):
    """Simple per-pose cost from occupancy grid."""
    if occ is None:
        return 0.0

    x = base_pose.pose.position.x
    y = base_pose.pose.position.y

    v = grid_value_at(occ, x, y)
    return _interpret_occ_value(v, cfg)


# =========================
# --- FRAME TRANSFORMS ---
# =========================

def transform_xy_to_base(base_pose_map, x_map, y_map):
    """Express a point given in map frame into base frame (2D only)."""
    yaw = yaw_from_quat(base_pose_map.pose.orientation)

    dx = x_map - base_pose_map.pose.position.x
    dy = y_map - base_pose_map.pose.position.y

    c = np.cos(-yaw)
    s = np.sin(-yaw)
    return c * dx - s * dy, s * dx + c * dy


# =========================
# --- ARM COST ---
# =========================

def _compute_xy_in_base(base_pose_map, handle_pose_map):
    """Compute handle (x, y) in robot base frame."""
    hx = handle_pose_map.pose.position.x
    hy = handle_pose_map.pose.position.y
    return transform_xy_to_base(base_pose_map, hx, hy)


def _soft_arm_penalty(r, cfg):
    """
    Gaussian-like penalty around nominal radius.
    """
    return ((r - cfg.arm_nominal_dist) ** 2) / (2.0 * (cfg.arm_sigma ** 2) + 1e-12)


def _hard_arm_penalty(r, cfg):
    """
    Hard penalty if outside allowed radius band.
    """
    if r < cfg.arm_min_dist or r > cfg.arm_max_dist:
        return cfg.arm_hard_penalty
    return 0.0


def _arm_centerline_penalty(yb, cfg):
    """
    Quadratic penalty when the handle y in robot base frame is inside the
    arm_centerline_danger_m zone.
    """
    danger_m = cfg.arm_centerline_danger_m
    penalty = cfg.arm_centerline_penalty
    if danger_m <= 0.0 or penalty <= 0.0:
        return 0.0
    abs_y = abs(float(yb))
    if abs_y >= danger_m:
        return 0.0
    ratio = 1.0 - abs_y / danger_m
    return penalty * ratio * ratio


def arm_comfort_cost(base_pose_map, handle_pose_map, cfg):
    """
    Arm cost: penalise handle being far from nominal radius and near the
    arm centerline (y ≈ 0) where IK configuration flips occur.
    """
    xb, yb = _compute_xy_in_base(base_pose_map, handle_pose_map)
    r = np.hypot(xb, yb)

    soft = _soft_arm_penalty(r, cfg)
    hard = _hard_arm_penalty(r, cfg)
    centerline = _arm_centerline_penalty(yb, cfg)

    return soft + hard + centerline


# =========================
# --- ANGLE SELECTION ---
# =========================

def _compute_angle_costs(base_pose_map, hinge_pose_map, angles, handle_pose_from_angle_fn, cfg):
    """
    Compute cost for each candidate angle.
    """
    costs = []
    poses = []

    for a in angles:
        hp = handle_pose_from_angle_fn(hinge_pose_map, a)
        c = arm_comfort_cost(base_pose_map, hp, cfg)

        costs.append(c)
        poses.append(hp)

    return np.array(costs), poses


def pick_best_angle(base_pose_map, hinge_pose_map, handle_radius, hinge_yaw, angles, handle_pose_from_angle_fn, cfg):
    """
    Choose the angle in 'angles' that minimizes arm comfort cost.
    Returns (best_angle, best_cost, best_handle_pose).
    """
    if not angles:
        return None, np.inf, None

    costs_arr, poses = _compute_angle_costs(
        base_pose_map, hinge_pose_map, angles, handle_pose_from_angle_fn, cfg
    )

    best_idx = int(np.argmin(costs_arr))

    return angles[best_idx], float(costs_arr[best_idx]), poses[best_idx]


# =========================
# --- TRANSITION COST ---
# =========================

def _compute_costmap_costs(occ, base_pose_samples, cfg):
    """
    Compute costmap costs for all samples.
    """
    return [cost_costmap_pose(occ, bp, cfg) for bp in base_pose_samples]


def _compute_arm_costs(base_pose_samples, lambda_angles_per_pose, hinge_pose_map, handle_pose_from_angle_fn, cfg):
    """
    Compute minimal arm cost per pose.
    """
    arm_costs = []

    for bp, angles in zip(base_pose_samples, lambda_angles_per_pose):
        if not angles:
            return None

        angle_costs = np.array([
            arm_comfort_cost(
                bp,
                handle_pose_from_angle_fn(hinge_pose_map, a),
                cfg
            )
            for a in angles
        ])

        arm_costs.append(float(np.min(angle_costs)))

    return arm_costs


def _aggregate_transition_costs(costmap_costs, arm_costs, cfg):
    """
    Combine costmap and arm costs into final transition cost.
    """
    worst_arm = float(np.max(arm_costs)) if arm_costs else 0.0
    sum_costmap = float(np.sum(costmap_costs))

    return cfg.w_costmap * sum_costmap + cfg.w_arm * worst_arm


def transition_cost(occ, base_pose_samples, lambda_angles_per_pose, handle_pose_from_angle_fn, hinge_pose_map, cfg):
    """
    For each pose along the action:
        - Take min over feasible angles of arm_cost.
        - Then take max over poses (worst point along the action)
        - Add costmap costs along the samples.
    """
    if len(base_pose_samples) != len(lambda_angles_per_pose):
        raise ValueError("base_pose_samples and lambda list lengths differ")

    costmap_costs = _compute_costmap_costs(occ, base_pose_samples, cfg)

    arm_costs = _compute_arm_costs(
        base_pose_samples,
        lambda_angles_per_pose,
        hinge_pose_map,
        handle_pose_from_angle_fn,
        cfg
    )

    if arm_costs is None:
        return np.inf

    return _aggregate_transition_costs(costmap_costs, arm_costs, cfg)