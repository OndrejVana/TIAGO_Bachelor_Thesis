# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np


class DiscState(object):
    __slots__ = ('ix', 'iy', 'itheta', 'd')

    def __init__(self, ix, iy, itheta, d):
        self.ix = ix
        self.iy = iy
        self.itheta = itheta
        self.d = d  # 0 or 1

    def __eq__(self, other):
        if not isinstance(other, DiscState):
            return False
        return (self.ix, self.iy, self.itheta, self.d) == (
            other.ix, other.iy, other.itheta, other.d
        )

    def __hash__(self):
        return hash((self.ix, self.iy, self.itheta, self.d))

    def __repr__(self):
        return 'DiscState(%d,%d,%d,%d)' % (self.ix, self.iy, self.itheta, self.d)


class Pose2D(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


class LatticeConfig(object):
    def __init__(self, xy_res, theta_bins, step_m, arc_radius_m, allow_reverse):
        self.xy_res = xy_res
        self.theta_bins = theta_bins
        self.step_m = step_m
        self.arc_radius_m = arc_radius_m
        self.allow_reverse = allow_reverse


# ============================================================
# Basic conversions
# ============================================================

def wrap_theta_bin(it, bins):
    return it % bins


def bin_to_yaw(it, bins):
    return (2.0 * np.pi) * (float(it) / float(bins))


def state_to_pose(s, cfg):
    return Pose2D(
        x=s.ix * cfg.xy_res,
        y=s.iy * cfg.xy_res,
        yaw=bin_to_yaw(s.itheta, cfg.theta_bins),
    )


def pose_to_state(p, d, cfg):
    ix = int(round(p.x / cfg.xy_res))
    iy = int(round(p.y / cfg.xy_res))
    it = int(round((p.yaw % (2 * np.pi)) / (2 * np.pi) * cfg.theta_bins)) % cfg.theta_bins
    return DiscState(ix, iy, it, d)


# ============================================================
# Internal helpers for primitive generation
# ============================================================

def _sample_parameter(n):
    """
    Sample normalized parameter from 0 to 1 inclusive.
    """
    return np.linspace(0.0, 1.0, n + 1)


def _poses_from_arrays(x_vals, y_vals, yaw_vals):
    """
    Build list of Pose2D samples from NumPy arrays.
    """
    return [
        Pose2D(x=float(x), y=float(y), yaw=float(yaw))
        for x, y, yaw in zip(x_vals, y_vals, yaw_vals)
    ]


def _straight_samples(p0, cfg, sign, t):
    """
    Sample straight motion primitive.
    """
    ds = sign * cfg.step_m * t
    x_vals = p0.x + ds * np.cos(p0.yaw)
    y_vals = p0.y + ds * np.sin(p0.yaw)
    yaw_vals = np.full_like(t, p0.yaw)
    return _poses_from_arrays(x_vals, y_vals, yaw_vals)


def _arc_center(p0, radius, sign):
    """
    Compute center of left/right turning circle.
    """
    cx = p0.x - sign * radius * np.sin(p0.yaw)
    cy = p0.y + sign * radius * np.cos(p0.yaw)
    return cx, cy


def _arc_samples(p0, cfg, sign, t):
    """
    Sample forward circular arc primitive.
    """
    radius = cfg.arc_radius_m
    dphi = cfg.step_m / radius
    arc_angles = dphi * t

    yaw_vals = p0.yaw + sign * arc_angles
    cx, cy = _arc_center(p0, radius, sign)

    x_vals = cx + sign * radius * np.sin(yaw_vals)
    y_vals = cy - sign * radius * np.cos(yaw_vals)

    return _poses_from_arrays(x_vals, y_vals, yaw_vals)


def _rev_arc_samples(p0, cfg, sign, t):
    """
    Sample reverse circular arc primitive (backing up while turning).

    The turning center is on the same side as for the forward arc with the
    same sign, but the robot moves backward along the circle — yaw decreases.
    This is the natural backing-up-while-turning motion needed for pull.
    """
    radius = cfg.arc_radius_m
    dphi = cfg.step_m / radius
    arc_angles = dphi * t

    yaw_vals = p0.yaw - sign * arc_angles
    cx, cy = _arc_center(p0, radius, sign)

    x_vals = cx + sign * radius * np.sin(yaw_vals)
    y_vals = cy - sign * radius * np.cos(yaw_vals)

    return _poses_from_arrays(x_vals, y_vals, yaw_vals)


def _rotation_samples(p0, cfg, sign, t):
    """
    Sample in-place rotation primitive.
    """
    dtheta = 2.0 * np.pi / cfg.theta_bins

    yaw_vals = p0.yaw + sign * dtheta * t
    x_vals = np.full_like(t, p0.x)
    y_vals = np.full_like(t, p0.y)

    return _poses_from_arrays(x_vals, y_vals, yaw_vals)


def _primitive_sign(kind):
    """
    Return turning / direction sign for a primitive kind.
    """
    if kind in ("fwd", "arcL", "rotL", "arcRevL"):
        return 1.0
    if kind in ("rev", "arcR", "rotR", "arcRevR"):
        return -1.0
    raise ValueError("Unknown primitive kind: %s" % kind)


def _is_straight_kind(kind):
    return kind in ("fwd", "rev")


def _is_arc_kind(kind):
    return kind in ("arcL", "arcR")


def _is_rev_arc_kind(kind):
    return kind in ("arcRevL", "arcRevR")


def _is_rotation_kind(kind):
    return kind in ("rotL", "rotR")


# ============================================================
# Public primitive API
# ============================================================

def primitive_samples(p0, kind, cfg, n=10):
    """
    kind:
      - "fwd" : straight forward
      - "rev" : straight reverse
      - "arcL" : forward left arc
      - "arcR" : forward right arc
      - "arcRevL" : reverse left arc  (back up while turning left)
      - "arcRevR" : reverse right arc (back up while turning right)
      - "rotL" : in-place left rotation
      - "rotR" : in-place right rotation
    """
    t = _sample_parameter(n)

    if _is_straight_kind(kind):
        sign = 1.0 if kind == "fwd" else -1.0
        return _straight_samples(p0, cfg, sign, t)

    if _is_arc_kind(kind):
        sign = 1.0 if kind == "arcL" else -1.0
        return _arc_samples(p0, cfg, sign, t)

    if _is_rev_arc_kind(kind):
        sign = 1.0 if kind == "arcRevL" else -1.0
        return _rev_arc_samples(p0, cfg, sign, t)

    if _is_rotation_kind(kind):
        sign = 1.0 if kind == "rotL" else -1.0
        return _rotation_samples(p0, cfg, sign, t)

    raise ValueError("Unknown primitive kind: %s" % kind)


def primitive_set(cfg):
    kinds = ["fwd", "arcL", "arcR", "rotL", "rotR"]
    if cfg.allow_reverse:
        kinds += ["rev", "arcRevL", "arcRevR"]
    return kinds