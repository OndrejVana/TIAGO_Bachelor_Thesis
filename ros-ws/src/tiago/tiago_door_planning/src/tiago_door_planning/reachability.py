# -*- coding: utf-8 -*-

from __future__ import print_function, division

import os
import numpy as np

from .utils import angle_wrap

# Geometric reachability

def _world_to_robot_xy(base_xy, base_yaw, target_xy):
    """
    Transform a planar world-frame point into robot frame.
    Robot frame convention: +x forward, +y left.
    Returns (dx_world, dy_world, dx_robot, dy_robot).
    """
    dx = float(target_xy[0]) - float(base_xy[0])
    dy = float(target_xy[1]) - float(base_xy[1])

    cos_yaw = np.cos(base_yaw)
    sin_yaw = np.sin(base_yaw)

    dx_robot = cos_yaw * dx + sin_yaw * dy
    dy_robot = -sin_yaw * dx + cos_yaw * dy
    return dx, dy, dx_robot, dy_robot


def _check_horizontal_distance(horizontal_dist, cfg, debug=False):
    if horizontal_dist < cfg.reach_min:
        if debug:
            print("[GeomCheck] REJECT: distance %.3fm < reach_min %.3fm" %
                  (horizontal_dist, cfg.reach_min))
        return False
    if horizontal_dist > cfg.reach_max:
        if debug:
            print("[GeomCheck] REJECT: distance %.3fm > reach_max %.3fm" %
                  (horizontal_dist, cfg.reach_max))
        return False
    return True


def _check_front_angle(dx_robot, dy_robot, cfg, debug=False):
    angle_robot_frame = np.arctan2(dy_robot, dx_robot)
    max_angle_rad = np.radians(cfg.max_reach_angle_deg)
    if abs(angle_robot_frame) > max_angle_rad:
        if debug:
            print("[GeomCheck] REJECT: angle from front %.1f° > max %.1f°" %
                  (np.degrees(angle_robot_frame), cfg.max_reach_angle_deg))
        return False, angle_robot_frame
    return True, angle_robot_frame


def _check_ellipsoidal_reach(dx_robot, dy_robot, cfg, debug=False):
    reach_forward = cfg.reach_max
    reach_lateral = cfg.reach_max * cfg.reach_lateral_factor
    if reach_forward <= 0.0 or reach_lateral <= 0.0:
        return True
    ellipse_dist = np.sqrt(
        (dx_robot / reach_forward) ** 2 +
        (dy_robot / reach_lateral) ** 2
    )
    if ellipse_dist > 1.10:
        if debug:
            print("[GeomCheck] REJECT: ellipse_dist %.3f > 1.10 "
                  "(dx_robot=%.3f, dy_robot=%.3f)" %
                  (ellipse_dist, dx_robot, dy_robot))
        return False
    return True


def _check_elevation(dz, horizontal_dist, cfg, debug=False):
    if horizontal_dist <= 0.01:
        return True, np.arctan2(dz, horizontal_dist)
    elevation_angle = np.arctan2(dz, horizontal_dist)
    min_elev_rad = np.radians(cfg.min_elevation_deg)
    max_elev_rad = np.radians(cfg.max_elevation_deg)
    if elevation_angle < min_elev_rad or elevation_angle > max_elev_rad:
        if debug:
            print("[GeomCheck] REJECT: elevation %.1f° not in [%.1f, %.1f]°" %
                  (np.degrees(elevation_angle),
                   cfg.min_elevation_deg,
                   cfg.max_elevation_deg))
        return False, elevation_angle
    return True, elevation_angle


def check_geometric_reachability(base_xy, base_yaw, grasp_xyz, cfg, debug=False):
    """
    Fast geometric workspace check for a mobile manipulator.

    Args:
        base_xy: (x, y) robot base position in world frame
        base_yaw: robot heading (radians)
        grasp_xyz: (x, y, z) grasp target in world frame
        cfg: object with workspace parameters (reach_min, reach_max, etc.)
        debug: print rejection reasons when True

    Returns:
        bool: True if target is geometrically reachable
    """
    dx, dy, dx_robot, dy_robot = _world_to_robot_xy(base_xy, base_yaw, grasp_xyz[:2])
    dz = grasp_xyz[2]

    horizontal_dist = np.sqrt(dx ** 2 + dy ** 2)
    if not _check_horizontal_distance(horizontal_dist, cfg, debug=debug):
        return False

    ok_angle, angle_robot_frame = _check_front_angle(dx_robot, dy_robot, cfg, debug=debug)
    if not ok_angle:
        return False

    if not _check_ellipsoidal_reach(dx_robot, dy_robot, cfg, debug=debug):
        return False

    ok_elev, elevation_angle = _check_elevation(dz, horizontal_dist, cfg, debug=debug)
    if not ok_elev:
        return False

    if debug:
        print("[GeomCheck] ACCEPT: dist=%.3fm, angle=%.1f°, elev=%.1f°" %
              (horizontal_dist,
               np.degrees(angle_robot_frame),
               np.degrees(elevation_angle)))
    return True


class ReachabilityBackendBase(object):
    def __init__(self, cfg):
        self.cfg = cfg

    def is_reachable(self, base_xy, base_yaw, grasp_xyz, grasp_yaw, roll_rad=None, debug=False):
        raise NotImplementedError

    def quality_at(self, base_xy, base_yaw, grasp_xyz, grasp_yaw, roll_rad=None):
        return 1.0


class GeometricReachabilityBackend(ReachabilityBackendBase):
    def is_reachable(self, base_xy, base_yaw, grasp_xyz, grasp_yaw, roll_rad=None, debug=False):
        return check_geometric_reachability(
            base_xy, base_yaw, grasp_xyz, self.cfg, debug=debug
        )


class OfflineReachabilityMap(object):
    """
    Pre-computed robot-centric reachability map stored as an NPZ file.

    Supports two map formats:
      Binary map - 'reachable' array (uint8). Legacy format from heuristic/moveit modes.
      Quality map - 'quality' array (float32) + 'quality_threshold' scalar.
                    Generated by quality mode. Stores robustness + connectivity scores.

    query() -> bool (binary classification, works for both formats)
    query_quality() -> float (0..1 quality score; returns 0.0 or 1.0 for binary maps)
    """

    def __init__(self, x_bins, y_bins, yaw_bins_rad, reachable, fixed_z,
                 quality=None, quality_threshold=0.25, wrist_roll_rad_bins=None):
        self.x_bins = np.asarray(x_bins, dtype=float)
        self.y_bins = np.asarray(y_bins, dtype=float)
        self.yaw_bins_rad = np.asarray(yaw_bins_rad, dtype=float)
        self.roll_bins = np.asarray(
            wrist_roll_rad_bins if wrist_roll_rad_bins is not None else [0.0], dtype=float
        )
        self.fixed_z = float(fixed_z)
        self.quality_threshold = float(quality_threshold)

        reachable_arr = np.asarray(reachable)
        if reachable_arr.ndim == 3:
            reachable_arr = reachable_arr[..., np.newaxis]
        self.reachable = reachable_arr

        if quality is not None:
            quality_arr = np.asarray(quality, dtype=float)
            if quality_arr.ndim == 3:
                quality_arr = quality_arr[..., np.newaxis]
            self.quality = quality_arr
        else:
            self.quality = None

        self._validate_shapes()

    def _validate_shapes(self):
        if self.reachable.ndim != 4:
            raise ValueError("Reachability map 'reachable' must be 4D [Nx, Ny, Nyaw, Nroll].")
        expected = (len(self.x_bins), len(self.y_bins), len(self.yaw_bins_rad), len(self.roll_bins))
        if self.reachable.shape != expected:
            raise ValueError(
                "Reachability map shape mismatch: expected %s, got %s" %
                (str(expected), str(self.reachable.shape))
            )

    @staticmethod
    def load_npz(path):
        if not path:
            raise ValueError("Empty reachability_map_path.")
        if not os.path.exists(path):
            raise IOError("Reachability map file does not exist: %s" % path)

        data = np.load(path, allow_pickle=False)
        OfflineReachabilityMap._validate_npz_keys(data, path)

        fixed_z = float(data["fixed_z"]) if "fixed_z" in data else 1.0
        quality = data["quality"].astype(float) if "quality" in data else None
        quality_threshold = float(data["quality_threshold"]) if "quality_threshold" in data else 0.25

        if "wrist_roll_rad_bins" in data:
            roll_bins = list(data["wrist_roll_rad_bins"].ravel())
        else:
            gen_roll = float(data["gen_wrist_roll_rad"]) if "gen_wrist_roll_rad" in data else 0.0
            roll_bins = [gen_roll]

        return OfflineReachabilityMap(
            x_bins=data["x_bins"],
            y_bins=data["y_bins"],
            yaw_bins_rad=data["yaw_bins_rad"],
            reachable=data["reachable"],
            fixed_z=fixed_z,
            quality=quality,
            quality_threshold=quality_threshold,
            wrist_roll_rad_bins=roll_bins,
        )

    @staticmethod
    def _validate_npz_keys(data, path):
        for k in ["x_bins", "y_bins", "yaw_bins_rad", "reachable"]:
            if k not in data:
                raise KeyError("Missing key '%s' in reachability map: %s" % (k, path))

    @staticmethod
    def _nearest_index(values, q):
        return int(np.argmin(np.abs(np.asarray(values, dtype=float) - float(q))))

    @staticmethod
    def _nearest_yaw_index(yaw_bins_rad, q_yaw):
        diffs = np.abs(((np.asarray(yaw_bins_rad, dtype=float) - float(q_yaw) + np.pi)
                        % (2.0 * np.pi)) - np.pi)
        return int(np.argmin(diffs))

    def _is_inside_xy_bounds(self, x_rel, y_rel):
        return (self.x_bins[0] <= x_rel <= self.x_bins[-1] and
                self.y_bins[0] <= y_rel <= self.y_bins[-1])

    def _lookup_indices(self, x_rel, y_rel, yaw_rel):
        ix = self._nearest_index(self.x_bins, x_rel)
        iy = self._nearest_index(self.y_bins, y_rel)
        iyaw = self._nearest_yaw_index(self.yaw_bins_rad, yaw_rel)
        return ix, iy, iyaw

    def _interpolated_quality(self, x_rel, y_rel, yaw_rel, roll_rad=None):
        """Linearly interpolate quality between the two nearest yaw bins.
        Roll dimension uses nearest-neighbour lookup.
        """
        if self.quality is None:
            return None

        ix = self._nearest_index(self.x_bins, x_rel)
        iy = self._nearest_index(self.y_bins, y_rel)
        iroll = self._nearest_index(self.roll_bins, roll_rad) if roll_rad is not None else 0

        yaws = self.yaw_bins_rad
        Nyaw = len(yaws)

        diffs = ((yaws - float(yaw_rel) + np.pi) % (2.0 * np.pi)) - np.pi
        abs_diffs = np.abs(diffs)
        i0 = int(np.argmin(abs_diffs))

        if diffs[i0] >= 0.0:
            i1 = (i0 - 1) % Nyaw
        else:
            i1 = (i0 + 1) % Nyaw

        d0 = abs_diffs[i0]
        d1 = abs(((yaws[i1] - float(yaw_rel) + np.pi) % (2.0 * np.pi)) - np.pi)
        span = d0 + d1
        if span < 1e-9:
            return float(self.quality[ix, iy, i0, iroll])

        w0 = 1.0 - d0 / span
        w1 = 1.0 - w0
        return float(w0 * self.quality[ix, iy, i0, iroll] + w1 * self.quality[ix, iy, i1, iroll])

    def query(self, x_rel, y_rel, yaw_rel, roll_rad=None):
        """Return True if the cell is classified as reachable."""
        if not self._is_inside_xy_bounds(x_rel, y_rel):
            return False
        q = self._interpolated_quality(x_rel, y_rel, yaw_rel, roll_rad=roll_rad)
        if q is not None:
            return q >= self.quality_threshold
        ix, iy, iyaw = self._lookup_indices(x_rel, y_rel, yaw_rel)
        iroll = self._nearest_index(self.roll_bins, roll_rad) if roll_rad is not None else 0
        return bool(self.reachable[ix, iy, iyaw, iroll] > 0.5)

    def query_quality(self, x_rel, y_rel, yaw_rel, roll_rad=None):
        """
        Return float quality score in [0, 1].
        For binary maps (no quality array), returns 1.0 if reachable, 0.0 otherwise.
        """
        if not self._is_inside_xy_bounds(x_rel, y_rel):
            return 0.0
        q = self._interpolated_quality(x_rel, y_rel, yaw_rel, roll_rad=roll_rad)
        if q is not None:
            return q
        ix, iy, iyaw = self._lookup_indices(x_rel, y_rel, yaw_rel)
        iroll = self._nearest_index(self.roll_bins, roll_rad) if roll_rad is not None else 0
        return 1.0 if bool(self.reachable[ix, iy, iyaw, iroll] > 0.5) else 0.0


class OfflineMapReachabilityBackend(ReachabilityBackendBase):
    def __init__(self, cfg):
        ReachabilityBackendBase.__init__(self, cfg)
        self._map = OfflineReachabilityMap.load_npz(cfg.reachability_map_path)

    def _check_z_compatibility(self, grasp_xyz, debug=False):
        dz = abs(float(grasp_xyz[2]) - float(self._map.fixed_z))
        if dz > self.cfg.reachability_z_tol:
            if debug:
                print("[OfflineMap] REJECT: |z - fixed_z| = %.3f > tol %.3f" %
                      (dz, self.cfg.reachability_z_tol))
            return False
        return True

    def _relative_pose_in_robot_frame(self, base_xy, base_yaw, grasp_xyz):
        dx = float(grasp_xyz[0]) - float(base_xy[0])
        dy = float(grasp_xyz[1]) - float(base_xy[1])
        cos_yaw = np.cos(base_yaw)
        sin_yaw = np.sin(base_yaw)
        return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy

    def _relative_grasp_yaw(self, base_yaw, grasp_yaw):
        if self.cfg.use_grasp_yaw:
            return angle_wrap(float(grasp_yaw) - float(base_yaw))
        return 0.0

    def is_reachable(self, base_xy, base_yaw, grasp_xyz, grasp_yaw, roll_rad=None, debug=False):
        if not self._check_z_compatibility(grasp_xyz, debug=debug):
            return False
        x_rel, y_rel = self._relative_pose_in_robot_frame(base_xy, base_yaw, grasp_xyz)

        excl = self.cfg.reachability_y_exclusion_half_width_m
        if excl > 0.0 and abs(y_rel) < excl:
            if debug:
                print("[OfflineMap] REJECT (y-exclusion): x_rel=%.3f y_rel=%.3f" %
                      (x_rel, y_rel))
            return False
        yaw_rel = self._relative_grasp_yaw(base_yaw, grasp_yaw)
        ok = self._map.query(x_rel, y_rel, yaw_rel, roll_rad=roll_rad)
        if debug:
            print("[OfflineMap] %s: x_rel=%.3f y_rel=%.3f yaw_rel=%.1f deg z=%.3f" %
                  ("ACCEPT" if ok else "REJECT",
                   x_rel, y_rel, np.degrees(yaw_rel), grasp_xyz[2]))
        return ok

    def quality_at(self, base_xy, base_yaw, grasp_xyz, grasp_yaw, roll_rad=None):
        """
        Return float quality score [0..1] for a grasp pose from the given base.
        Returns 0.0 if z is out of tolerance or the position is outside map bounds.
        For binary maps (heuristic/moveit), returns 1.0 or 0.0.
        """
        if not self._check_z_compatibility(grasp_xyz):
            return 0.0
        x_rel, y_rel = self._relative_pose_in_robot_frame(base_xy, base_yaw, grasp_xyz)
        yaw_rel = self._relative_grasp_yaw(base_yaw, grasp_yaw)
        return self._map.query_quality(x_rel, y_rel, yaw_rel, roll_rad=roll_rad)


def make_reachability_backend(cfg):
    """Factory: build the appropriate backend from cfg.reachability_backend."""
    backend = str(cfg.reachability_backend).strip().lower()
    if backend == "geometric":
        return GeometricReachabilityBackend(cfg)
    if backend == "offline_map":
        return OfflineMapReachabilityBackend(cfg)
    raise ValueError("Unsupported reachability backend '%s'" % backend)