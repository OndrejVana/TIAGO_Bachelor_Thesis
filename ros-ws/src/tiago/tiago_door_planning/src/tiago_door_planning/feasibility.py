# -*- coding: utf-8 -*-
from __future__ import print_function, division

import numpy as np
import time as time_module
import rospy

from .door_collision import (
    DoorGeom,
    door_polygon,
    circle_intersects_poly,
    point_in_poly,
    poly_aabb,
    extract_occ_info,
)
from .utils import angle_wrap
from .reachability import (
    OfflineReachabilityMap,
    make_reachability_backend,
)

class FeasConfig(object):
    def __init__(self,
                 angle_step_deg,
                 open_angle_rad,
                 robot_radius,
                 reach_min,
                 reach_max,
                 handle_height=1.0,
                 reach_lateral_factor=0.7,
                 max_reach_angle_deg=100.0,
                 min_elevation_deg=-30.0,
                 max_elevation_deg=60.0,
                 reachability_backend="geometric",
                 reachability_map_path="",
                 reachability_fixed_z=1.0,
                 reachability_z_tol=0.15,
                 reachability_y_exclusion_half_width_m=0.0,
                 use_grasp_yaw=True,
                 grasp_yaw_offset_rad=0.0,
                 reachability_wrist_roll_rad=None):
        self.angle_step_deg = angle_step_deg
        self.open_angle_rad = open_angle_rad
        self.robot_radius = robot_radius

        self.reach_min = reach_min
        self.reach_max = reach_max

        self.handle_height = handle_height

        self.reach_lateral_factor = reach_lateral_factor
        self.max_reach_angle_deg = max_reach_angle_deg
        self.min_elevation_deg = min_elevation_deg
        self.max_elevation_deg = max_elevation_deg

        self.reachability_backend = str(reachability_backend).strip().lower()
        self.reachability_map_path = str(reachability_map_path)
        self.reachability_fixed_z = float(reachability_fixed_z)
        self.reachability_z_tol = float(reachability_z_tol)
        self.reachability_y_exclusion_half_width_m = float(reachability_y_exclusion_half_width_m)
        self.use_grasp_yaw = bool(use_grasp_yaw)
        self.grasp_yaw_offset_rad = float(grasp_yaw_offset_rad)
        self.reachability_wrist_roll_rad = (
            float(reachability_wrist_roll_rad) if reachability_wrist_roll_rad is not None else None
        )


class LambdaResult(object):
    """
    Result of feasible door-angle reconstruction for one base state.
    """
    def __init__(self, angles0, angles1, feasible_mask,
                 sampled_angles_rad=None, components=None, quality_by_angle=None):
        self.angles0 = angles0
        self.angles1 = angles1
        self.feasible_mask = feasible_mask
        self.sampled_angles_rad = sampled_angles_rad if sampled_angles_rad is not None else []
        self.components = components if components is not None else []
        self.quality_by_angle = quality_by_angle if quality_by_angle is not None else {}

class LambdaComputer(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self._cache = {}
        self._reachability = make_reachability_backend(cfg)

        self._stats = {
            "compute_calls": 0,
            "cache_hits": 0,
            "cache_misses": 0,
            "compute_time_total": 0.0,
        }

    def reset_stats(self):
        self._stats = {
            "compute_calls": 0,
            "cache_hits": 0,
            "cache_misses": 0,
            "compute_time_total": 0.0,
        }

    def get_stats(self):
        out = dict(self._stats)
        calls = max(1, int(out["compute_calls"]))
        out["cache_hit_rate"] = float(out["cache_hits"]) / float(calls)
        return out

    def _make_cache_key(self, base_xy, base_yaw, hinge_xy, hinge_yaw,
                        handle_radius, opening_sign, grasp_yaw_extra_offset):
        """
        Build cache key from quantized geometric inputs.
        """
        return (
            int(round(base_xy[0] * 100.0)),
            int(round(base_xy[1] * 100.0)),
            int(round(base_yaw * 100.0)),
            int(round(hinge_xy[0] * 100.0)),
            int(round(hinge_xy[1] * 100.0)),
            int(round(hinge_yaw * 100.0)),
            int(round(handle_radius * 100.0)),
            int(round(float(opening_sign) * 10.0)),
            int(round(float(grasp_yaw_extra_offset) * 100.0)),
        )

    def _sample_angles_rad(self):
        """
        Sample door opening angles from 0 to fully open.
        """
        step = np.radians(self.cfg.angle_step_deg)
        angles_rad = np.arange(
            0.0,
            self.cfg.open_angle_rad + 0.5 * step,
            step,
            dtype=float
        )

        if len(angles_rad) == 0:
            return np.array([self.cfg.open_angle_rad], dtype=float)

        if not np.isclose(angles_rad[-1], self.cfg.open_angle_rad):
            if angles_rad[-1] > self.cfg.open_angle_rad:
                angles_rad[-1] = self.cfg.open_angle_rad
            else:
                angles_rad = np.append(angles_rad, self.cfg.open_angle_rad)

        return angles_rad

    def _compute_handle_positions(self, hinge_xy, hinge_yaw, handle_radius,
                                  angles_rad, opening_sign):
        """
        Compute handle positions and corresponding door yaws for all sampled angles.
        """
        hinge = np.array(hinge_xy, dtype=float)
        door_yaws = hinge_yaw + float(opening_sign) * angles_rad
        handle_positions = hinge + handle_radius * np.column_stack(
            [np.cos(door_yaws), np.sin(door_yaws)]
        )
        return door_yaws, handle_positions

    def _compute_reachable_mask(self, base_xy, base_yaw, door_yaws, handle_positions,
                                grasp_yaw_extra_offset=0.0):
        """
        Evaluate reachability backend for all sampled handle poses.
        grasp_yaw_extra_offset is added to grasp_yaw only (not handle positions).
        Pass np.pi for pull doors so the robot faces the handle from the correct side.
        Returns (reachable, quality_by_angle) where quality_by_angle maps angle to score.
        """
        n = len(door_yaws)
        reachable = np.zeros(n, dtype=bool)
        quality_by_angle = {}
        roll_rad = getattr(self.cfg, 'reachability_wrist_roll_rad', None)

        for i in range(n):
            handle_xyz = [
                float(handle_positions[i, 0]),
                float(handle_positions[i, 1]),
                float(self.cfg.handle_height),
            ]

            grasp_yaw = angle_wrap(
                float(door_yaws[i]) + float(self.cfg.grasp_yaw_offset_rad)
                + float(grasp_yaw_extra_offset)
            )

            debug_this = True
            reachable[i] = self._reachability.is_reachable(
                base_xy=base_xy,
                base_yaw=base_yaw,
                grasp_xyz=handle_xyz,
                grasp_yaw=grasp_yaw,
                roll_rad=roll_rad,
                debug=debug_this,
            )

            if reachable[i] and hasattr(self._reachability, 'quality_at'):
                quality_by_angle[float(door_yaws[i])] = self._reachability.quality_at(
                    base_xy=base_xy,
                    base_yaw=base_yaw,
                    grasp_xyz=handle_xyz,
                    grasp_yaw=grasp_yaw,
                    roll_rad=roll_rad,
                )

        return reachable, quality_by_angle

    def _compute_feasible_mask(self, base_xy, hinge_xy, hinge_yaw, angles_rad,
                               door_geom, occ_grid, occ_thresh, reachable,
                               opening_sign):
        """
        Filter sampled angles using door/base and door/occupancy collision checks.
        """
        n = len(angles_rad)
        feasible = np.zeros(n, dtype=bool)
        base = np.array(base_xy, dtype=float)

        for i in range(n):
            if not reachable[i]:
                continue

            ang = angles_rad[i]
            poly = door_polygon(
                hinge_xy, hinge_yaw, ang, door_geom, opening_sign=opening_sign
            )

            if circle_intersects_poly(base[0], base[1], self.cfg.robot_radius, poly):
                continue

            if occ_grid is not None and self._door_hits_occupancy(poly, occ_grid, occ_thresh):
                continue

            feasible[i] = True

        return feasible

    def compute(self, base_xy, base_yaw, hinge_xy, hinge_yaw, handle_radius,
                door_geom, occ_grid, occ_thresh, opening_sign=1.0,
                grasp_yaw_extra_offset=0.0):
        """
        Returns LambdaResult containing interval-connected feasible angle sets.
        grasp_yaw_extra_offset is added to grasp_yaw only, not handle positions.
        Pass np.pi for pull doors to flip the approach direction.
        """
        t0 = time_module.time()
        self._stats["compute_calls"] += 1

        k = self._make_cache_key(
            base_xy, base_yaw, hinge_xy, hinge_yaw, handle_radius, opening_sign,
            grasp_yaw_extra_offset
        )

        if k in self._cache:
            self._stats["cache_hits"] += 1
            self._stats["compute_time_total"] += (time_module.time() - t0)
            return self._cache[k]

        angles_rad = self._sample_angles_rad()
        door_yaws, handle_positions = self._compute_handle_positions(
            hinge_xy, hinge_yaw, handle_radius, angles_rad, opening_sign
        )

        reachable, quality_by_angle = self._compute_reachable_mask(
            base_xy, base_yaw, door_yaws, handle_positions,
            grasp_yaw_extra_offset=grasp_yaw_extra_offset
        )

        feasible = self._compute_feasible_mask(
            base_xy, hinge_xy, hinge_yaw, angles_rad,
            door_geom, occ_grid, occ_thresh, reachable, opening_sign
        )

        feasible_list = feasible.tolist()
        components = self._components_1d(feasible_list)
        angles0, angles1 = self._classify_intervals_from_components(components, angles_rad)

        if not angles0:
            n_total = len(angles_rad)
            n_reach = int(np.sum(reachable))
            n_feas  = int(np.sum(feasible))
            rospy.logwarn(
                "[LambdaComputer] angles0 empty. "
                "base=(%.2f, %.2f, %.1f deg) "
                "reachable=%d/%d  feasible=%d/%d  components=%d",
                float(base_xy[0]), float(base_xy[1]), float(np.degrees(base_yaw)),
                n_reach, n_total, n_feas, n_total, len(components),
            )

        result = LambdaResult(
            angles0=angles0,
            angles1=angles1,
            feasible_mask=feasible_list,
            sampled_angles_rad=angles_rad.tolist(),
            components=[list(c) for c in components],
            quality_by_angle=quality_by_angle,
        )

        self._cache[k] = result
        self._stats["cache_misses"] += 1
        self._stats["compute_time_total"] += (time_module.time() - t0)
        return result

    def _components_1d(self, feasible):
        """
        Extract connected components of a 1D boolean feasibility mask.
        """
        comps = []
        current_comp = []

        for i, ok in enumerate(feasible):
            if ok:
                current_comp.append(i)
            else:
                if current_comp:
                    comps.append(current_comp)
                    current_comp = []

        if current_comp:
            comps.append(current_comp)

        return comps

    def _classify_intervals_from_components(self, comps, angles_rad):
        """
        Classify connected feasible components into interval 0 and interval 1.
        """
        if not comps:
            return [], []

        tol = 0.5 * np.radians(self.cfg.angle_step_deg)
        closed_ref = 0.0
        open_ref = self.cfg.open_angle_rad

        idx0 = None
        idx1 = None

        for k, comp in enumerate(comps):
            comp_angles = angles_rad[np.asarray(comp, dtype=int)]
            comp_min = float(comp_angles[0])
            comp_max = float(comp_angles[-1])

            if comp_min <= closed_ref + tol:
                idx0 = k

            if comp_max >= open_ref - tol:
                idx1 = k

        if idx0 is None and comps:
            idx0 = 0

        if idx1 is None and comps:
            idx1 = max(range(len(comps)), key=lambda k: int(comps[k][-1]))

        angles0 = angles_rad[np.asarray(comps[idx0], dtype=int)].tolist() if idx0 is not None else []
        angles1 = angles_rad[np.asarray(comps[idx1], dtype=int)].tolist() if idx1 is not None else []

        return angles0, angles1

    def _polygon_grid_window(self, poly, occ):
        """
        Compute clamped grid search window for a polygon AABB.
        """
        res, ox, oy, w, h = extract_occ_info(occ)

        mnx, mny, mxx, mxy = poly_aabb(poly)
        ix0 = int((mnx - ox) / res) - 1
        iy0 = int((mny - oy) / res) - 1
        ix1 = int((mxx - ox) / res) + 1
        iy1 = int((mxy - oy) / res) + 1

        ix0 = max(0, min(w - 1, ix0))
        ix1 = max(0, min(w - 1, ix1))
        iy0 = max(0, min(h - 1, iy0))
        iy1 = max(0, min(h - 1, iy1))

        return ix0, iy0, ix1, iy1, res, ox, oy, w, h

    def _occupied_cell_centers_in_polygon_window(self, poly, occ, occ_thresh):
        """
        Return occupied cell centers inside polygon AABB window.
        """
        ix0, iy0, ix1, iy1, res, ox, oy, w, h = self._polygon_grid_window(poly, occ)
        data = np.array(occ.data, dtype=np.int8)

        iy_range = np.arange(iy0, iy1 + 1)
        ix_range = np.arange(ix0, ix1 + 1)
        ix_grid, iy_grid = np.meshgrid(ix_range, iy_range)

        ix_flat = ix_grid.ravel()
        iy_flat = iy_grid.ravel()

        indices = iy_flat * w + ix_flat
        occupied_mask = data[indices] >= occ_thresh

        if not np.any(occupied_mask):
            return None, None

        wx = ox + (ix_flat[occupied_mask] + 0.5) * res
        wy = oy + (iy_flat[occupied_mask] + 0.5) * res
        return wx, wy

    def _door_hits_occupancy(self, poly, occ, occ_thresh):
        """Check if door polygon intersects occupied grid cells."""
        wx, wy = self._occupied_cell_centers_in_polygon_window(poly, occ, occ_thresh)

        if wx is None:
            return False

        for i in range(len(wx)):
            if point_in_poly((wx[i], wy[i]), poly):
                return True

        return False