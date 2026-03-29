# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import time as time_module

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid

from .lattice import (
    DiscState,
    Pose2D,
    LatticeConfig,
    primitive_set,
    primitive_samples,
    pose_to_state,
    state_to_pose,
)
from .feasibility import LambdaComputer, FeasConfig
from .door_collision import DoorGeom, circle_hits_occupancy
from .search_core import eps_schedule_search, weighted_astar, SearchResult
from .costs import CostConfig, transition_cost, pick_best_angle
from .door_model import DoorModel
from .utils import pose_stamped, yaw_from_quat
from .intervals import (
    lambda_intersection_nonempty,
    intersect_angle_sets,
    propagate_interval_feasibility,
    interval_angles,
    can_switch_intervals,
    pick_best_monotonic_angle,
)
from .planner_logs import (
    log_start_state,
    log_geometry_summary,
    log_handle_distances,
    log_state_successors,
    log_search_summary,
)
from .planner_config import (
    _SuccessorTracking, PlannerConfig, PlanOutput,
)


class PlannerCore(object):
    def __init__(self, cfg, door_model):
        self.cfg = cfg
        self.door_model = door_model

        self._occ = None
        self._occ_thresh = cfg.occ_threshold

        self._lat_cfg = LatticeConfig(
            xy_res=cfg.xy_res,
            theta_bins=cfg.theta_bins,
            step_m=cfg.step_m,
            arc_radius_m=cfg.arc_radius_m,
            allow_reverse=cfg.allow_reverse,
        )

        self._door_geom = DoorGeom(
            width_m=float(getattr(door_model, "door_width", 0.90)),
            thickness_m=cfg.door_thickness_m,
            open_angle_rad=cfg.door_open_angle_rad,
        )

        self._lambda = LambdaComputer(
            FeasConfig(
                angle_step_deg=cfg.door_angle_step_deg,
                open_angle_rad=cfg.door_open_angle_rad,
                robot_radius=cfg.robot_radius,
                reach_min=cfg.reach_min,
                reach_max=cfg.reach_max,
                handle_height=cfg.handle_height,
                reach_lateral_factor=cfg.reach_lateral_factor,
                max_reach_angle_deg=cfg.max_reach_angle_deg,
                min_elevation_deg=cfg.min_elevation_deg,
                max_elevation_deg=cfg.max_elevation_deg,
                reachability_backend=cfg.reachability_backend,
                reachability_map_path=cfg.reachability_map_path,
                reachability_fixed_z=cfg.reachability_fixed_z,
                reachability_z_tol=cfg.reachability_z_tol,
                reachability_y_exclusion_half_width_m=cfg.reachability_y_exclusion_half_width_m,
                use_grasp_yaw=cfg.use_grasp_yaw,
                grasp_yaw_offset_rad=cfg.grasp_yaw_offset_rad,
            )
        )

    def set_occupancy(self, occ, occ_threshold):
        self._occ = occ
        self._occ_thresh = int(occ_threshold)
        self.cfg.cost.occ_threshold = self._occ_thresh
        self._lambda._cache.clear()

    def _primitive_hits_occupancy(self, samples):
        """
        Hard validity check: reject primitive if any sampled base pose
        intersects occupied space.
        """
        if self._occ is None:
            return False

        for sp in samples:
            if circle_hits_occupancy(
                cx=sp.x,
                cy=sp.y,
                radius=self.cfg.robot_radius,
                occ=self._occ,
                occ_thresh=self._occ_thresh,
            ):
                return True
        return False

    @staticmethod
    def opening_sign(push_motion, hinge_side):
        hinge = str(hinge_side).strip().lower()
        if hinge not in ("left", "right"):
            raise ValueError(
                "Unsupported hinge_side '%s' (expected 'left' or 'right')" % hinge_side
            )
        if bool(push_motion):
            return 1.0 if hinge == "left" else -1.0
        return 1.0 if hinge == "right" else -1.0

    def _extract_hinge_geometry(self, hinge_pose_map, handle_pose_map, push_motion, hinge_side):
        hx = hinge_pose_map.pose.position.x
        hy = hinge_pose_map.pose.position.y
        hinge_yaw = yaw_from_quat(hinge_pose_map.pose.orientation)
        opening_sign = self.opening_sign(push_motion=push_motion, hinge_side=hinge_side)

        handle_radius = np.hypot(
            handle_pose_map.pose.position.x - hx,
            handle_pose_map.pose.position.y - hy
        )

        return hx, hy, hinge_yaw, opening_sign, handle_radius

    def _base_start_to_pose2d(self, base_start):
        return Pose2D(
            x=base_start.pose.position.x,
            y=base_start.pose.position.y,
            yaw=yaw_from_quat(base_start.pose.orientation),
        )

    def _make_lambda_for_pose_xyyaw(self, hx, hy, hinge_yaw, handle_radius, opening_sign,
                                    push_motion=True):
        # Pull doors: the robot must approach from the opposite side (grasp direction flipped).
        # Add pi to grasp_yaw only — handle positions are still computed from the true hinge yaw.
        grasp_yaw_extra_offset = 0.0 if bool(push_motion) else np.pi

        lambda_state_cache = {}

        def lambda_for_pose_xyyaw(x, y, yaw):
            key = (
                int(round(float(x) / self.cfg.xy_res)),
                int(round(float(y) / self.cfg.xy_res)),
                int(round(float(yaw) * 1000.0)),
                int(round(float(hx) * 1000.0)),
                int(round(float(hy) * 1000.0)),
                int(round(float(hinge_yaw) * 1000.0)),
                int(round(float(handle_radius) * 1000.0)),
                int(round(float(opening_sign) * 10.0)),
                int(round(float(grasp_yaw_extra_offset) * 100.0)),
            )
            if key in lambda_state_cache:
                return lambda_state_cache[key]

            lam = self._lambda.compute(
                base_xy=(x, y),
                base_yaw=yaw,
                hinge_xy=(hx, hy),
                hinge_yaw=hinge_yaw,
                handle_radius=handle_radius,
                door_geom=self._door_geom,
                occ_grid=self._occ,
                occ_thresh=self._occ_thresh,
                opening_sign=opening_sign,
                grasp_yaw_extra_offset=grasp_yaw_extra_offset,
            )
            lambda_state_cache[key] = lam
            return lam

        return lambda_for_pose_xyyaw, lambda_state_cache

    def _determine_start_state(self, start_p, lambda_for_pose_xyyaw):
        start_lam = lambda_for_pose_xyyaw(start_p.x, start_p.y, start_p.yaw)

        if start_lam.angles0:
            start_d = 0
        else:
            rospy.logerr(
                "[Planner] Start pose has no feasible interval-0 angles. Cannot start closed-door opening plan."
            )
            raise RuntimeError("Start pose has no feasible interval-0 angles.")

        start_s = pose_to_state(start_p, d=start_d, cfg=self._lat_cfg)
        return start_lam, start_d, start_s

    def _compute_goal_settings(self, goal_open_angle_rad):
        goal_angle = (
            float(goal_open_angle_rad)
            if goal_open_angle_rad > 1e-6
            else self.cfg.goal_open_angle_rad
        )
        goal_tol = self.cfg.goal_tolerance_rad
        return goal_angle, goal_tol

    def _make_handle_pose_from_angle(self, opening_sign):
        def handle_pose_from_angle(hinge_pose, ang):
            return self.door_model.handle_pose_from_hinge(
                hinge_pose, ang, self.cfg.frame_map, opening_sign=opening_sign
            )
        return handle_pose_from_angle

    def _make_lambda_for_state(self, lambda_for_pose_xyyaw):
        def lambda_for_state(s):
            p = state_to_pose(s, self._lat_cfg)
            return lambda_for_pose_xyyaw(p.x, p.y, p.yaw)
        return lambda_for_state

    def _make_goal_test(self, lambda_for_state, goal_angle, goal_tol):
        def is_goal(s):
            lam = lambda_for_state(s)
            angles = interval_angles(lam, s.d)
            if not angles:
                return False
            angles_arr = np.array(angles)
            return bool(np.any(np.abs(angles_arr - goal_angle) <= goal_tol))
        return is_goal

    def _make_heuristic(self, lambda_for_state, goal_angle):
        def h(s):
            lam = lambda_for_state(s)
            angles = interval_angles(lam, s.d)
            if not angles:
                return 1e6
            angles_arr = np.array(angles)
            best_gap = np.min(np.abs(angles_arr - goal_angle))
            return float(best_gap)
        return h

    def _should_log_detail(self, tr):
        log_detail = tr.debug_log_detail and tr.expansion_count <= 3
        if tr.expansion_count == 4:
            tr.debug_log_detail = False
        return log_detail

    def _update_expansion_counters(self, s, tr):
        tr.expansion_count += 1
        tr.runtime["succ_calls"] += 1

        if s.d == 1:
            tr.d1_expansion_count += 1

        if tr.expansion_count == 1 or tr.expansion_count % 100 == 0:
            rospy.loginfo(
                "[Planner] Expanded %d states (%d in interval 1)...",
                tr.expansion_count,
                tr.d1_expansion_count
            )

    def _make_switch_edge(self, s, p0, lam0, out, rejected_stats, log_detail):
        if can_switch_intervals(lam0, tol=1e-6):
            ns_switch = pose_to_state(p0, d=(1 - s.d), cfg=self._lat_cfg)
            switch_cost = 1e-3
            out.append((ns_switch, switch_cost))
            rospy.loginfo(
                "[Planner] INTERVAL SWITCH d=%d -> d=%d at (ix=%d,iy=%d,itheta=%d): "
                "angles0=[%.1f,%.1f]deg angles1=[%.1f,%.1f]deg",
                s.d, 1 - s.d, s.ix, s.iy, s.itheta,
                np.degrees(min(lam0.angles0)), np.degrees(max(lam0.angles0)),
                np.degrees(min(lam0.angles1)), np.degrees(max(lam0.angles1)),
            )
        else:
            rejected_stats['switch_unavailable'] += 1

    def _primitive_samples_to_pose_stamped(self, samples):
        return [
            pose_stamped(self.cfg.frame_map, sp.x, sp.y, sp.yaw, 0.0)
            for sp in samples
        ]

    def _collect_lambdas_per_pose(self, samples, lambda_for_pose_xyyaw):
        lambdas_per_pose = []
        for sp in samples:
            lambdas_per_pose.append(lambda_for_pose_xyyaw(sp.x, sp.y, sp.yaw))
        return lambdas_per_pose

    def _extract_angles_for_interval(self, lambdas_per_pose, interval_d, rejected_stats):
        angles_per_pose = []

        for lam in lambdas_per_pose:
            a = interval_angles(lam, interval_d)
            if not a:
                rejected_stats['sample_interval_empty'] += 1
                return False, []
            angles_per_pose.append(a)

        return True, angles_per_pose

    def _check_primitive_continuity(self, angles_per_pose, rejected_stats, log_detail):
        cont_tol = 0.5 * np.radians(self.cfg.door_angle_step_deg)
        cont_ok, surviving_angles, fail_idx = propagate_interval_feasibility(
            angles_per_pose, tol=cont_tol
        )
        if not cont_ok:
            rejected_stats['continuity'] += 1
            if log_detail:
                rospy.loginfo(
                    "[Planner] -> Reject primitive: continuity failed at sample %s",
                    str(fail_idx)
                )
            return False, [], fail_idx

        return True, surviving_angles, None

    def _compute_step_cost(self, base_samples_ps, angles_per_pose, handle_pose_from_angle,
                           hinge_pose_map, primitive_kind="fwd"):
        return transition_cost(
            occ=self._occ,
            base_pose_samples=base_samples_ps,
            lambda_angles_per_pose=angles_per_pose,
            handle_pose_from_angle_fn=handle_pose_from_angle,
            hinge_pose_map=hinge_pose_map,
            cfg=self.cfg.cost,
            primitive_kind=primitive_kind,
        )

    def _append_motion_successor(self, samples, interval_d, out, step_cost, tr):
        plast = samples[-1]
        pN = Pose2D(plast.x, plast.y, plast.yaw)
        ns = pose_to_state(pN, d=interval_d, cfg=self._lat_cfg)
        out.append((ns, step_cost))
        tr.runtime["primitives_accepted"] += 1

    def _make_successor_function(self, prims, lambda_for_pose_xyyaw, lambda_for_state,
                                 handle_pose_from_angle, hinge_pose_map):
        tr = _SuccessorTracking()

        def succ(s):
            succ_t0 = time_module.time()

            self._update_expansion_counters(s, tr)
            log_detail = self._should_log_detail(tr)

            out = []
            p0 = state_to_pose(s, self._lat_cfg)
            lam0 = lambda_for_pose_xyyaw(p0.x, p0.y, p0.yaw)

            self._make_switch_edge(s, p0, lam0, out, tr.rejected, log_detail)

            for kind in prims:
                tr.runtime["primitives_tested"] += 1
                samples = primitive_samples(
                    p0, kind, self._lat_cfg, n=self.cfg.primitive_samples_n
                )

                if self._primitive_hits_occupancy(samples):
                    tr.rejected['primitive_collision'] += 1
                    if log_detail:
                        rospy.loginfo(
                            "[Planner] -> Reject primitive: base footprint hits occupancy"
                        )
                    continue

                base_samples_ps = self._primitive_samples_to_pose_stamped(samples)
                lambdas_per_pose = self._collect_lambdas_per_pose(samples, lambda_for_pose_xyyaw)

                primitive_interval_ok, angles_per_pose = self._extract_angles_for_interval(
                    lambdas_per_pose, s.d, tr.rejected
                )
                if not primitive_interval_ok:
                    continue

                cont_ok, surviving_angles, fail_idx = self._check_primitive_continuity(
                    angles_per_pose, tr.rejected, log_detail
                )
                if not cont_ok:
                    continue

                step_cost = self._compute_step_cost(
                    base_samples_ps, angles_per_pose, handle_pose_from_angle, hinge_pose_map, kind
                )

                if not np.isfinite(step_cost) or step_cost >= 1e9:
                    tr.rejected['cost'] += 1
                    continue

                self._append_motion_successor(samples, s.d, out, step_cost, tr)

            if log_detail:
                log_state_successors(s, lam0, out, tr.rejected)

            tr.runtime["succ_time_total"] += (time_module.time() - succ_t0)
            return out

        return succ, tr

    def _run_search(self, start_s, is_goal, succ, h, time_budget_s):
        time_limit = max(0.2, time_budget_s)

        if self.cfg.use_eps_schedule:
            rospy.loginfo(
                "[Planner] Search mode: repeated weighted A* epsilon schedule "
                "(eps_start=%.2f, eps_end=%.2f, eps_step=%.2f, time_budget=%.2fs)",
                self.cfg.eps_start,
                self.cfg.eps_end,
                self.cfg.eps_step,
                time_limit,
            )
            return eps_schedule_search(
                start=start_s,
                is_goal=is_goal,
                succ=succ,
                heuristic=h,
                eps_start=self.cfg.eps_start,
                eps_end=self.cfg.eps_end,
                eps_step=self.cfg.eps_step,
                total_time_s=time_limit,
            )

        rospy.loginfo(
            "[Planner] Search mode: single weighted A* (w=%.2f, time_budget=%.2fs)",
            self.cfg.w_astar,
            time_limit,
        )
        return weighted_astar(
            start=start_s,
            is_goal=is_goal,
            succ=succ,
            heuristic=h,
            w=self.cfg.w_astar,
            time_limit_s=time_limit,
        )

    def _build_output_path_and_angles(self, r, lambda_for_state, lambda_for_pose_xyyaw,
                                      handle_pose_from_angle, hinge_pose_map,
                                      handle_radius, hinge_yaw):
        base_path = Path()
        base_path.header.stamp = rospy.Time.now()
        base_path.header.frame_id = self.cfg.frame_map
        base_path.poses = []

        angles_out = []
        prev_angle = 0.0

        if r.path:
            st0 = r.path[0]
            lam0 = lambda_for_state(st0)
            a0 = interval_angles(lam0, st0.d)
            if a0:
                prev_angle = float(min(a0))

        for i, st in enumerate(r.path):
            p = state_to_pose(st, self._lat_cfg)
            bp = pose_stamped(self.cfg.frame_map, p.x, p.y, p.yaw, 0.0)
            base_path.poses.append(bp)

            lam = lambda_for_pose_xyyaw(p.x, p.y, p.yaw)
            angles = interval_angles(lam, st.d)

            if not angles:
                rospy.logwarn(
                    "[Planner] No feasible angles at path index %d; reusing previous angle %.3f rad",
                    i, prev_angle
                )
                angles_out.append(float(prev_angle))
                continue

            best_a, best_cost, filtered = pick_best_monotonic_angle(
                base_pose_map=bp,
                hinge_pose_map=hinge_pose_map,
                handle_radius=handle_radius,
                hinge_yaw=hinge_yaw,
                angles=angles,
                handle_pose_from_angle_fn=handle_pose_from_angle,
                cfg=self.cfg.cost,
                prev_angle=prev_angle,
                monotonic_tol=self.cfg.monotonic_angle_tol_rad,
            )

            if best_a is None:
                above = sorted([
                    float(a) for a in angles
                    if float(a) >= float(prev_angle) - self.cfg.monotonic_angle_tol_rad
                ])
                if above:
                    chosen = above[0]
                    rospy.logwarn(
                        "[Planner] Monotonic picker fallback at path index %d: using smallest above-prev angle %.3f rad",
                        i, chosen
                    )
                else:
                    chosen = float(prev_angle)
                    rospy.logwarn(
                        "[Planner] Monotonic picker failed at path index %d: reusing previous angle %.3f rad",
                        i, chosen
                    )
            else:
                chosen = float(best_a)

            if chosen + self.cfg.monotonic_angle_tol_rad < prev_angle:
                rospy.logwarn(
                    "[Planner] Monotonicity violation after selection at path index %d: chosen=%.3f prev=%.3f, clamping",
                    i, chosen, prev_angle
                )
                chosen = float(prev_angle)

            angles_out.append(chosen)
            prev_angle = chosen

        if angles_out:
            rospy.loginfo(
                "[Planner] Final monotonic angle sequence: start=%.2f deg, end=%.2f deg, n=%d",
                np.degrees(angles_out[0]),
                np.degrees(angles_out[-1]),
                len(angles_out)
            )
            rospy.logdebug(
                "[Planner] angles_out_deg=%s",
                str([round(np.degrees(a), 1) for a in angles_out])
            )

        return base_path, angles_out

    def plan(self, base_start, hinge_pose_map, handle_pose_map,
             goal_open_angle_rad, push_motion, hinge_side, time_budget_s):
        """
        Computes a base path + per-waypoint door angle sequence.
        """
        plan_t0 = time_module.time()
        self._lambda._cache.clear()
        self._lambda.reset_stats()

        hx, hy, hinge_yaw, opening_sign, handle_radius = self._extract_hinge_geometry(
            hinge_pose_map, handle_pose_map, push_motion, hinge_side
        )
        start_p = self._base_start_to_pose2d(base_start)

        lambda_for_pose_xyyaw, lambda_state_cache = self._make_lambda_for_pose_xyyaw(
            hx, hy, hinge_yaw, handle_radius, opening_sign, push_motion=push_motion
        )
        lambda_for_state = self._make_lambda_for_state(lambda_for_pose_xyyaw)

        start_lam, start_d, start_s = self._determine_start_state(
            start_p, lambda_for_pose_xyyaw
        )
        log_start_state(start_p, start_s)

        goal_angle, goal_tol = self._compute_goal_settings(goal_open_angle_rad)

        log_geometry_summary(
            goal_angle, goal_tol, push_motion, hinge_side,
            hx, hy, hinge_yaw, handle_radius, opening_sign, start_lam
        )
        rospy.loginfo("[Planner] Starting search from d=%d", start_d)

        log_handle_distances(
            start_p, hx, hy, hinge_yaw, handle_radius, opening_sign, goal_angle,
            self.cfg.reach_min, self.cfg.reach_max
        )

        is_goal = self._make_goal_test(lambda_for_state, goal_angle, goal_tol)
        h = self._make_heuristic(lambda_for_state, goal_angle)
        prims = primitive_set(self._lat_cfg)
        handle_pose_from_angle = self._make_handle_pose_from_angle(opening_sign)

        succ, tr = self._make_successor_function(
            prims,
            lambda_for_pose_xyyaw,
            lambda_for_state,
            handle_pose_from_angle,
            hinge_pose_map
        )

        r = self._run_search(start_s, is_goal, succ, h, time_budget_s)

        plan_elapsed = time_module.time() - plan_t0
        log_search_summary(r, tr, lambda_state_cache, plan_elapsed,
                           self.cfg, self._lambda.get_stats())

        if not r.success or not r.path:
            raise RuntimeError("Planner failed: %s" % r.message)

        base_path, angles_out = self._build_output_path_and_angles(
            r,
            lambda_for_state,
            lambda_for_pose_xyyaw,
            handle_pose_from_angle,
            hinge_pose_map,
            handle_radius,
            hinge_yaw
        )

        return PlanOutput(
            base_path=base_path,
            angles_rad=angles_out,
            expanded_states=r.expands
        )