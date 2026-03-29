# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import rospy

from .costs import CostConfig


# ============================================================
# _SuccessorTracking
# ============================================================

class _SuccessorTracking(object):
    """Mutable counters and stats accumulated inside the successor function."""

    def __init__(self):
        self.expansion_count = 0
        self.d1_expansion_count = 0
        self.debug_log_detail = True
        self.rejected = {
            'switch_unavailable': 0,
            'primitive_collision': 0,
            'sample_interval_empty': 0,
            'continuity': 0,
            'cost': 0,
        }
        self.runtime = {
            "succ_calls": 0,
            "succ_time_total": 0.0,
            "primitives_tested": 0,
            "primitives_accepted": 0,
        }


# ============================================================
# PlannerConfig
# ============================================================

class PlannerConfig(object):
    def __init__(self,
                 frame_map="map",
                 xy_res=0.05,
                 theta_bins=16,
                 step_m=0.10,
                 arc_radius_m=0.40,
                 allow_reverse=True,
                 primitive_samples_n=10,
                 door_open_angle_rad=None,
                 door_angle_step_deg=2.0,
                 door_thickness_m=0.04,
                 robot_radius=0.30,
                 reach_min=0.35,
                 reach_max=0.85,
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
                 use_eps_schedule=True,
                 w_astar=2.0,
                 eps_start=4.0,
                 eps_end=1.0,
                 eps_step=1.0,
                 goal_open_angle_rad=None,
                 goal_tolerance_rad=None,
                 occ_threshold=50,
                 cost=None,
                 monotonic_angle_tol_rad=None):
        self.frame_map = frame_map
        self.xy_res = xy_res
        self.theta_bins = theta_bins
        self.step_m = step_m
        self.arc_radius_m = arc_radius_m
        self.allow_reverse = allow_reverse
        self.primitive_samples_n = primitive_samples_n
        self.door_open_angle_rad = (
            door_open_angle_rad if door_open_angle_rad is not None else np.radians(90.0)
        )
        self.door_angle_step_deg = door_angle_step_deg
        self.door_thickness_m = door_thickness_m
        self.robot_radius = robot_radius
        self.reach_min = reach_min
        self.reach_max = reach_max
        self.monotonic_angle_tol_rad = (
            monotonic_angle_tol_rad
            if monotonic_angle_tol_rad is not None
            else np.radians(0.5)
        )

        self.handle_height = handle_height
        self.reach_lateral_factor = reach_lateral_factor
        self.max_reach_angle_deg = max_reach_angle_deg
        self.min_elevation_deg = min_elevation_deg
        self.max_elevation_deg = max_elevation_deg

        self.reachability_backend = str(reachability_backend).strip().lower()
        self.reachability_map_path = str(reachability_map_path)
        self.reachability_map_path_right_arm = ""
        self.reachability_map_path_left_arm  = ""
        self.reachability_fixed_z = float(reachability_fixed_z)
        self.reachability_z_tol = float(reachability_z_tol)
        self.reachability_y_exclusion_half_width_m = float(reachability_y_exclusion_half_width_m)
        self.use_grasp_yaw = bool(use_grasp_yaw)
        self.grasp_yaw_offset_rad = float(grasp_yaw_offset_rad)

        self.use_eps_schedule = use_eps_schedule
        self.w_astar = w_astar
        self.eps_start = eps_start
        self.eps_end = eps_end
        self.eps_step = eps_step
        self.goal_open_angle_rad = (
            goal_open_angle_rad if goal_open_angle_rad is not None else np.radians(85.0)
        )
        self.goal_tolerance_rad = (
            goal_tolerance_rad if goal_tolerance_rad is not None else np.radians(5.0)
        )
        self.occ_threshold = occ_threshold
        self.cost = cost if cost is not None else CostConfig()

    @staticmethod
    def from_rosparams(ns):
        """
        Build a PlannerConfig from ROS parameters.
        ns should be "~" when called from the node.
        """
        def gp(name, default):
            return rospy.get_param(ns + name if not name.startswith("/") else name, default)

        cfg = PlannerConfig()
        cfg.frame_map = gp("frames/map", cfg.frame_map)
        cfg.xy_res = float(gp("planner/xy_resolution", cfg.xy_res))
        cfg.theta_bins = int(gp("planner/theta_bins", cfg.theta_bins))

        cfg.step_m = float(gp("planner/primitive_step", cfg.step_m))
        cfg.arc_radius_m = float(gp("planner/arc_radius_m", cfg.arc_radius_m))
        cfg.allow_reverse = bool(gp("planner/allow_reverse", cfg.allow_reverse))
        cfg.primitive_samples_n = int(gp("planner/primitive_samples_n", cfg.primitive_samples_n))

        cfg.door_open_angle_rad = float(gp("planner/door_open_angle_rad", cfg.door_open_angle_rad))
        cfg.door_angle_step_deg = float(gp("planner/door_angle_step_deg", cfg.door_angle_step_deg))
        cfg.door_thickness_m = float(gp("planner/door_thickness_m", cfg.door_thickness_m))

        cfg.robot_radius = float(gp("planner/robot_radius", cfg.robot_radius))
        cfg.reach_min = float(gp("planner/reach_min", cfg.reach_min))
        cfg.reach_max = float(gp("planner/reach_max", cfg.reach_max))

        cfg.handle_height = float(gp("planner/handle_height", cfg.handle_height))
        cfg.reach_lateral_factor = float(gp("planner/reach_lateral_factor", cfg.reach_lateral_factor))
        cfg.max_reach_angle_deg = float(gp("planner/max_reach_angle_deg", cfg.max_reach_angle_deg))
        cfg.min_elevation_deg = float(gp("planner/min_elevation_deg", cfg.min_elevation_deg))
        cfg.max_elevation_deg = float(gp("planner/max_elevation_deg", cfg.max_elevation_deg))

        cfg.reachability_backend = str(
            gp("planner/reachability_backend", cfg.reachability_backend)
        ).strip().lower()
        cfg.reachability_map_path = str(gp("planner/reachability_map_path", cfg.reachability_map_path))
        cfg.reachability_map_path_right_arm = str(
            gp("planner/reachability_map_path_right_arm", ""))
        cfg.reachability_map_path_left_arm = str(
            gp("planner/reachability_map_path_left_arm", ""))
        cfg.reachability_fixed_z = float(gp("planner/reachability_fixed_z", cfg.reachability_fixed_z))
        cfg.reachability_z_tol = float(gp("planner/reachability_z_tol", cfg.reachability_z_tol))
        cfg.reachability_y_exclusion_half_width_m = float(
            gp("planner/reachability_y_exclusion_half_width_m",
               cfg.reachability_y_exclusion_half_width_m)
        )
        cfg.use_grasp_yaw = bool(gp("planner/use_grasp_yaw", cfg.use_grasp_yaw))
        cfg.grasp_yaw_offset_rad = float(gp("planning/grasp_yaw_offset_rad", cfg.grasp_yaw_offset_rad))

        cfg.use_eps_schedule = bool(gp("planner/use_eps_schedule", cfg.use_eps_schedule))
        cfg.w_astar = float(gp("planner/w_astar", cfg.w_astar))
        cfg.eps_start = float(gp("planner/eps_start", cfg.eps_start))
        cfg.eps_end = float(gp("planner/eps_end", cfg.eps_end))
        cfg.eps_step = float(gp("planner/eps_step", cfg.eps_step))

        cfg.goal_open_angle_rad = float(gp("planner/goal_open_angle_rad", cfg.goal_open_angle_rad))
        cfg.goal_tolerance_rad = float(gp("planner/goal_tolerance_rad", cfg.goal_tolerance_rad))

        cfg.occ_threshold = int(gp("costmap/occ_threshold", cfg.occ_threshold))

        cfg.cost.occ_threshold = cfg.occ_threshold
        cfg.cost.w_costmap = float(gp("costs/w_costmap", cfg.cost.w_costmap))
        cfg.cost.w_arm = float(gp("costs/w_arm", cfg.cost.w_arm))
        cfg.cost.arm_nominal_dist = float(gp("costs/arm_nominal_dist", cfg.cost.arm_nominal_dist))
        cfg.cost.arm_sigma = float(gp("costs/arm_sigma", cfg.cost.arm_sigma))
        cfg.cost.arm_min_dist = float(gp("costs/arm_min_dist", cfg.cost.arm_min_dist))
        cfg.cost.arm_max_dist = float(gp("costs/arm_max_dist", cfg.cost.arm_max_dist))
        cfg.cost.arm_hard_penalty = float(gp("costs/arm_hard_penalty", cfg.cost.arm_hard_penalty))
        cfg.cost.arm_centerline_danger_m = float(
            gp("costs/arm_centerline_danger_m", cfg.cost.arm_centerline_danger_m)
        )
        cfg.cost.arm_centerline_penalty = float(
            gp("costs/arm_centerline_penalty", cfg.cost.arm_centerline_penalty)
        )
        cfg.cost.w_reverse_straight = float(
            gp("costs/w_reverse_straight", cfg.cost.w_reverse_straight)
        )
        cfg.cost.w_reverse_arc = float(
            gp("costs/w_reverse_arc", cfg.cost.w_reverse_arc)
        )
        cfg.cost.w_rotation = float(
            gp("costs/w_rotation", cfg.cost.w_rotation)
        )

        cfg.monotonic_angle_tol_rad = float(
            gp("planner/monotonic_angle_tol_rad", cfg.monotonic_angle_tol_rad)
        )

        return cfg


# ============================================================
# PlanOutput
# ============================================================

class PlanOutput(object):
    """Result container returned by PlannerCore.plan()."""

    def __init__(self, base_path, angles_rad, expanded_states):
        self.base_path = base_path
        self.angles_rad = angles_rad
        self.expanded_states = expanded_states