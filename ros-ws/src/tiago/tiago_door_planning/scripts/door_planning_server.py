#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import rospy
import actionlib
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, OccupancyGrid
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

import tf2_ros

from tiago_door_planning.msg import (
    PlanDoorOpeningAction,
    PlanDoorOpeningResult,
    PlanDoorOpeningFeedback,
)

from tiago_door_planning.planner_core import PlannerCore
from tiago_door_planning.planner_config import PlannerConfig
from tiago_door_planning.traj_gen import (
    BaseTimingConfig,
    compute_base_timestamps,
    build_handle_path_from_angles,
    build_handle_path_from_detected_frame,
    build_grasp_target_path_from_handle_path,
    densify_paths,
    ArmTrajConfig,
    MoveItWaypointIK,
)
from tiago_door_planning.door_model import DoorModel
from tiago_door_planning.execution_monitor import (
    ExecutionMonitorConfig,
    build_execution_reference,
    monitor_execution_reference,
)


class DoorPlanningServer(object):
    """
    Server for planning door opening trajectories:
      - planning in (x,y,theta,d) using PlannerCore
      - handle path from chosen door angles
      - EE target path from handle path + grasp transform
      - arm trajectory via waypoint IK
      - unified sampled execution reference
      - monitoring hooks over execution reference
      - publishes base_path + handle_path + ee_target_path for debugging / BT
    """

    def __init__(self):
        self._load_basic_params()
        self._load_grasp_params()
        self._load_monitor_config()

        self._door_model = self._create_door_model()
        _, self._planner = self._create_planner()
        self._base_timing_cfg = self._create_base_timing_config()
        _, self._ik = self._create_arm_planner()

        self._planner_right, self._ik_right = self._create_arm_specific_planner("right")
        self._planner_left,  self._ik_left  = self._create_arm_specific_planner("left")

        self._init_runtime_state()
        self._init_tf()
        self._init_publishers()
        self._init_subscribers()
        self._init_action_server()


    def _load_basic_params(self):
        self._frame_map = rospy.get_param("~frames/map", "map")
        self._frame_base = rospy.get_param("~frames/base", "base_footprint")

        self._hinge_topic = rospy.get_param("~topics/hinge_pose", "/door/hinge_pose_map")
        self._handle_topic = rospy.get_param("~topics/handle_pose", "/door/handle_pose_map")
        self._hinge_side_topic = rospy.get_param("~topics/hinge_side", "/door/hinge_side")

        self._base_path_out = rospy.get_param("~topics/base_path_out", "/door_plan/base_path")
        self._handle_path_out = rospy.get_param("~topics/handle_path_out", "/door_plan/handle_path")
        self._ee_target_path_out = rospy.get_param("~topics/ee_target_path_out", "/door_plan/ee_target_path")

        self._occ_topic = rospy.get_param("~costmap/occupancy_topic", "")
        self._occ_thresh = int(rospy.get_param("~costmap/occ_threshold", 50))

        self._use_detected_handle_frame = bool(
            rospy.get_param("~planning/use_detected_handle_frame", True)
        )

        self._arm_ik_n_per_segment = int(
            rospy.get_param("~planning/arm_ik_n_per_segment", 3)
        )

    def _load_grasp_params(self):
        # Push grasp parameters
        self._grasp_offset_x = float(rospy.get_param("~planning/grasp_offset_x", 0.0))
        self._grasp_offset_y = float(rospy.get_param("~planning/grasp_offset_y", 0.0))
        self._grasp_offset_z = float(rospy.get_param("~planning/grasp_offset_z", 0.0))
        self._grasp_roll_rad = float(rospy.get_param("~planning/grasp_roll_rad", 0.0))
        self._grasp_pitch_rad = float(rospy.get_param("~planning/grasp_pitch_rad", 0.0))
        self._grasp_yaw_rad = float(rospy.get_param("~planning/grasp_yaw_rad", 0.0))

        # Pull grasp parameters
        self._pull_grasp_offset_x = float(rospy.get_param("~planning/pull_grasp_offset_x", 0.0))
        self._pull_grasp_offset_y = float(rospy.get_param("~planning/pull_grasp_offset_y", 0.0))
        self._pull_grasp_offset_z = float(rospy.get_param("~planning/pull_grasp_offset_z", 0.0))
        self._pull_grasp_roll_rad = float(rospy.get_param("~planning/pull_grasp_roll_rad", 0.0))
        self._pull_grasp_pitch_rad = float(rospy.get_param("~planning/pull_grasp_pitch_rad", 0.0))
        self._pull_grasp_yaw_rad = float(rospy.get_param("~planning/pull_grasp_yaw_rad", np.pi))

    def _load_monitor_config(self):
        self._monitor_cfg = ExecutionMonitorConfig(
            time_monotonic_tol=float(rospy.get_param("~monitor/time_monotonic_tol", 1e-6)),
            angle_monotonic_tol_rad=float(rospy.get_param("~monitor/angle_monotonic_tol_rad", np.radians(0.5))),
            max_base_step_m=float(rospy.get_param("~monitor/max_base_step_m", 0.20)),
            max_base_yaw_step_rad=float(rospy.get_param("~monitor/max_base_yaw_step_rad", 0.60)),
            max_ee_step_m=float(rospy.get_param("~monitor/max_ee_step_m", 0.20)),
            max_handle_step_m=float(rospy.get_param("~monitor/max_handle_step_m", 0.20)),
            arm_time_mismatch_tol=float(rospy.get_param("~monitor/arm_time_mismatch_tol", 0.50)),
            n_per_sparse_step=self._arm_ik_n_per_segment,
        )

    def _create_door_model(self):
        door_width = float(rospy.get_param("~door_model/door_width", 0.90))
        handle_offset = float(rospy.get_param("~door_model/handle_offset_from_hinge", 0.80))
        handle_height = float(
            rospy.get_param(
                "~door_model/door_height",
                rospy.get_param("~door_model/handle_height", 1.0)
            )
        )

        rospy.loginfo(
            "Door model: width=%.2fm, handle_offset=%.2fm, handle_height=%.2fm",
            door_width, handle_offset, handle_height
        )

        return DoorModel(
            door_width=door_width,
            handle_offset_from_hinge=handle_offset,
            handle_height=handle_height,
        )

    def _create_planner(self):
        planner_cfg = PlannerConfig.from_rosparams("~")
        planner = PlannerCore(planner_cfg, door_model=self._door_model)
        return planner_cfg, planner

    def _create_base_timing_config(self):
        return BaseTimingConfig(
            v_max=float(rospy.get_param("~timing/v_max", 0.20)),
            w_max=float(rospy.get_param("~timing/w_max", 0.50)),
            dt_min=float(rospy.get_param("~timing/dt_min", 0.05)),
            dt_max=float(rospy.get_param("~timing/dt_max", 1.00)),
        )

    def _create_arm_planner(self):
        moveit_group = rospy.get_param("~moveit/group", "arm_torso")
        ee_link = rospy.get_param("~moveit/ee_link", "")
        ik_timeout = float(rospy.get_param("~moveit/ik_timeout", 0.05))
        plan_time = float(rospy.get_param("~moveit/plan_time", 2.0))
        waypoint_dt = float(rospy.get_param("~moveit/waypoint_dt", 0.15))
        fallback_wrist_rolls = list(
            rospy.get_param("~moveit/fallback_wrist_rolls", [0.0, 1.5708, 3.14159])
        )
        ik_max_failure_fraction = float(
            rospy.get_param("~moveit/ik_max_failure_fraction", 0.15)
        )
        ik_max_joint_jump_rad = float(
            rospy.get_param("~moveit/ik_max_joint_jump_rad", 2.0)
        )
        unseeded_max_jump_rad = float(
            rospy.get_param("~moveit/unseeded_max_jump_rad", 0.5)
        )
        ik_max_consecutive_gap = int(
            rospy.get_param("~moveit/ik_max_consecutive_gap", 2)
        )

        door_collision_enabled = bool(rospy.get_param("~planning/door_collision_enabled", False))
        door_collision_width = float(rospy.get_param("~door_model/door_width", 0.90))
        door_collision_thickness = float(rospy.get_param("~planner/door_thickness_m", 0.04))
        door_collision_clearance = float(
            rospy.get_param("~planning/door_collision_handle_clearance", 0.35)
        )

        arm_cfg = ArmTrajConfig(
            group=moveit_group,
            ee_link=ee_link,
            ik_timeout=ik_timeout,
            plan_time=plan_time,
            waypoint_dt=waypoint_dt,
            fallback_wrist_rolls=fallback_wrist_rolls,
            door_collision_enabled=door_collision_enabled,
            door_width=door_collision_width,
            door_thickness=door_collision_thickness,
            door_collision_handle_clearance=door_collision_clearance,
            ik_max_failure_fraction=ik_max_failure_fraction,
            ik_max_joint_jump_rad=ik_max_joint_jump_rad,
            unseeded_max_jump_rad=unseeded_max_jump_rad,
            ik_max_consecutive_gap=ik_max_consecutive_gap,
        )
        try:
            ik = MoveItWaypointIK(arm_cfg)
        except RuntimeError as e:
            rospy.logwarn(
                "[PlanningServer] MoveIt group '%s' not found, skipping default IK "
                "(expected for Tiago++ which uses per-arm planners): %s",
                moveit_group, e,
            )
            ik = None
        return arm_cfg, ik

    def _create_arm_specific_planner(self, arm):
        """
        Build a (PlannerCore, MoveItWaypointIK) pair for one arm of Tiago++.

        arm: "right" or "left"
        Returns (None, None) if the per-arm reachability map path is not configured.
        """
        map_param  = "~planner/reachability_map_path_{}_arm".format(arm)
        group_param = "~moveit/group_{}_arm".format(arm)
        ee_param    = "~moveit/ee_link_{}_arm".format(arm)

        map_path = str(rospy.get_param(map_param, "")).strip()
        if not map_path:
            rospy.loginfo(
                "[PlanningServer] No reachability map configured for %s arm (%s) "
                "— dual-arm mode disabled for this arm.",
                arm, map_param,
            )
            return None, None

        rospy.loginfo("[PlanningServer] Building %s-arm planner (map: %s)", arm, map_path)

        cfg = PlannerConfig.from_rosparams("~")
        cfg.reachability_map_path = map_path

        offset_param = "~planner/grasp_yaw_offset_rad_{}_arm".format(arm)
        if rospy.has_param(offset_param):
            cfg.grasp_yaw_offset_rad = float(rospy.get_param(offset_param))
            rospy.loginfo(
                "[PlanningServer] %s-arm grasp_yaw_offset_rad overridden to %.4f rad",
                arm, cfg.grasp_yaw_offset_rad,
            )

        planner = PlannerCore(cfg, door_model=self._door_model)

        # Build the arm-specific MoveIt IK runner.
        moveit_group = str(rospy.get_param(group_param, "arm_{}_torso".format(arm)))
        ee_link      = str(rospy.get_param(ee_param,    "arm_{}_7_link".format(arm)))

        arm_cfg = ArmTrajConfig(
            group=moveit_group,
            ee_link=ee_link,
            ik_timeout=float(rospy.get_param("~moveit/ik_timeout", 0.05)),
            plan_time=float(rospy.get_param("~moveit/plan_time", 2.0)),
            waypoint_dt=float(rospy.get_param("~moveit/waypoint_dt", 0.15)),
            fallback_wrist_rolls=list(
                rospy.get_param("~moveit/fallback_wrist_rolls", [0.0, 1.5708, 3.14159])
            ),
            door_collision_enabled=bool(rospy.get_param("~planning/door_collision_enabled", False)),
            door_width=float(rospy.get_param("~door_model/door_width", 0.90)),
            door_thickness=float(rospy.get_param("~planner/door_thickness_m", 0.04)),
            door_collision_handle_clearance=float(
                rospy.get_param("~planning/door_collision_handle_clearance", 0.35)
            ),
            ik_max_failure_fraction=float(rospy.get_param("~moveit/ik_max_failure_fraction", 0.15)),
            ik_max_joint_jump_rad=float(rospy.get_param("~moveit/ik_max_joint_jump_rad", 2.0)),
            unseeded_max_jump_rad=float(rospy.get_param("~moveit/unseeded_max_jump_rad", 0.5)),
            ik_max_consecutive_gap=int(rospy.get_param("~moveit/ik_max_consecutive_gap", 2)),
        )
        ik = MoveItWaypointIK(arm_cfg)

        rospy.loginfo(
            "[PlanningServer] %s-arm planner ready (MoveIt group: %s, ee_link: %s)",
            arm, moveit_group, ee_link,
        )
        return planner, ik

    def _init_runtime_state(self):
        self._hinge = None
        self._handle = None
        self._hinge_side = None
        self._occ = None

        self._last_execution_samples = []
        self._last_execution_monitor_report = {}

    def _init_tf(self):
        self._tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tfl = tf2_ros.TransformListener(self._tfbuf)

    def _init_publishers(self):
        self._pub_base = rospy.Publisher(self._base_path_out, Path, queue_size=1, latch=True)
        self._pub_base_poses = rospy.Publisher(
            self._base_path_out + "_poses", PoseArray, queue_size=1, latch=True
        )
        self._pub_handle = rospy.Publisher(self._handle_path_out, Path, queue_size=1, latch=True)
        self._pub_ee_target = rospy.Publisher(self._ee_target_path_out, Path, queue_size=1, latch=True)
        self._pub_ee_poses = rospy.Publisher(
            self._ee_target_path_out + "_poses", PoseArray, queue_size=1, latch=True
        )
        self._pub_display_traj = rospy.Publisher(
            "/door_plan/display_trajectory", DisplayTrajectory, queue_size=1, latch=True
        )

    def _init_subscribers(self):
        rospy.Subscriber(self._hinge_topic, PoseStamped, self._cb_hinge, queue_size=1)
        rospy.Subscriber(self._handle_topic, PoseStamped, self._cb_handle, queue_size=1)
        rospy.Subscriber(self._hinge_side_topic, String, self._cb_hinge_side, queue_size=1)

        if self._occ_topic:
            rospy.Subscriber(self._occ_topic, OccupancyGrid, self._cb_occ, queue_size=1)

    def _init_action_server(self):
        self._as = actionlib.SimpleActionServer(
            "plan_door_opening",
            PlanDoorOpeningAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._as.start()


    def _cb_hinge(self, msg):
        self._hinge = msg

    def _cb_handle(self, msg):
        self._handle = msg

    def _cb_hinge_side(self, msg):
        value = str(msg.data).strip().lower()
        self._hinge_side = value if value else None

    def _cb_occ(self, msg):
        self._occ = msg
        self._planner.set_occupancy(msg, occ_threshold=self._occ_thresh)


    def _get_base_pose(self):
        trans = self._tfbuf.lookup_transform(
            self._frame_map,
            self._frame_base,
            rospy.Time(0),
            rospy.Duration(1.0)
        )

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self._frame_map
        ps.pose.position.x = trans.transform.translation.x
        ps.pose.position.y = trans.transform.translation.y
        ps.pose.position.z = trans.transform.translation.z
        ps.pose.orientation = trans.transform.rotation
        return ps

    def _wait_for_inputs(self, allowed_time_s, fb):
        t0 = rospy.Time.now()

        while not rospy.is_shutdown() and (
            self._hinge is None or
            self._handle is None or
            self._hinge_side is None
        ):
            if self._as.is_preempt_requested():
                raise RuntimeError("Preempted")

            fb.stage = "waiting_for_door_poses"
            fb.progress = 0.0
            fb.expanded_states = 0
            self._as.publish_feedback(fb)
            rospy.sleep(0.05)

            if (rospy.Time.now() - t0).to_sec() > max(1.0, allowed_time_s):
                raise RuntimeError("Timeout waiting for hinge/handle/hinge_side inputs")

        return self._hinge, self._handle, self._hinge_side

    def _publish_feedback(self, fb, stage, progress, expanded_states=None):
        fb.stage = stage
        fb.progress = progress
        if expanded_states is not None:
            fb.expanded_states = int(expanded_states)
        self._as.publish_feedback(fb)

    def _validate_plan_output(self, base_path, angles_rad):
        if not base_path.poses or len(base_path.poses) < 1:
            raise RuntimeError("Planner produced empty base_path")

        if len(angles_rad) != len(base_path.poses):
            raise RuntimeError("Planner angles_rad length != base_path length")

    def _log_base_plan_summary(self, base_path, angles_rad):
        rospy.loginfo(
            "[PlanningServer] Base path planning completed: %d waypoint(s), angles from %.1f to %.1f deg",
            len(base_path.poses),
            np.degrees(angles_rad[0]) if angles_rad else 0.0,
            np.degrees(angles_rad[-1]) if angles_rad else 0.0
        )


    def _build_handle_path(self, hinge_pose, handle_pose, hinge_side, goal, angles_rad):
        opening_sign = self._planner.opening_sign(goal.push_motion, hinge_side)

        if self._use_detected_handle_frame:
            rospy.loginfo("[PlanningServer] Building handle path from detected handle frame propagation")
            return build_handle_path_from_detected_frame(
                hinge_pose_map=hinge_pose,
                detected_handle_pose_map=handle_pose,
                angles_rad=angles_rad,
                frame_id=self._frame_map,
                opening_sign=opening_sign,
            )

        rospy.logwarn("[PlanningServer] Falling back to synthetic handle path from door model")
        return build_handle_path_from_angles(
            door_model=self._door_model,
            hinge_pose_map=hinge_pose,
            angles_rad=angles_rad,
            frame_id=self._frame_map,
            opening_sign=opening_sign,
        )

    def _build_ee_target_path(self, handle_path, push_motion, base_path=None,
                              hinge_yaw=None):
        if push_motion:
            ox = self._grasp_offset_x
            oy = self._grasp_offset_y
            oz = self._grasp_offset_z
            roll = self._grasp_roll_rad
            pitch = self._grasp_pitch_rad
            yaw = self._grasp_yaw_rad
            side = "push"
        else:
            ox = self._pull_grasp_offset_x
            oy = self._pull_grasp_offset_y
            oz = self._pull_grasp_offset_z
            roll = self._pull_grasp_roll_rad
            pitch = self._pull_grasp_pitch_rad
            yaw = self._pull_grasp_yaw_rad
            side = "pull"

        rospy.loginfo(
            "[PlanningServer] Building EE target path from handle path with %s grasp transform "
            "(xyz=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f])",
            side, ox, oy, oz, roll, pitch, yaw,
        )

        return build_grasp_target_path_from_handle_path(
            handle_path=handle_path,
            frame_id=self._frame_map,
            offset_x=ox,
            offset_y=oy,
            offset_z=oz,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
        )

    def _build_execution_samples(self, base_path, angles_rad, handle_path, ee_target_path):
        base_times = compute_base_timestamps(base_path, self._base_timing_cfg)

        execution_samples = build_execution_reference(
            base_path=base_path,
            base_times=base_times,
            angles_rad=angles_rad,
            handle_path=handle_path,
            ee_target_path=ee_target_path,
        )

        self._last_execution_samples = execution_samples
        self._log_execution_reference_summary(execution_samples)
        return execution_samples, base_times


    def _generate_arm_traj_if_requested(self, goal, ee_target_path, timestamps, fb,
                                         base_path=None, hinge_pose=None, handle_path=None):
        arm_traj = JointTrajectory()

        if not goal.generate_arm_traj:
            return arm_traj

        rospy.loginfo("[PlanningServer] Starting arm trajectory generation (MoveIt IK + planning)...")
        self._publish_feedback(fb, "waypoint_ik", 0.7)

        try:
            arm_traj = self._ik.plan_joint_trajectory(
                ee_target_path,
                timestamps=timestamps,
                base_path=base_path,
                hinge_pose=hinge_pose,
                handle_path=handle_path,
            )
            rospy.loginfo("[PlanningServer] Arm trajectory generation completed successfully")
        except Exception as ik_error:
            rospy.logerr("[PlanningServer] Arm trajectory generation failed: %s", str(ik_error))
            raise

        return arm_traj

    def _monitor_execution(self, execution_samples, arm_traj):
        monitor_report = monitor_execution_reference(
            execution_samples=execution_samples,
            arm_traj=arm_traj,
            monitor_cfg=self._monitor_cfg,
        )

        self._last_execution_monitor_report = monitor_report
        self._log_execution_monitor_report(monitor_report)
        return monitor_report

    def _publish_paths_if_requested(self, goal, base_path, handle_path, ee_target_path, arm_traj=None):
        if not goal.publish_paths:
            return

        self._pub_base.publish(base_path)

        pa_base = PoseArray()
        pa_base.header = base_path.header
        pa_base.poses = [ps.pose for ps in base_path.poses]
        self._pub_base_poses.publish(pa_base)

        self._pub_handle.publish(handle_path)
        self._pub_ee_target.publish(ee_target_path)

        pa = PoseArray()
        pa.header = ee_target_path.header
        pa.poses = [ps.pose for ps in ee_target_path.poses]
        self._pub_ee_poses.publish(pa)

        if arm_traj is not None and arm_traj.points:
            dt = DisplayTrajectory()
            dt.model_id = ""
            rt = RobotTrajectory()
            rt.joint_trajectory = arm_traj
            dt.trajectory.append(rt)
            self._pub_display_traj.publish(dt)
            rospy.loginfo("[PlanningServer] Published arm trajectory for visualization "
                          "(/door_plan/display_trajectory)")


    def _fill_success_result(self, res, base_path, handle_path, arm_traj, base_times, monitor_report):
        res.success = True

        if monitor_report.get("ok", False):
            res.message = "Planned successfully with PlannerCore"
        else:
            res.message = "Planned successfully with PlannerCore (execution monitor warnings present)"

        res.base_path = base_path
        res.handle_path = handle_path
        res.arm_trajectory = arm_traj
        res.base_times = list(base_times)

    def _fill_error_result(self, res, error):
        res.success = False
        res.message = str(error)


    def _log_execution_reference_summary(self, execution_samples):
        if not execution_samples:
            rospy.logwarn("[PlanningServer] Execution reference is empty")
            return

        t0 = execution_samples[0].time_from_start
        tN = execution_samples[-1].time_from_start
        a0 = execution_samples[0].door_angle_rad
        aN = execution_samples[-1].door_angle_rad

        rospy.loginfo(
            "[PlanningServer] Execution reference built: n=%d, duration=%.3fs, angle_start=%.1f deg, angle_end=%.1f deg",
            len(execution_samples),
            max(0.0, tN - t0),
            np.degrees(a0),
            np.degrees(aN),
        )

        first = execution_samples[0]
        last = execution_samples[-1]

        rospy.loginfo(
            "[PlanningServer] First sample: t=%.3f base=(%.3f, %.3f) handle=(%.3f, %.3f, %.3f) ee=(%.3f, %.3f, %.3f)",
            first.time_from_start,
            first.base_pose.pose.position.x,
            first.base_pose.pose.position.y,
            first.handle_pose.pose.position.x,
            first.handle_pose.pose.position.y,
            first.handle_pose.pose.position.z,
            first.ee_target_pose.pose.position.x,
            first.ee_target_pose.pose.position.y,
            first.ee_target_pose.pose.position.z,
        )

        rospy.loginfo(
            "[PlanningServer] Last sample: t=%.3f base=(%.3f, %.3f) handle=(%.3f, %.3f, %.3f) ee=(%.3f, %.3f, %.3f)",
            last.time_from_start,
            last.base_pose.pose.position.x,
            last.base_pose.pose.position.y,
            last.handle_pose.pose.position.x,
            last.handle_pose.pose.position.y,
            last.handle_pose.pose.position.z,
            last.ee_target_pose.pose.position.x,
            last.ee_target_pose.pose.position.y,
            last.ee_target_pose.pose.position.z,
        )

    def _log_execution_monitor_report(self, report):
        if not report:
            rospy.logwarn("[PlanningServer] Empty execution monitor report")
            return

        metrics = report.get("metrics", {})
        warnings = report.get("warnings", [])
        ok = bool(report.get("ok", False))

        rospy.loginfo(
            "[PlanningServer] Execution monitor: ok=%s, samples=%d, warnings=%d",
            ok,
            metrics.get("num_samples", 0),
            len(warnings),
        )

        rospy.loginfo(
            "[PlanningServer] Execution monitor metrics: "
            "max_base_step=%.3f m, max_base_yaw_step=%.3f rad, max_handle_step=%.3f m, max_ee_step=%.3f m, "
            "angle_nonmono=%d, time_nonmono=%d, arm_points=%d",
            metrics.get("max_base_step_m", 0.0),
            metrics.get("max_base_yaw_step_rad", 0.0),
            metrics.get("max_handle_step_m", 0.0),
            metrics.get("max_ee_step_m", 0.0),
            metrics.get("angle_nonmonotonic_count", 0),
            metrics.get("time_nonmonotonic_count", 0),
            metrics.get("arm_point_count", 0),
        )

        for w in warnings:
            rospy.logwarn("[PlanningServer] Execution monitor warning: %s", w)


    def get_last_execution_reference(self):
        return self._last_execution_samples

    def get_last_execution_monitor_report(self):
        return self._last_execution_monitor_report


    def _select_arm_for_hinge_side(self, hinge_side):
        """
        Return "right" or "left" based on hinge_side.
          hinge_side == "left"  → handle is on the right → use right arm
          hinge_side == "right" → handle is on the left  → use left arm
        """
        return "right" if str(hinge_side).strip().lower() == "left" else "left"

    def _execute(self, goal):
        fb = PlanDoorOpeningFeedback()
        res = PlanDoorOpeningResult()

        saved_planner = self._planner
        saved_ik      = self._ik

        try:
            hinge_pose, handle_pose, hinge_side = self._wait_for_inputs(
                goal.allowed_planning_time, fb
            )

            active_arm = self._select_arm_for_hinge_side(hinge_side)
            rospy.loginfo(
                "[PlanningServer] hinge_side='%s' → using %s arm for planning",
                hinge_side, active_arm,
            )

            if active_arm == "right" and self._planner_right is not None:
                self._planner = self._planner_right
                self._ik      = self._ik_right
            elif active_arm == "left" and self._planner_left is not None:
                self._planner = self._planner_left
                self._ik      = self._ik_left


            if self._occ is not None:
                self._planner.set_occupancy(self._occ, occ_threshold=self._occ_thresh)

            base_start = self._get_base_pose()

            self._publish_feedback(fb, "planning_core", 0.1)

            plan_out = self._planner.plan(
                base_start=base_start,
                hinge_pose_map=hinge_pose,
                handle_pose_map=handle_pose,
                goal_open_angle_rad=goal.goal_open_angle_rad,
                push_motion=goal.push_motion,
                hinge_side=hinge_side,
                time_budget_s=max(0.5, float(goal.allowed_planning_time)),
            )

            base_path = plan_out.base_path
            angles_rad = plan_out.angles_rad
            fb.expanded_states = int(getattr(plan_out, "expanded_states", 0))

            self._validate_plan_output(base_path, angles_rad)

            if angles_rad and angles_rad[0] > 1e-6:
                base_path.poses.insert(0, base_start)
                angles_rad = [0.0] + list(angles_rad)


            self._log_base_plan_summary(base_path, angles_rad)

            self._publish_feedback(fb, "building_handle_path", 0.5, fb.expanded_states)

            handle_path = self._build_handle_path(
                hinge_pose=hinge_pose,
                handle_pose=handle_pose,
                hinge_side=hinge_side,
                goal=goal,
                angles_rad=angles_rad,
            )
            
            hinge_yaw = np.arctan2(
                handle_pose.pose.position.y - hinge_pose.pose.position.y,
                handle_pose.pose.position.x - hinge_pose.pose.position.x,
            )

            if not goal.push_motion:
                hinge_yaw = hinge_yaw + np.pi
            rospy.loginfo(
                "[PlanningServer] hinge_yaw = %.3f rad (%.1f deg) from hinge (%.3f, %.3f) to handle (%.3f, %.3f)%s",
                hinge_yaw, np.degrees(hinge_yaw),
                hinge_pose.pose.position.x, hinge_pose.pose.position.y,
                handle_pose.pose.position.x, handle_pose.pose.position.y,
                " [pull: flipped by pi]" if not goal.push_motion else "",
            )

            sparse_ee_target_path = self._build_ee_target_path(
                handle_path, goal.push_motion, base_path, hinge_yaw=hinge_yaw
            )

            execution_samples, base_times = self._build_execution_samples(
                base_path=base_path,
                angles_rad=angles_rad,
                handle_path=handle_path,
                ee_target_path=sparse_ee_target_path,
            )

            dense_base_path, dense_handle_path, dense_times = densify_paths(
                base_path=base_path,
                handle_path=handle_path,
                base_times=base_times,
                n_per_segment=self._arm_ik_n_per_segment,
                frame_id=self._frame_map,
            )

            dense_ee_target_path = self._build_ee_target_path(
                dense_handle_path, goal.push_motion, dense_base_path, hinge_yaw=hinge_yaw
            )

            arm_traj = self._generate_arm_traj_if_requested(
                goal=goal,
                ee_target_path=dense_ee_target_path,
                timestamps=dense_times,
                fb=fb,
                base_path=dense_base_path,
                hinge_pose=hinge_pose,
                handle_path=dense_handle_path,
            )

            monitor_report = self._monitor_execution(
                execution_samples=execution_samples,
                arm_traj=arm_traj,
            )

            self._publish_paths_if_requested(
                goal=goal,
                base_path=base_path,
                handle_path=handle_path,
                ee_target_path=dense_ee_target_path,
                arm_traj=arm_traj,
            )

            self._fill_success_result(
                res=res,
                base_path=base_path,
                handle_path=handle_path,
                arm_traj=arm_traj,
                base_times=base_times,
                monitor_report=monitor_report,
            )

            self._publish_feedback(fb, "done", 1.0, fb.expanded_states)
            self._as.set_succeeded(res)

        except Exception as e:
            self._planner, self._ik = saved_planner, saved_ik
            if str(e) == "Preempted":
                self._as.set_preempted()
                return

            self._fill_error_result(res, e)
            self._as.set_aborted(res)

        else:
            self._planner, self._ik = saved_planner, saved_ik


def main():
    rospy.init_node("tiago_door_planning")
    _ = DoorPlanningServer()
    rospy.spin()


if __name__ == "__main__":
    main()