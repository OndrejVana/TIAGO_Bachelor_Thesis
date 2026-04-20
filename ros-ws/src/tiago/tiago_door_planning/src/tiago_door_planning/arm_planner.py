# -*- coding: utf-8 -*-


from __future__ import print_function, division

import time as time_module
from copy import deepcopy

import numpy as np
import rospy
import moveit_commander
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotState as MoveItRobotState


class ArmTrajConfig(object):
    def __init__(self, group="arm_torso", ee_link="", ik_timeout=0.05,
                 plan_time=2.0, waypoint_dt=0.15, fallback_wrist_rolls=None,
                 door_collision_enabled=False,
                 door_width=0.90, door_thickness=0.04, door_height=2.10,
                 door_collision_handle_clearance=0.35,
                 ik_max_failure_fraction=0.15,
                 ik_max_joint_jump_rad=2.0,
                 unseeded_max_jump_rad=0.5,
                 ik_max_consecutive_gap=2):
        self.group = group
        self.ee_link = ee_link
        self.ik_timeout = ik_timeout
        self.plan_time = plan_time
        self.waypoint_dt = waypoint_dt
        self.fallback_wrist_rolls = list(fallback_wrist_rolls) if fallback_wrist_rolls else []
        self.door_collision_enabled = bool(door_collision_enabled)
        self.door_width = float(door_width)
        self.door_thickness = float(door_thickness)
        self.door_height = float(door_height)
        self.door_collision_handle_clearance = float(door_collision_handle_clearance)
        self.ik_max_failure_fraction = float(ik_max_failure_fraction)
        self.ik_max_joint_jump_rad = float(ik_max_joint_jump_rad)
        self.unseeded_max_jump_rad = float(unseeded_max_jump_rad)
        self.ik_max_consecutive_gap = int(ik_max_consecutive_gap)


class MoveItWaypointIK(object):
    """Solve per-waypoint IK using MoveIt's /compute_ik service."""

    def __init__(self, cfg):
        self.cfg = cfg
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(cfg.group)
        self.group.set_planning_time(cfg.plan_time)

        if cfg.ee_link:
            self.group.set_end_effector_link(cfg.ee_link)

        rospy.loginfo("[ArmPlanner] Waiting for /compute_ik service...")
        rospy.wait_for_service("/compute_ik", timeout=10.0)
        self._ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("[ArmPlanner] /compute_ik service ready.")

        rospy.wait_for_service("/check_state_validity", timeout=10.0)
        self._check_validity_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        rospy.loginfo("[ArmPlanner] /check_state_validity service ready.")

        self._scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.5)  # allow scene interface to initialize
        rospy.loginfo("[ArmPlanner] Planning scene interface ready.")

    def _active_joint_names(self):
        return self.group.get_active_joints()

    def _current_state_with_seed(self, seed):
        jn = self._active_joint_names()
        if len(jn) != len(seed):
            rospy.logwarn(
                "[ArmPlanner] IK seed length (%d) != active joint count (%d). Ignoring seed.",
                len(seed), len(jn)
            )
            return None

        cs = self.group.get_current_state()
        name_to_idx = dict((name, i) for i, name in enumerate(cs.joint_state.name))
        positions = list(cs.joint_state.position)

        for joint_name, joint_value in zip(jn, seed):
            if joint_name in name_to_idx:
                positions[name_to_idx[joint_name]] = float(joint_value)
            else:
                rospy.logwarn("[ArmPlanner] Active joint '%s' not in current state.", joint_name)

        cs.joint_state.position = positions
        return cs

    def _set_start_state_from_seed(self, seed):
        if seed is None:
            self.group.set_start_state_to_current_state()
            return
        seeded_state = self._current_state_with_seed(seed)
        if seeded_state is None:
            self.group.set_start_state_to_current_state()
            return
        self.group.set_start_state(seeded_state)

    def _extract_trajectory_from_plan(self, plan):
        if isinstance(plan, tuple) and len(plan) >= 2:
            success = bool(plan[0])
            if not success:
                rospy.logdebug("[ArmPlanner] MoveIt plan() returned success=False")
            return plan[1] if success else None
        return plan

    def _extract_final_joint_positions(self, traj):
        if traj is None or not hasattr(traj, "joint_trajectory"):
            rospy.logdebug("[ArmPlanner] No valid trajectory returned from MoveIt")
            return None
        if not traj.joint_trajectory.points:
            rospy.logdebug("[ArmPlanner] Empty trajectory points from MoveIt")
            return None
        return list(traj.joint_trajectory.points[-1].positions)

    def _check_joint_config_valid(self, joint_positions):
        """Return True if joint_positions passes MoveIt collision/validity check."""
        req = GetStateValidityRequest()
        req.group_name = self.cfg.group
        rs = MoveItRobotState()
        rs.joint_state.name = self._active_joint_names()
        rs.joint_state.position = list(joint_positions)
        req.robot_state = rs
        try:
            resp = self._check_validity_srv(req)
            return resp.valid
        except rospy.ServiceException as e:
            rospy.logwarn("[ArmPlanner] check_state_validity failed: %s — treating as valid", e)
            return True

    def _call_ik_service(self, pose_stamped, seed=None):
        """
        Call /compute_ik with an optional seed for joint-configuration continuity.
        Returns list of active joint positions, or None on failure.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.cfg.group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self.cfg.ik_timeout * 10)
        req.ik_request.attempts = 10

        if self.cfg.ee_link:
            req.ik_request.ik_link_name = self.cfg.ee_link

        if seed is not None:
            active_names = self._active_joint_names()
            if len(seed) == len(active_names):
                rs = MoveItRobotState()
                rs.joint_state.name = list(active_names)
                rs.joint_state.position = [float(v) for v in seed]
                req.ik_request.robot_state = rs

        try:
            resp = self._ik_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("[ArmPlanner] /compute_ik service exception: %s", str(e))
            return None

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logdebug("[ArmPlanner] IK error code %d", resp.error_code.val)
            return None

        active_names = self._active_joint_names()
        name_to_pos = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
        result = []
        for n in active_names:
            if n not in name_to_pos:
                rospy.logwarn("[ArmPlanner] IK solution missing joint '%s'", n)
                return None
            result.append(name_to_pos[n])
        return result

    def _solve_ik(self, pose, seed=None):
        return self._call_ik_service(pose, seed)

    def _transform_to_base_frame(self, ee_pose_map, base_pose_map):
        """
        Transform ee_pose_map (PoseStamped in map frame) into the base frame
        defined by base_pose_map (PoseStamped in map frame).
        """
        def ps_to_matrix(ps):
            q = [ps.pose.orientation.x, ps.pose.orientation.y,
                 ps.pose.orientation.z, ps.pose.orientation.w]
            T = tft.quaternion_matrix(q)
            T[0, 3] = float(ps.pose.position.x)
            T[1, 3] = float(ps.pose.position.y)
            T[2, 3] = float(ps.pose.position.z)
            return T

        T_map_base = ps_to_matrix(base_pose_map)
        T_map_ee = ps_to_matrix(ee_pose_map)

        T_base_ee = np.dot(np.linalg.inv(T_map_base), T_map_ee)

        ps = PoseStamped()
        ps.header.frame_id = "base_footprint"
        ps.header.stamp = ee_pose_map.header.stamp
        ps.pose.position.x = float(T_base_ee[0, 3])
        ps.pose.position.y = float(T_base_ee[1, 3])
        ps.pose.position.z = float(T_base_ee[2, 3])

        q = tft.quaternion_from_matrix(T_base_ee)
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])

        return ps

    def _pose_with_roll(self, pose_stamped, new_roll):
        """
        Return a copy of pose_stamped with the wrist roll replaced by new_roll.
        """
        o = pose_stamped.pose.orientation
        q = [o.x, o.y, o.z, o.w]
        _, pitch, yaw = tft.euler_from_quaternion(q, axes='sxyz')
        q_new = tft.quaternion_from_euler(float(new_roll), float(pitch), float(yaw), axes='sxyz')
        ps = deepcopy(pose_stamped)
        ps.pose.orientation.x = float(q_new[0])
        ps.pose.orientation.y = float(q_new[1])
        ps.pose.orientation.z = float(q_new[2])
        ps.pose.orientation.w = float(q_new[3])
        return ps

    def _joint_distance(self, q_a, q_b):
        """Max absolute joint difference between two configurations."""
        if q_a is None or q_b is None:
            return float('inf')
        return max(abs(a - b) for a, b in zip(q_a, q_b))

    def _solve_ik_with_fallback(self, pose, seed, waypoint_idx, n):
        """
        Try IK with the given seed first. If that fails:
          1. Retry with no seed (escapes stuck configuration branches).
          2. Try each fallback_wrist_roll (keeps approach yaw, replaces roll).

        Returns (joint_positions, label_str) or (None, None) on total failure.
        """
        candidates = []

        t0 = time_module.time()
        q = self._call_ik_service(pose, seed)
        elapsed = time_module.time() - t0
        if q is not None:
            candidates.append((q, "seeded"))

        if not candidates:
            rospy.logwarn(
                "[ArmPlanner] IK seeded fail at WP %d/%d (%.3fs). Retrying unseeded...",
                waypoint_idx, n, elapsed
            )

        q_unseeded = self._call_ik_service(pose, seed=None)
        if q_unseeded is not None:
            unseeded_jump = self._joint_distance(q_unseeded, seed) if seed is not None else 0.0
            if seed is None or unseeded_jump <= self.cfg.unseeded_max_jump_rad:
                candidates.append((q_unseeded, "unseeded"))
            else:
                rospy.logwarn(
                    "[ArmPlanner] IK WP %d/%d: unseeded jump %.3f rad > unseeded_max_jump "
                    "%.3f rad — discarding (wrong arm configuration).",
                    waypoint_idx, n, unseeded_jump, self.cfg.unseeded_max_jump_rad
                )

        for roll in self.cfg.fallback_wrist_rolls:
            pose_alt = self._pose_with_roll(pose, roll)
            q_alt = self._call_ik_service(pose_alt, seed)
            if q_alt is None:
                q_alt = self._call_ik_service(pose_alt, seed=None)
            if q_alt is not None:
                candidates.append((q_alt, "roll_%.2f" % roll))

        if not candidates:
            rospy.logerr(
                "[ArmPlanner] IK failed all attempts at WP %d/%d "
                "(%d roll variants tried)",
                waypoint_idx, n, len(self.cfg.fallback_wrist_rolls)
            )
            return None, None

        if seed is not None:
            candidates.sort(key=lambda pair: self._joint_distance(pair[0], seed))

        best_q, best_label = candidates[0]
        max_jump = self._joint_distance(best_q, seed) if seed is not None else 0.0

        if seed is not None and max_jump > self.cfg.ik_max_joint_jump_rad:
            rospy.logerr(
                "[ArmPlanner] IK WP %d/%d: best candidate jump %.2f rad > threshold %.2f. "
                "Rejecting (would cause controller abort) — treating as IK failure.",
                waypoint_idx, n, max_jump, self.cfg.ik_max_joint_jump_rad
            )
            return None, None

        if len(candidates) > 1 and best_label != "seeded":
            rospy.logwarn(
                "[ArmPlanner] IK WP %d/%d: seeded failed, using '%s' "
                "(max_jump=%.3f rad from seed)",
                waypoint_idx, n, best_label, max_jump
            )

        return best_q, best_label

    def _update_door_collision(self, hinge_pose, handle_pose):
        """
        Add/update a box collision object 'door' in the MoveIt planning scene.

        Args:
            hinge_pose:  PoseStamped (map frame) — hinge position (fixed)
            handle_pose: PoseStamped (map frame) — current handle position
        """
        hx = float(hinge_pose.pose.position.x)
        hy = float(hinge_pose.pose.position.y)
        handle_x = float(handle_pose.pose.position.x)
        handle_y = float(handle_pose.pose.position.y)

        door_yaw = np.arctan2(handle_y - hy, handle_x - hx)

        clearance = self.cfg.door_collision_handle_clearance
        collision_len = max(0.0, self.cfg.door_width - clearance)
        if collision_len < 0.05:
            return

        cx = hx + (collision_len / 2.0) * np.cos(door_yaw)
        cy = hy + (collision_len / 2.0) * np.sin(door_yaw)
        cz = self.cfg.door_height / 2.0

        q = tft.quaternion_from_euler(0.0, 0.0, door_yaw)

        ps = PoseStamped()
        ps.header.frame_id = hinge_pose.header.frame_id if hinge_pose.header.frame_id else "map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = cx
        ps.pose.position.y = cy
        ps.pose.position.z = cz
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])

        self._scene.add_box("door", ps, (
            collision_len,
            self.cfg.door_thickness,
            self.cfg.door_height,
        ))

    def remove_door_collision(self):
        """Remove the door collision object from the planning scene."""
        self._scene.remove_world_object("door")

    def plan_joint_trajectory(self, ee_target_path, timestamps=None, base_path=None,
                              hinge_pose=None, handle_path=None):
        """
        Solve IK for each pose in ee_target_path and assemble a JointTrajectory.

        Args:
            ee_target_path: nav_msgs/Path of EE target poses (map frame)
            timestamps:     optional list of time_from_start (seconds), one per pose.
                            Falls back to uniform waypoint_dt spacing if None.
            base_path:      optional nav_msgs/Path of base poses (map frame), one per EE
                            waypoint. When provided, each EE pose is transformed into the
                            corresponding waypoint's base frame before IK.
            hinge_pose:     optional PoseStamped — hinge position for door collision object.
                            If provided together with handle_path and
                            cfg.door_collision_enabled=True, the door box is added to the
                            MoveIt planning scene before each IK call.
            handle_path:    optional nav_msgs/Path — handle positions at each dense
                            waypoint, used to compute the door orientation for collision.
        """
        if not ee_target_path.poses:
            raise RuntimeError("Empty ee_target_path")

        n = len(ee_target_path.poses)
        rospy.loginfo("[ArmPlanner] Planning arm trajectory for %d waypoint(s)", n)

        if timestamps is not None and len(timestamps) != n:
            rospy.logwarn(
                "[ArmPlanner] timestamps length (%d) != waypoint count (%d), ignoring",
                len(timestamps), n
            )
            timestamps = None

        use_base_transform = (
            base_path is not None
            and len(base_path.poses) == n
        )
        if base_path is not None and not use_base_transform:
            rospy.logwarn(
                "[ArmPlanner] base_path length (%d) != ee_target_path length (%d), "
                "ignoring base_path for IK frame transform",
                len(base_path.poses) if base_path else 0, n
            )

        use_door_collision = (
            self.cfg.door_collision_enabled
            and hinge_pose is not None
            and handle_path is not None
            and len(handle_path.poses) == n
        )
        if use_door_collision:
            rospy.loginfo("[ArmPlanner] Door collision enabled: updating scene per waypoint.")
        elif self.cfg.door_collision_enabled:
            rospy.logwarn(
                "[ArmPlanner] door_collision_enabled=True but hinge_pose/handle_path missing "
                "or length mismatch — skipping door collision."
            )

        joint_names = self._active_joint_names()
        points = []
        seed = None
        n_failed = 0

        # Pre-compute timestamps so the gap-filler can interpolate correctly.
        all_timestamps = [
            float(timestamps[i]) if timestamps is not None else i * self.cfg.waypoint_dt
            for i in range(n)
        ]
        # all_joints[i] = list of joint positions (IK success) or None (failure).
        all_joints = [None] * n

        for i, ps in enumerate(ee_target_path.poses):
            if use_door_collision:
                self._update_door_collision(hinge_pose, handle_path.poses[i])
                rospy.sleep(0.05)  # let planning scene propagate

            if use_base_transform:
                ik_pose = self._transform_to_base_frame(ps, base_path.poses[i])
                rospy.loginfo(
                    "[ArmPlanner] IK waypoint %d/%d  map=(%.3f,%.3f,%.3f) "
                    "base=(%.3f,%.3f,%.3f)",
                    i + 1, n,
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                    ik_pose.pose.position.x, ik_pose.pose.position.y, ik_pose.pose.position.z
                )
            else:
                ik_pose = ps
                rospy.loginfo(
                    "[ArmPlanner] IK waypoint %d/%d  (x=%.3f y=%.3f z=%.3f)",
                    i + 1, n,
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z
                )

            q, seed_type = self._solve_ik_with_fallback(ik_pose, seed, i + 1, n)

            if q is None:
                n_failed += 1
                rospy.logwarn(
                    "[ArmPlanner] IK failed at WP %d/%d — will fill by interpolation",
                    i + 1, n
                )
            else:
                rospy.loginfo(
                    "[ArmPlanner] IK OK  waypoint %d/%d (%s) (joints: %s...)",
                    i + 1, n, seed_type,
                    ', '.join(['%.3f' % j for j in q[:3]])
                )
                seed = q
                all_joints[i] = list(q)
                
        max_consec = 0
        run = 0
        for jq in all_joints:
            if jq is None:
                run += 1
                if run > max_consec:
                    max_consec = run
            else:
                run = 0

        if max_consec > self.cfg.ik_max_consecutive_gap:
            raise RuntimeError(
                "IK failed at %d consecutive waypoints (limit %d). "
                "The planned base path is kinematically infeasible for continuous "
                "arm motion — the behaviour tree should retry with a new plan."
                % (max_consec, self.cfg.ik_max_consecutive_gap)
            )

        n_filled = 0
        n_unfillable = 0
        for i in range(n):
            if all_joints[i] is not None:
                continue
            prev_i = next((j for j in range(i - 1, -1, -1)
                           if all_joints[j] is not None), None)
            next_i = next((j for j in range(i + 1, n)
                           if all_joints[j] is not None), None)
            if prev_i is None or next_i is None:
                # Leading/trailing gap: no anchor on one side.
                n_unfillable += 1
                continue
            dt = all_timestamps[next_i] - all_timestamps[prev_i]
            alpha = (all_timestamps[i] - all_timestamps[prev_i]) / dt if dt > 1e-9 else 0.5
            interp = [
                (1.0 - alpha) * all_joints[prev_i][k] + alpha * all_joints[next_i][k]
                for k in range(len(all_joints[prev_i]))
            ]
            if not self._check_joint_config_valid(interp):
                rospy.logwarn(
                    "[ArmPlanner] Interpolated config at waypoint %d failed collision check", i
                )
                n_unfillable += 1
                continue
            all_joints[i] = interp
            n_filled += 1

        if n_filled > 0:
            rospy.loginfo(
                "[ArmPlanner] Gap-filled %d/%d IK failures by joint-space interpolation "
                "(max consecutive gap: %d)",
                n_filled, n_failed, max_consec
            )

        # Abort for unfillable leading/trailing waypoints (no anchor on one side).
        max_unfillable = max(1, int(np.ceil(self.cfg.ik_max_failure_fraction * n)))
        if n_unfillable > max_unfillable:
            raise RuntimeError(
                "IK failed at %d/%d waypoints with no interpolation anchor "
                "(limit %d, %.0f%% of %d)"
                % (n_unfillable, n, max_unfillable,
                   self.cfg.ik_max_failure_fraction * 100, n)
            )

        # Build the final trajectory from all_joints (IK successes + interpolated gaps).
        for i in range(n):
            if all_joints[i] is None:
                continue
            pt = JointTrajectoryPoint()
            pt.positions = all_joints[i]
            pt.time_from_start = rospy.Duration.from_sec(all_timestamps[i])
            points.append(pt)

        rospy.loginfo("[ArmPlanner] Arm trajectory planned: %d waypoints", len(points))

        if use_door_collision:
            self.remove_door_collision()
            rospy.loginfo("[ArmPlanner] Door collision object removed from planning scene.")

        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = joint_names
        jt.points = points
        return jt