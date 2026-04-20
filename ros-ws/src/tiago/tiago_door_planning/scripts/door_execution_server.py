#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import rospy
import actionlib
import numpy as np
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTolerance,
)

from tiago_door_planning.msg import (
    ExecuteDoorOpeningAction,
    ExecuteDoorOpeningResult,
    ExecuteDoorOpeningFeedback,
)
from sensor_msgs.msg import JointState
from tiago_door_planning.utils import angle_wrap


class DoorExecutionServer(object):
    """
    Action server that executes door opening by coordinating:
    - Base motion via cmd_vel (velocity control)
    - Arm motion via FollowJointTrajectoryAction

    Both execute simultaneously with time synchronization.
    """

    def __init__(self):
        self._load_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_tf()
        self._init_arm_client()
        self._init_action_server()

    # ============================================================
    # Initialization
    # ============================================================

    def _load_parameters(self):
        # Frames
        self._frame_map = rospy.get_param("~frames/map", "map")
        self._frame_base = rospy.get_param("~frames/base", "base_footprint")

        # Control parameters
        self._control_rate = float(rospy.get_param("~control_rate", 20.0))  # Hz
        self._position_tolerance = float(rospy.get_param("~position_tolerance", 0.05))  # m
        self._angle_tolerance = float(rospy.get_param("~angle_tolerance", 0.1))  # rad

        # Velocity limits
        self._max_linear_vel = float(rospy.get_param("~max_linear_vel", 0.3))  # m/s
        self._max_angular_vel = float(rospy.get_param("~max_angular_vel", 0.6))  # rad/s

        # PID gains for base tracking
        self._kp_linear = float(rospy.get_param("~kp_linear", 1.0))
        self._kp_angular = float(rospy.get_param("~kp_angular", 2.0))

        self._max_base_lag_m = float(rospy.get_param("~max_base_lag_m", 0.15))

        # External interfaces — dual-arm (Tiago++) or single-arm fallback
        _arm_legacy = rospy.get_param("~arm_controller", "")
        self._arm_controller_right = rospy.get_param(
            "~arm_controller_right",
            _arm_legacy if _arm_legacy else "/arm_right_controller/follow_joint_trajectory",
        )
        self._arm_controller_left = rospy.get_param(
            "~arm_controller_left",
            _arm_legacy if _arm_legacy else "/arm_left_controller/follow_joint_trajectory",
        )
        self._torso_controller = rospy.get_param(
            "~torso_controller",
            "/torso_controller/follow_joint_trajectory"
        )
        self._action_name = rospy.get_param("~action_name", "execute_door_opening")

    def _init_publishers(self):
        self._cmd_vel_pub = rospy.Publisher(
            "/mobile_base_controller/cmd_vel", Twist, queue_size=1
        )

    def _init_subscribers(self):
        self._current_odom = None
        self._odom_sub = rospy.Subscriber(
            "/mobile_base_controller/odom", Odometry, self._odom_callback, queue_size=1
        )
        self._ee_target_path = None
        self._ee_path_sub = rospy.Subscriber(
            "/door_plan/ee_target_path", Path, self._ee_path_callback, queue_size=1
        )
        self._current_joint_state = None
        self._joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self._joint_state_callback, queue_size=1
        )

    def _init_tf(self):
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

    def _init_arm_client(self):
        self._arm_client_right = actionlib.SimpleActionClient(
            self._arm_controller_right, FollowJointTrajectoryAction
        )
        self._arm_client_left = actionlib.SimpleActionClient(
            self._arm_controller_left, FollowJointTrajectoryAction
        )
        self._torso_client = actionlib.SimpleActionClient(
            self._torso_controller, FollowJointTrajectoryAction
        )
        self._arm_client = self._arm_client_right

        for name, client in [
            (self._arm_controller_right, self._arm_client_right),
            (self._arm_controller_left,  self._arm_client_left),
            (self._torso_controller,     self._torso_client),
        ]:
            rospy.loginfo("Waiting for controller: %s", name)
            if not client.wait_for_server(rospy.Duration(10.0)):
                rospy.logwarn("Controller not available at startup: %s", name)
            else:
                rospy.loginfo("Connected to controller: %s", name)

    def _select_arm_client_for_joints(self, joint_names):
        """Pick right or left arm client based on joint name convention."""
        for jn in joint_names:
            if "left" in jn:
                return self._arm_client_left
        return self._arm_client_right

    def _init_action_server(self):
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ExecuteDoorOpeningAction,
            execute_cb=self._execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("Door execution server ready: %s", self._action_name)

    # ============================================================
    # Basic callbacks and helpers
    # ============================================================

    def _odom_callback(self, msg):
        self._current_odom = msg

    def _ee_path_callback(self, msg):
        self._ee_target_path = msg

    def _joint_state_callback(self, msg):
        self._current_joint_state = msg

    def _get_current_arm_joints(self, joint_names):
        if self._current_joint_state is None:
            return None
        js = self._current_joint_state
        try:
            return [js.position[js.name.index(n)] for n in joint_names]
        except ValueError as e:
            rospy.logwarn("[Execution] Joint not found in joint_state: %s", e)
            return None

    def _approach_arm_to_start(self, arm_traj, approach_time):
        """Move arm from current position to the first trajectory waypoint."""
        if not arm_traj.points:
            return True
        current_joints = self._get_current_arm_joints(arm_traj.joint_names)
        if current_joints is None:
            rospy.logwarn("[Execution] No joint state available — skipping arm pre-approach")
            return True

        first_pt = arm_traj.points[0]
        approach = JointTrajectory()
        approach.header.stamp = rospy.Time(0)
        approach.joint_names = arm_traj.joint_names

        pt0 = JointTrajectoryPoint()
        pt0.positions = current_joints
        pt0.velocities = [0.0] * len(current_joints)
        pt0.time_from_start = rospy.Duration(0.0)

        pt1 = JointTrajectoryPoint()
        pt1.positions = list(first_pt.positions)
        pt1.velocities = [0.0] * len(first_pt.positions)
        pt1.time_from_start = rospy.Duration(approach_time)

        approach.points = [pt0, pt1]
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = approach
        goal.goal_time_tolerance = rospy.Duration(5.0)

        arm_client = self._select_arm_client_for_joints(arm_traj.joint_names)
        arm_client.send_goal(goal)
        rospy.loginfo("[Execution] Arm pre-approach: %.1fs to start configuration", approach_time)

        TERMINAL = (
            actionlib.GoalStatus.SUCCEEDED,
            actionlib.GoalStatus.ABORTED,
            actionlib.GoalStatus.PREEMPTED,
            actionlib.GoalStatus.REJECTED,
        )
        deadline = rospy.Time.now() + rospy.Duration(approach_time + 10.0)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self._is_preempt_requested():
                arm_client.cancel_goal()
                return False
            if rospy.Time.now() > deadline:
                rospy.logwarn("[Execution] Arm pre-approach timed out")
                return False
            state = arm_client.get_state()
            if state in TERMINAL:
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("[Execution] Arm pre-approach completed")
                    return True
                rospy.logwarn("[Execution] Arm pre-approach failed: state=%d", state)
                return False
            rate.sleep()
        return False

    def _find_arm_traj_index(self, arm_traj, t_unscaled):
        """Return the arm trajectory waypoint index at unscaled time t_unscaled."""
        pts = arm_traj.points
        best = 0
        for k in range(len(pts)):
            if pts[k].time_from_start.to_sec() <= t_unscaled:
                best = k
            else:
                break
        return best

    def _broadcast_ee_target_tf(self, t_unscaled, arm_traj=None):
        """Broadcast the current EE target waypoint as a TF frame 'ee_target_current'.

        Uses the arm trajectory timestamps to map elapsed time to the correct dense
        ee_target_path index, so the cross moves continuously regardless of whether
        the ee_target_path has more poses than the sparse base path.
        """
        if self._ee_target_path is None:
            return
        poses = self._ee_target_path.poses
        if not poses:
            return

        if arm_traj is not None and arm_traj.points:
            idx = self._find_arm_traj_index(arm_traj, t_unscaled)
        else:
            idx = 0
        idx = min(idx, len(poses) - 1)
        p = poses[idx].pose

        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = self._frame_map
        ts.child_frame_id = "ee_target_current"
        ts.transform.translation.x = p.position.x
        ts.transform.translation.y = p.position.y
        ts.transform.translation.z = p.position.z
        ts.transform.rotation = p.orientation
        self._tf_broadcaster.sendTransform(ts)

    def _get_current_pose_map(self):
        """Get current base pose in map frame"""
        try:
            trans = self._tf_buffer.lookup_transform(
                self._frame_map,
                self._frame_base,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            pose = PoseStamped()
            pose.header.frame_id = self._frame_map
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            return pose

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:
            rospy.logwarn("TF lookup failed: %s", str(e))
            return None

    def _yaw_from_pose(self, pose_stamped):
        quat = pose_stamped.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        return euler[2]

    def _position_error(self, current_pose, target_pose):
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return dx, dy, distance

    def _orientation_error(self, current_pose, target_pose):
        current_yaw = self._yaw_from_pose(current_pose)
        target_yaw = self._yaw_from_pose(target_pose)

        angle_to_target = np.arctan2(
            target_pose.pose.position.y - current_pose.pose.position.y,
            target_pose.pose.position.x - current_pose.pose.position.x
        )
        heading_error = angle_wrap(angle_to_target - current_yaw)
        orientation_error = angle_wrap(target_yaw - current_yaw)

        return current_yaw, target_yaw, heading_error, orientation_error

    def _compute_velocity_command(self, current_pose, target_pose):
        """
        Compute cmd_vel to move from current to target pose.
        Returns: (linear_vel, angular_vel, distance, orientation_error)
        """
        dx, dy, distance = self._position_error(current_pose, target_pose)
        current_yaw, target_yaw, heading_error, orientation_error = self._orientation_error(
            current_pose, target_pose
        )

        linear_vel = self._kp_linear * distance

        if distance > self._position_tolerance * 2:
            angular_vel = self._kp_angular * heading_error
        else:
            angular_vel = self._kp_angular * orientation_error

        linear_vel = np.clip(
            linear_vel, -self._max_linear_vel, self._max_linear_vel
        )
        angular_vel = np.clip(
            angular_vel, -self._max_angular_vel, self._max_angular_vel
        )

        return linear_vel, angular_vel, distance, orientation_error

    def _stop_base(self):
        """Stop base motion"""
        cmd = Twist()
        self._cmd_vel_pub.publish(cmd)

    def _cancel_all_motion(self):
        self._stop_base()
        self._arm_client_right.cancel_all_goals()
        self._arm_client_left.cancel_all_goals()
        self._torso_client.cancel_all_goals()

    def _is_preempt_requested(self):
        return self._as.is_preempt_requested()

    # ============================================================
    # Goal validation / setup
    # ============================================================

    def _make_result(self, success=False, message=""):
        result = ExecuteDoorOpeningResult()
        result.success = bool(success)
        result.message = str(message)
        return result

    def _validate_goal(self, goal):
        result = self._make_result(success=False)

        if not goal.base_path.poses:
            result.message = "Empty base path"
            return False, result

        if len(goal.base_times) == 0:
            result.message = "Empty base_times — planning server must provide base_times"
            return False, result

        if len(goal.base_times) != len(goal.base_path.poses):
            result.message = "base_times length (%d) != base_path poses (%d)" % (
                len(goal.base_times), len(goal.base_path.poses))
            return False, result

        return True, result

    def _clamp_velocity_scaling(self, velocity_scaling):
        if velocity_scaling <= 0:
            velocity_scaling = 1.0
        return float(np.clip(velocity_scaling, 0.1, 1.0))

    def _has_arm_trajectory(self, arm_traj):
        return bool(arm_traj.points and len(arm_traj.points) > 0)

    def _copy_scaled_trajectory_point(self, point, velocity_scaling):
        scaled_point = point
        scaled_point.time_from_start = rospy.Duration(
            point.time_from_start.to_sec() / velocity_scaling
        )
        return scaled_point

    def _split_trajectory(self, traj, joint_names_subset):
        """Extract a sub-trajectory containing only the given joints."""
        indices = [i for i, n in enumerate(traj.joint_names) if n in joint_names_subset]
        if not indices:
            return None
        sub = JointTrajectory()
        sub.header = traj.header
        sub.joint_names = [traj.joint_names[i] for i in indices]
        for pt in traj.points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = [pt.positions[i] for i in indices]
            new_pt.time_from_start = pt.time_from_start
            sub.points.append(new_pt)
        return sub

    def _scale_arm_trajectory(self, arm_traj, velocity_scaling):
        scaled = JointTrajectory()
        scaled.header = arm_traj.header
        scaled.header.stamp = rospy.Time(0)
        scaled.joint_names = arm_traj.joint_names

        pts = arm_traj.points
        n = len(pts)
        scaled.points = []

        for i, pt in enumerate(pts):
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions)
            new_pt.time_from_start = rospy.Duration(pt.time_from_start.to_sec() / velocity_scaling)

            n_joints = len(pt.positions)
            vels = []
            for j in range(n_joints):
                if i == 0 or i == n - 1:
                    vels.append(0.0)
                else:
                    t_prev = pts[i - 1].time_from_start.to_sec() / velocity_scaling
                    t_next = pts[i + 1].time_from_start.to_sec() / velocity_scaling
                    dt = t_next - t_prev
                    if dt < 1e-9:
                        vels.append(0.0)
                    else:
                        vels.append((pts[i + 1].positions[j] - pts[i - 1].positions[j]) / dt)
            new_pt.velocities = vels
            scaled.points.append(new_pt)

        return scaled

    def _interpolate_arm_joints(self, arm_traj, t_unscaled):
        """
        Linearly interpolate arm joint positions at unscaled planning time t_unscaled.
        Uses arm_traj's own time_from_start values (which match base_times).
        """
        pts = arm_traj.points
        n = len(pts)
        if n == 0:
            return []
        if t_unscaled <= pts[0].time_from_start.to_sec():
            return list(pts[0].positions)
        if t_unscaled >= pts[-1].time_from_start.to_sec():
            return list(pts[-1].positions)
        for i in range(n - 1):
            ta = pts[i].time_from_start.to_sec()
            tb = pts[i + 1].time_from_start.to_sec()
            if ta <= t_unscaled <= tb:
                dt = tb - ta
                if dt < 1e-9:
                    return list(pts[i].positions)
                frac = (t_unscaled - ta) / dt
                return [
                    float(pts[i].positions[k]) + frac * (float(pts[i + 1].positions[k]) - float(pts[i].positions[k]))
                    for k in range(len(pts[i].positions))
                ]
        return list(pts[-1].positions)

    def _send_arm_from_waypoint(self, arm_traj, from_idx, base_times, velocity_scaling, t_start):
        """
        Send arm trajectory from waypoint from_idx to the end.
        """
        n = len(arm_traj.points)
        if from_idx >= n - 1:
            return

        t_elapsed = (rospy.Time.now() - t_start).to_sec()
        t_unscaled = t_elapsed * velocity_scaling
        current_q = self._interpolate_arm_joints(arm_traj, t_unscaled)

        t0 = base_times[from_idx] if from_idx < len(base_times) else base_times[-1]
        pts = arm_traj.points

        all_pos = [current_q if current_q else list(pts[from_idx].positions)]
        all_t = [0.0]

        for global_j in range(from_idx, n):
            t_raw = base_times[global_j] if global_j < len(base_times) else base_times[-1]
            dt_scaled = (t_raw - t0) / velocity_scaling
            if dt_scaled <= 1e-6:
                continue
            all_pos.append(list(pts[global_j].positions))
            all_t.append(dt_scaled)

        if len(all_t) < 2:
            return

        m = len(all_t)
        n_joints = len(all_pos[0])

        all_vel = []
        for j in range(m):
            vels = []
            for k in range(n_joints):
                if j == 0 or j == m - 1:
                    vels.append(0.0)
                else:
                    ddt = all_t[j + 1] - all_t[j - 1]
                    if ddt < 1e-9:
                        vels.append(0.0)
                    else:
                        vels.append((all_pos[j + 1][k] - all_pos[j - 1][k]) / ddt)
            all_vel.append(vels)

        sub = JointTrajectory()
        sub.header.stamp = rospy.Time(0)
        sub.joint_names = arm_traj.joint_names
        for j in range(m):
            pt = JointTrajectoryPoint()
            pt.positions = all_pos[j]
            pt.velocities = all_vel[j]
            pt.time_from_start = rospy.Duration(all_t[j])
            sub.points.append(pt)

        torso_joints = set(jn for jn in sub.joint_names if "torso" in jn)
        arm_joints_set = set(jn for jn in sub.joint_names if jn not in torso_joints)

        arm_sub = self._split_trajectory(sub, arm_joints_set)
        torso_sub = self._split_trajectory(sub, torso_joints)

        if arm_sub:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = arm_sub
            self._select_arm_client_for_joints(arm_sub.joint_names).send_goal(goal)

        if torso_sub:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = torso_sub
            self._torso_client.send_goal(goal)

        rospy.logdebug(
            "[ArmSync] Sent arm segment from WP %d, %d points remaining",
            from_idx, m
        )

    # ============================================================
    # Feedback
    # ============================================================

    def _publish_feedback(self, stage, progress, current_waypoint, total_waypoints):
        fb = ExecuteDoorOpeningFeedback()
        fb.stage = stage
        fb.progress = progress
        fb.current_waypoint = current_waypoint
        fb.total_waypoints = total_waypoints
        self._as.publish_feedback(fb)

    # ============================================================
    # Time-synchronized execution
    # ============================================================

    def _waypoint_reached(self, distance, angle_error):
        return (
            distance < self._position_tolerance and
            abs(angle_error) < self._angle_tolerance
        )

    def _make_cmd_vel(self, linear_vel, angular_vel, velocity_scaling):
        cmd = Twist()
        cmd.linear.x = linear_vel * velocity_scaling
        cmd.angular.z = angular_vel * velocity_scaling
        return cmd

    def _find_target_index(self, base_times, t_unscaled):
        """Return the first waypoint index at or ahead of t_unscaled."""
        for i, t in enumerate(base_times):
            if t >= t_unscaled:
                return i
        return len(base_times) - 1

    def _unwrap_joint_trajectory(self, traj):
        from copy import deepcopy
        import math

        if len(traj.points) < 2:
            return traj

        out = deepcopy(traj)
        n_joints = len(out.points[0].positions)

        for i in range(1, len(out.points)):
            prev = list(out.points[i - 1].positions)
            curr = list(out.points[i].positions)
            fixed = list(curr)
            for j in range(n_joints):
                diff = curr[j] - prev[j]
                diff_wrapped = (diff + math.pi) % (2.0 * math.pi) - math.pi
                fixed[j] = prev[j] + diff_wrapped
            out.points[i].positions = fixed

        return out

    def _resample_arm_trajectory(self, traj, subdivisions=4):
        """
        Insert linearly-interpolated joint-space points between each pair of
        existing IK waypoints.
        """
        from copy import deepcopy

        if len(traj.points) < 2:
            return traj

        n_joints = len(traj.points[0].positions)
        new_points = [deepcopy(traj.points[0])]

        for i in range(1, len(traj.points)):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            t0 = p0.time_from_start.to_sec()
            t1 = p1.time_from_start.to_sec()

            for k in range(1, subdivisions + 1):
                frac = float(k) / subdivisions
                pt = JointTrajectoryPoint()
                pt.positions = [
                    float(p0.positions[j]) + frac * (float(p1.positions[j]) - float(p0.positions[j]))
                    for j in range(n_joints)
                ]
                pt.time_from_start = rospy.Duration(t0 + frac * (t1 - t0))
                new_points.append(pt)

        out = deepcopy(traj)
        out.points = new_points
        rospy.loginfo(
            "[ArmPrep] Resampled trajectory: %d -> %d points (subdivisions=%d)",
            len(traj.points), len(new_points), subdivisions
        )
        return out

    def _send_arm_trajectory_once(self, arm_traj, velocity_scaling):
        """
        Prepare and send the arm trajectory once:
          1. Unwrap joints to eliminate IK branch-flip spin-arounds.
          2. Resample to add dense intermediate points (smoother Cartesian path).
          3. Scale by velocity_scaling and compute central-difference velocities.
          4. Split arm/torso joints and send to their controllers.
        Both arm and base use the same velocity_scaling so they run in sync.
        """
        traj = self._resample_arm_trajectory(arm_traj, subdivisions=4)
        scaled = self._scale_arm_trajectory(traj, velocity_scaling)

        torso_joints = set(jn for jn in scaled.joint_names if "torso" in jn)
        arm_joints_set = set(jn for jn in scaled.joint_names if jn not in torso_joints)

        arm_sub = self._split_trajectory(scaled, arm_joints_set)
        torso_sub = self._split_trajectory(scaled, torso_joints)

        if arm_sub:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = arm_sub

            goal.path_tolerance = [
                JointTolerance(name=n, position=0.3) for n in arm_sub.joint_names
            ]
            goal.goal_tolerance = [
                JointTolerance(name=n, position=0.05) for n in arm_sub.joint_names
            ]
            goal.goal_time_tolerance = rospy.Duration(3.0)
            self._arm_client = self._select_arm_client_for_joints(arm_sub.joint_names)
            self._arm_client.send_goal(goal)
            rospy.loginfo("[Execution] Arm trajectory sent: %d points, duration=%.2fs",
                          len(arm_sub.points),
                          arm_sub.points[-1].time_from_start.to_sec() if arm_sub.points else 0.0)

        if torso_sub:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = torso_sub
            goal.path_tolerance = [
                JointTolerance(name=n, position=0.3) for n in torso_sub.joint_names
            ]
            goal.goal_tolerance = [
                JointTolerance(name=n, position=0.05) for n in torso_sub.joint_names
            ]
            goal.goal_time_tolerance = rospy.Duration(3.0)
            self._torso_client.send_goal(goal)
            rospy.loginfo("[Execution] Torso trajectory sent: %d points", len(torso_sub.points))

    def _execute_time_synchronized(self, base_path, base_times, velocity_scaling, t_start, arm_traj=None):
        """
        Time-based base execution loop.
        """
        rate = rospy.Rate(self._control_rate)
        total_scaled_duration = base_times[-1] / max(velocity_scaling, 1e-6)
        deadline = t_start + rospy.Duration(total_scaled_duration * 3.0 + 10.0)
        total_waypoints = len(base_path.poses)
        final_pose = base_path.poses[-1]

        rospy.loginfo(
            "[Execution] Base control: %d waypoints, %.2fs planned, scaling=%.2f",
            total_waypoints, base_times[-1], velocity_scaling
        )

        while not rospy.is_shutdown():
            if self._is_preempt_requested():
                self._stop_base()
                return False, "preempted"

            if rospy.Time.now() > deadline:
                self._stop_base()
                return False, "Execution deadline exceeded"

            t_elapsed = (rospy.Time.now() - t_start).to_sec()
            t_unscaled = t_elapsed * velocity_scaling

            current_pose = self._get_current_pose_map()
            if current_pose is None:
                rospy.logwarn_throttle(1.0, "[Execution] Cannot get current pose")
                rate.sleep()
                continue

            if t_unscaled >= base_times[-1]:
                _, _, dist = self._position_error(current_pose, final_pose)
                _, _, _, angle_err = self._orientation_error(current_pose, final_pose)
                if self._waypoint_reached(dist, angle_err):
                    self._stop_base()
                    rospy.loginfo("[Execution] Final pose reached (dist=%.3fm)", dist)
                    return True, ""
                target_pose = final_pose
                target_idx = total_waypoints - 1
            else:
                target_idx = self._find_target_index(base_times, t_unscaled)
                target_pose = base_path.poses[target_idx]

                _, _, lag_m = self._position_error(current_pose, target_pose)
                if lag_m > self._max_base_lag_m:
                    rospy.logerr(
                        "[Execution] Base lag %.3fm exceeds limit %.3fm — aborting",
                        lag_m, self._max_base_lag_m
                    )
                    self._cancel_all_motion()
                    return False, "Base lag %.2fm exceeded limit %.2fm" % (lag_m, self._max_base_lag_m)

            self._broadcast_ee_target_tf(t_unscaled, arm_traj=arm_traj)

            linear_vel, angular_vel, _, _ = self._compute_velocity_command(
                current_pose, target_pose
            )
            cmd = self._make_cmd_vel(linear_vel, angular_vel, velocity_scaling)
            self._cmd_vel_pub.publish(cmd)

            progress = float(min(1.0, t_elapsed / max(total_scaled_duration, 1e-6)))
            self._publish_feedback("executing", progress, target_idx + 1, total_waypoints)

            rate.sleep()

        self._stop_base()
        return False, "ROS shutdown"

    # ============================================================
    # Arm completion
    # ============================================================

    def _wait_for_arm_completion(self, arm_start_time, arm_traj=None, velocity_scaling=1.0):
        if arm_start_time is None:
            return True, ""

        rospy.loginfo("Waiting for arm/torso trajectories to complete...")

        TERMINAL_STATES = (
            actionlib.GoalStatus.SUCCEEDED,
            actionlib.GoalStatus.ABORTED,
            actionlib.GoalStatus.PREEMPTED,
            actionlib.GoalStatus.REJECTED,
            actionlib.GoalStatus.LOST,
        )

        rate = rospy.Rate(20.0)
        deadline = rospy.Time.now() + rospy.Duration(60.0)

        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                rospy.logwarn("[Execution] Arm completion timeout")
                return False, "arm execution timeout"

            if arm_traj is not None:
                t_elapsed = (rospy.Time.now() - arm_start_time).to_sec()
                t_unscaled = t_elapsed * velocity_scaling
                self._broadcast_ee_target_tf(t_unscaled, arm_traj=arm_traj)

            arm_done = self._arm_client.get_state() in TERMINAL_STATES
            torso_done = self._torso_client.get_state() in TERMINAL_STATES

            if arm_done and torso_done:
                break

            rate.sleep()

        arm_state = self._arm_client.get_state()
        torso_state = self._torso_client.get_state()

        rospy.loginfo(
            "Arm/torso completed in %.2fs — arm_state=%d torso_state=%d",
            (rospy.Time.now() - arm_start_time).to_sec(), arm_state, torso_state
        )

        if arm_state not in (actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.LOST):
            return False, "arm execution failed: state={}".format(arm_state)
        if torso_state not in (actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.LOST):
            return False, "torso execution failed: state={}".format(torso_state)

        return True, ""

    # ============================================================
    # Action callback
    # ============================================================

    def _execute_cb(self, goal):
        """Execute door opening with time-synchronized base and arm motion."""
        rospy.loginfo("=== Door Execution Server: Starting time-synchronized execution ===")

        valid, result = self._validate_goal(goal)
        if not valid:
            self._as.set_aborted(result)
            return

        base_path = goal.base_path
        base_times = list(goal.base_times)
        arm_traj = goal.arm_trajectory
        velocity_scaling = self._clamp_velocity_scaling(goal.velocity_scaling)

        rospy.loginfo(
            "Base waypoints: %d, Total duration: %.2fs, Velocity scaling: %.2f",
            len(base_path.poses), base_times[-1], velocity_scaling
        )

        arm = arm_traj if self._has_arm_trajectory(arm_traj) else None

        # Move arm from tuck to pre-grasp position before synchronized execution.
        if arm is not None:
            approach_time = float(rospy.get_param("~arm_approach_time", 5.0))
            if not self._approach_arm_to_start(arm, approach_time):
                result.message = "Arm pre-approach failed"
                self._as.set_aborted(result)
                return

        t_start = rospy.Time.now()

        if arm is not None:
            self._send_arm_trajectory_once(arm, velocity_scaling)
        arm_start_time = t_start if arm is not None else None

        ok, message = self._execute_time_synchronized(
            base_path, base_times, velocity_scaling, t_start, arm_traj=arm
        )

        if not ok:
            self._cancel_all_motion()
            if message == "preempted":
                self._as.set_preempted()
            else:
                result.message = message
                self._as.set_aborted(result)
            return

        ok, message = self._wait_for_arm_completion(
            arm_start_time, arm_traj=arm, velocity_scaling=velocity_scaling
        )
        if not ok:
            self._cancel_all_motion()
            result.message = message
            self._as.set_aborted(result)
            return

        result.success = True
        result.message = "Door opening executed successfully"
        rospy.loginfo("=== Door execution completed successfully ===")
        self._as.set_succeeded(result)


def main():
    rospy.init_node("door_execution_server", anonymous=False)
    server = DoorExecutionServer()
    rospy.loginfo("Door execution server running...")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass