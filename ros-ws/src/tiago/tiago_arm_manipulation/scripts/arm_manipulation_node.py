#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import moveit_commander

from tiago_arm_manipulation.srv import MoveToPose, MoveToPoseResponse
from tiago_arm_manipulation.srv import MoveToNamed, MoveToNamedResponse
from tiago_arm_manipulation.srv import DoorPregrasp, DoorPregraspResponse


def quat_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def quat_for_handle_grasp(yaw, wrist_rotation=-math.pi / 2):
    """
    Create quaternion for grasping horizontal door handle.

    Args:
        yaw: Direction to face (toward handle)
        wrist_rotation: Rotation around approach axis (default: -90 deg for palm-down grasp)

    Returns:
        Quaternion for end-effector orientation
    """
    q_array = tft.quaternion_from_euler(wrist_rotation, 0.0, yaw)
    q = Quaternion()
    q.x = q_array[0]
    q.y = q_array[1]
    q.z = q_array[2]
    q.w = q_array[3]
    return q


class TiagoArmManipulationNode(object):
    def __init__(self):
        rospy.init_node("tiago_arm_manipulation", anonymous=False)

        self.base_frame_fallback = rospy.get_param("~base_frame_fallback", "base_footprint")
        self.handle_topic        = rospy.get_param("~handle_topic", "/door/handle_pose_map")
        hinge_side_topic         = rospy.get_param("~hinge_side_topic", "/door/hinge_side")

        self.default_pos_tol = rospy.get_param("~position_tolerance", 0.01)
        self.default_ori_tol = rospy.get_param("~orientation_tolerance", 0.05)
        self.default_vel     = rospy.get_param("~velocity_scaling", 0.2)
        self.default_acc     = rospy.get_param("~acceleration_scaling", 0.2)

        self.default_approach = rospy.get_param("~approach_distance", 0.12)
        self.default_lat      = rospy.get_param("~lateral_offset", 0.0)
        self.default_vert     = rospy.get_param("~vertical_offset", 0.0)

        self.named_home = rospy.get_param("~named_home", "home")
        self.named_stow = rospy.get_param("~named_stow", "stow")

        # Right arm: used when hinge_side == "left"
        group_name_right          = rospy.get_param("~move_group_name", "arm_right")
        ee_link_right             = rospy.get_param("~ee_link", "arm_right_7_link")
        gripper_ctrl_right        = rospy.get_param("~gripper_controller", "hand_right_controller")
        gripper_joints_right      = rospy.get_param(
            "~gripper_joint_names",
            ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_mrl_joint"])

        # Left arm: used when hinge_side == "right"
        group_name_left           = rospy.get_param("~move_group_name_left", "arm_left")
        ee_link_left              = rospy.get_param("~ee_link_left", "arm_left_7_link")
        gripper_ctrl_left         = rospy.get_param("~gripper_controller_left", "hand_left_controller")
        gripper_joints_left       = rospy.get_param(
            "~gripper_joint_names_left",
            ["hand_left_thumb_joint", "hand_left_index_joint", "hand_left_mrl_joint"])

        self.tf_buffer   = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=False)

        rospy.loginfo("Initialising MoveGroup '%s' (right arm)…", group_name_right)
        self._group_right = moveit_commander.MoveGroupCommander(group_name_right)
        if ee_link_right:
            try:
                self._group_right.set_end_effector_link(ee_link_right)
            except Exception as e:
                rospy.logwarn("Failed to set right ee_link '%s': %s", ee_link_right, str(e))

        rospy.loginfo("Initialising MoveGroup '%s' (left arm)…", group_name_left)
        self._group_left = moveit_commander.MoveGroupCommander(group_name_left)
        if ee_link_left:
            try:
                self._group_left.set_end_effector_link(ee_link_left)
            except Exception as e:
                rospy.logwarn("Failed to set left ee_link '%s': %s", ee_link_left, str(e))

        # Default to right arm; _on_hinge_side() switches this at runtime.
        self.group = self._group_right
        self._active_gripper_joint_names = list(gripper_joints_right)

        self.planning_frame = self.group.get_planning_frame() or self.base_frame_fallback
        rospy.loginfo(
            "Right arm: group=%s  ee=%s  planning_frame=%s",
            group_name_right,
            self._group_right.get_end_effector_link(),
            self.planning_frame,
        )
        rospy.loginfo(
            "Left arm:  group=%s  ee=%s",
            group_name_left,
            self._group_left.get_end_effector_link(),
        )

        self._gripper_joints_right = list(gripper_joints_right)
        self._gripper_joints_left  = list(gripper_joints_left)

        self._gripper_client_right = self._make_gripper_client(gripper_ctrl_right)
        self._gripper_client_left  = self._make_gripper_client(gripper_ctrl_left)
        self.gripper_client        = self._gripper_client_right  # matches active arm

        self.last_handle  = None
        rospy.Subscriber(self.handle_topic, PoseStamped, self._on_handle, queue_size=1)
        rospy.Subscriber(hinge_side_topic,  String,      self._on_hinge_side, queue_size=1)

        rospy.Service("~move_to_pose",   MoveToPose,   self._srv_move_to_pose)
        rospy.Service("~move_to_named",  MoveToNamed,  self._srv_move_to_named)
        rospy.Service("~door_pregrasp",  DoorPregrasp, self._srv_door_pregrasp)
        rospy.Service("~home",           Trigger,      self._srv_home)
        rospy.Service("~stow",           Trigger,      self._srv_stow)
        rospy.Service("~gripper_open",   Trigger,      self._srv_gripper_open)
        rospy.Service("~gripper_close",  Trigger,      self._srv_gripper_close)

        rospy.loginfo("tiago_arm_manipulation ready (waiting for /door/hinge_side to select arm).")

    def _make_gripper_client(self, controller_name):
        action = "/{}/follow_joint_trajectory".format(controller_name)
        client = actionlib.SimpleActionClient(action, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for gripper server: %s …", action)
        if not client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Gripper server not reachable: %s", action)
            return None
        rospy.loginfo("Gripper server connected: %s", action)
        return client

    def _on_handle(self, msg):
        self.last_handle = msg

    def _on_hinge_side(self, msg):
        """
        Switch the active arm based on hinge side:
          hinge_side == "left"  → door handle is on the RIGHT → use RIGHT arm
          hinge_side == "right" → door handle is on the LEFT  → use LEFT arm
        """
        side = str(msg.data).strip().lower()
        if side == "left":
            if self.group is not self._group_right:
                rospy.loginfo("[ArmManip] hinge_side=left → switching to RIGHT arm")
            self.group                       = self._group_right
            self.gripper_client              = self._gripper_client_right
            self._active_gripper_joint_names = self._gripper_joints_right
        elif side == "right":
            if self.group is not self._group_left:
                rospy.loginfo("[ArmManip] hinge_side=right → switching to LEFT arm")
            self.group                       = self._group_left
            self.gripper_client              = self._gripper_client_left
            self._active_gripper_joint_names = self._gripper_joints_left
        else:
            rospy.logwarn_throttle(10.0, "[ArmManip] Unknown hinge_side value: '%s'", side)

    def _to_planning_frame(self, pose_stamped, timeout=0.5):
        if pose_stamped.header.frame_id == self.planning_frame:
            return pose_stamped
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.planning_frame,
                pose_stamped.header.frame_id,
                pose_stamped.header.stamp if pose_stamped.header.stamp != rospy.Time(0)
                    else rospy.Time.now(),
                rospy.Duration(timeout),
            )
            return tf2_geometry_msgs.do_transform_pose(pose_stamped, tfm)
        except Exception as e:
            raise RuntimeError("TF transform failed (%s -> %s): %s" %
                               (pose_stamped.header.frame_id, self.planning_frame, str(e)))

    def _configure_group(self, pos_tol, ori_tol, vel, acc):
        self.group.set_goal_position_tolerance(pos_tol)
        self.group.set_goal_orientation_tolerance(ori_tol)
        self.group.set_max_velocity_scaling_factor(vel)
        self.group.set_max_acceleration_scaling_factor(acc)

    def _plan_and_maybe_execute(self, execute):
        plan = self.group.plan()
        traj = None
        ok   = True

        if isinstance(plan, tuple) and len(plan) >= 2:
            ok   = bool(plan[0])
            traj = plan[1]
        else:
            traj = plan

        if not ok:
            return False, "Planning failed"

        if execute:
            exec_ok = self.group.execute(traj, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            if not exec_ok:
                return False, "Execution failed"
        return True, "OK"

    def _srv_move_to_pose(self, req):
        try:
            pos_tol = req.position_tolerance    if req.position_tolerance    > 0 else self.default_pos_tol
            ori_tol = req.orientation_tolerance if req.orientation_tolerance > 0 else self.default_ori_tol
            vel     = req.velocity_scaling      if req.velocity_scaling      > 0 else self.default_vel
            acc     = req.acceleration_scaling  if req.acceleration_scaling  > 0 else self.default_acc

            self._configure_group(pos_tol, ori_tol, vel, acc)
            target = self._to_planning_frame(req.target)
            self.group.set_pose_target(target)

            ok, msg = self._plan_and_maybe_execute(req.execute)
            return MoveToPoseResponse(ok=ok, message=msg)
        except Exception as e:
            return MoveToPoseResponse(ok=False, message=str(e))

    def _srv_move_to_named(self, req):
        try:
            vel = req.velocity_scaling     if req.velocity_scaling     > 0 else self.default_vel
            acc = req.acceleration_scaling if req.acceleration_scaling > 0 else self.default_acc
            self._configure_group(self.default_pos_tol, self.default_ori_tol, vel, acc)
            self.group.set_named_target(req.name)
            ok, msg = self._plan_and_maybe_execute(req.execute)
            return MoveToNamedResponse(ok=ok, message=msg)
        except Exception as e:
            return MoveToNamedResponse(ok=False, message=str(e))

    def _srv_home(self, _req):
        resp = self._srv_move_to_named(MoveToNamed._request_class(
            name=self.named_home,
            velocity_scaling=self.default_vel,
            acceleration_scaling=self.default_acc,
            execute=True,
        ))
        return TriggerResponse(success=resp.ok, message=resp.message)

    def _srv_stow(self, _req):
        resp = self._srv_move_to_named(MoveToNamed._request_class(
            name=self.named_stow,
            velocity_scaling=self.default_vel,
            acceleration_scaling=self.default_acc,
            execute=True,
        ))
        return TriggerResponse(success=resp.ok, message=resp.message)

    def _compute_pregrasp_from_handle(self, handle_base, approach_distance, lateral_offset, vertical_offset):
        hx  = handle_base.pose.position.x
        hy  = handle_base.pose.position.y
        hz  = handle_base.pose.position.z
        yaw = math.atan2(hy, hx)

        tgt = PoseStamped()
        tgt.header.stamp    = rospy.Time.now()
        tgt.header.frame_id = self.planning_frame

        dx = math.cos(yaw)
        dy = math.sin(yaw)

        tgt.pose.position.x   = hx - approach_distance * dx
        tgt.pose.position.y   = hy - approach_distance * dy + lateral_offset
        tgt.pose.position.z   = hz + vertical_offset
        tgt.pose.orientation  = quat_for_handle_grasp(yaw)
        return tgt

    def _srv_door_pregrasp(self, req):
        try:
            if req.use_latest_handle:
                if self.last_handle is None:
                    return DoorPregraspResponse(
                        ok=False,
                        message="No /door/handle_pose_map received yet",
                        planned_target_base=PoseStamped(),
                    )
                handle = self.last_handle
            else:
                handle = req.handle_override

            approach = req.approach_distance  if req.approach_distance  > 0 else self.default_approach
            lat      = req.lateral_offset
            vert     = req.vertical_offset
            vel      = req.velocity_scaling      if req.velocity_scaling      > 0 else self.default_vel
            acc      = req.acceleration_scaling  if req.acceleration_scaling  > 0 else self.default_acc

            handle_base = self._to_planning_frame(handle)
            target      = self._compute_pregrasp_from_handle(handle_base, approach, lat, vert)

            self._configure_group(self.default_pos_tol, self.default_ori_tol, vel, acc)
            self.group.set_pose_target(target)

            ok, msg = self._plan_and_maybe_execute(req.execute)
            return DoorPregraspResponse(ok=ok, message=msg, planned_target_base=target)
        except Exception as e:
            return DoorPregraspResponse(ok=False, message=str(e), planned_target_base=PoseStamped())

    def _srv_gripper_open(self, _req):
        ok, msg = self._gripper_command(open_gripper=True)
        return TriggerResponse(success=ok, message=msg)

    def _srv_gripper_close(self, _req):
        ok, msg = self._gripper_command(open_gripper=False)
        return TriggerResponse(success=ok, message=msg)

    def _gripper_command(self, open_gripper):
        if self.gripper_client is None:
            return False, "Gripper action client not available"
        try:
            goal       = FollowJointTrajectoryGoal()
            trajectory = JointTrajectory()
            trajectory.joint_names = list(self._active_gripper_joint_names)

            point = JointTrajectoryPoint()
            point.positions       = [0.0, 0.0, 0.0] if open_gripper else [2.0, 2.0, 2.0]
            point.time_from_start = rospy.Duration(2.0)
            trajectory.points     = [point]
            goal.trajectory       = trajectory

            self.gripper_client.send_goal(goal)
            if not self.gripper_client.wait_for_result(rospy.Duration(5.0)):
                self.gripper_client.cancel_goal()
                return False, "Gripper action timeout"
            return True, "OK"
        except Exception as e:
            return False, str(e)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = TiagoArmManipulationNode()
    node.spin()