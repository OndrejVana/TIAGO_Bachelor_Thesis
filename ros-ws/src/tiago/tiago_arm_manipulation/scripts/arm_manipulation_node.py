#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger, TriggerResponse

import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import moveit_commander
import moveit_msgs.msg

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
        wrist_rotation: Rotation around approach axis (default: -90.0 for palm-down grasp)
    
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

        # Params
        self.group_name = rospy.get_param("~move_group_name", "arm")
        self.ee_link = rospy.get_param("~ee_link", "")
        self.base_frame_fallback = rospy.get_param("~base_frame_fallback", "base_footprint")

        self.handle_topic = rospy.get_param("~handle_topic", "/door/handle_pose_map")

        self.default_pos_tol = rospy.get_param("~position_tolerance", 0.01)
        self.default_ori_tol = rospy.get_param("~orientation_tolerance", 0.05)
        self.default_vel = rospy.get_param("~velocity_scaling", 0.2)
        self.default_acc = rospy.get_param("~acceleration_scaling", 0.2)

        self.default_approach = rospy.get_param("~approach_distance", 0.12)
        self.default_lat = rospy.get_param("~lateral_offset", 0.0)
        self.default_vert = rospy.get_param("~vertical_offset", 0.0)

        self.named_home = rospy.get_param("~named_home", "home")
        self.named_stow = rospy.get_param("~named_stow", "stow")

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # MoveIt init
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=False)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        if self.ee_link:
            try:
                self.group.set_end_effector_link(self.ee_link)
            except Exception as e:
                rospy.logwarn("Failed to set ee_link '%s': %s", self.ee_link, str(e))

        self.planning_frame = self.group.get_planning_frame()
        if not self.planning_frame:
            self.planning_frame = rospy.get_param("~planning_frame", "")
        if not self.planning_frame:
            self.planning_frame = self.base_frame_fallback

        rospy.loginfo("Move group: %s | planning_frame: %s | ee_link: %s",
                      self.group_name, self.planning_frame, self.group.get_end_effector_link())

        # Handle subscription
        self.last_handle = None
        self.sub_handle = rospy.Subscriber(self.handle_topic, PoseStamped, self._on_handle, queue_size=1)

        # Services
        self.srv_pose = rospy.Service("~move_to_pose", MoveToPose, self._srv_move_to_pose)
        self.srv_named = rospy.Service("~move_to_named", MoveToNamed, self._srv_move_to_named)
        self.srv_pregrasp = rospy.Service("~door_pregrasp", DoorPregrasp, self._srv_door_pregrasp)

        # Convenience triggers
        self.srv_home = rospy.Service("~home", Trigger, self._srv_home)
        self.srv_stow = rospy.Service("~stow", Trigger, self._srv_stow)

        # Gripper
        self.gripper_client = None
        self._init_gripper_client()

        self.srv_open = rospy.Service("~gripper_open", Trigger, self._srv_gripper_open)
        self.srv_close = rospy.Service("~gripper_close", Trigger, self._srv_gripper_close)

        rospy.loginfo("tiago_arm_manipulation ready.")

    def _init_gripper_client(self):
        action_name = "/hand_controller/follow_joint_trajectory"
        self.gripper_client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for gripper action server: %s ...", action_name)
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Gripper action server not reachable: %s", action_name)
            self.gripper_client = None
        else:
            rospy.loginfo("Gripper action server connected: %s", action_name)

    def _on_handle(self, msg):
        self.last_handle = msg

    def _to_planning_frame(self, pose_stamped, timeout=0.5):
        if pose_stamped.header.frame_id == self.planning_frame:
            return pose_stamped
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.planning_frame,
                pose_stamped.header.frame_id,
                pose_stamped.header.stamp if pose_stamped.header.stamp != rospy.Time(0) else rospy.Time.now(),
                rospy.Duration(timeout)
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
        ok = True

        if isinstance(plan, tuple) and len(plan) >= 2:
            ok = bool(plan[0])
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

    # Services
    def _srv_move_to_pose(self, req):
        try:
            pos_tol = req.position_tolerance if req.position_tolerance > 0 else self.default_pos_tol
            ori_tol = req.orientation_tolerance if req.orientation_tolerance > 0 else self.default_ori_tol
            vel = req.velocity_scaling if req.velocity_scaling > 0 else self.default_vel
            acc = req.acceleration_scaling if req.acceleration_scaling > 0 else self.default_acc

            self._configure_group(pos_tol, ori_tol, vel, acc)

            target = self._to_planning_frame(req.target)
            self.group.set_pose_target(target)

            ok, msg = self._plan_and_maybe_execute(req.execute)
            return MoveToPoseResponse(ok=ok, message=msg)
        except Exception as e:
            return MoveToPoseResponse(ok=False, message=str(e))

    def _srv_move_to_named(self, req):
        try:
            vel = req.velocity_scaling if req.velocity_scaling > 0 else self.default_vel
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
            execute=True
        ))
        return TriggerResponse(success=resp.ok, message=resp.message)

    def _srv_stow(self, _req):
        resp = self._srv_move_to_named(MoveToNamed._request_class(
            name=self.named_stow,
            velocity_scaling=self.default_vel,
            acceleration_scaling=self.default_acc,
            execute=True
        ))
        return TriggerResponse(success=resp.ok, message=resp.message)

    def _compute_pregrasp_from_handle(self, handle_base, approach_distance, lateral_offset, vertical_offset):
        """
        Compute a simple pregrasp target:
          - position: offset from handle along -X of base frame by approach_distance
          - orientation: face the handle with gripper rotated 90° for sideways grasp
        """
        hx = handle_base.pose.position.x
        hy = handle_base.pose.position.y
        hz = handle_base.pose.position.z

        yaw = math.atan2(hy, hx)

        tgt = PoseStamped()
        tgt.header.stamp = rospy.Time.now()
        tgt.header.frame_id = self.planning_frame

        dx = math.cos(yaw)
        dy = math.sin(yaw)

        tgt.pose.position.x = hx - approach_distance * dx
        tgt.pose.position.y = hy - approach_distance * dy + lateral_offset
        tgt.pose.position.z = hz + vertical_offset
        tgt.pose.orientation = quat_for_handle_grasp(yaw)
        return tgt

    def _srv_door_pregrasp(self, req):
        try:
            if req.use_latest_handle:
                if self.last_handle is None:
                    return DoorPregraspResponse(ok=False, message="No /door/handle_pose_map received yet",
                                                planned_target_base=PoseStamped())
                handle = self.last_handle
            else:
                handle = req.handle_override

            approach = req.approach_distance if req.approach_distance > 0 else self.default_approach
            lat = req.lateral_offset
            vert = req.vertical_offset

            vel = req.velocity_scaling if req.velocity_scaling > 0 else self.default_vel
            acc = req.acceleration_scaling if req.acceleration_scaling > 0 else self.default_acc

            # Transform handle into planning frame
            handle_base = self._to_planning_frame(handle)

            target = self._compute_pregrasp_from_handle(handle_base, approach, lat, vert)

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
            goal = FollowJointTrajectoryGoal()
            trajectory = JointTrajectory()
            trajectory.joint_names = ['hand_thumb_joint', 'hand_index_joint', 
                                     'hand_mrl_joint']
            
            point = JointTrajectoryPoint()
            if open_gripper:
                # Open position
                point.positions = [0.0, 0.0, 0.0]
            else:
                # Close position
                point.positions = [2.0, 2.0, 2.0]
            
            point.time_from_start = rospy.Duration(2.0)
            trajectory.points = [point]
            goal.trajectory = trajectory
            
            self.gripper_client.send_goal(goal)
            finished = self.gripper_client.wait_for_result(rospy.Duration(5.0))
            if not finished:
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