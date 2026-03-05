#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

import tf.transformations as tft


class DoorApproachGoalGenerator(object):
    def __init__(self):
        self.plane_topic = rospy.get_param("~plane_topic", "/door/plane_map")
        self.handle_topic = rospy.get_param("~handle_topic", "/door/handle_pose_map")
        self.goal_topic = rospy.get_param("~goal_topic", "/tiago_move_base_control/goal")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/tiago_move_base_control/generate_goal")

        self.stand_off_m = float(rospy.get_param("~stand_off_m", 0.75))
        self.lateral_offset_m = float(rospy.get_param("~lateral_offset_m", 0.0))
        self.use_handle_as_target = bool(rospy.get_param("~use_handle_as_target", True))
        self.face_door = bool(rospy.get_param("~face_door", True))
        self.normal_sign = int(rospy.get_param("~normal_sign", -1))
        self.min_update_period_s = float(rospy.get_param("~min_update_period_s", 0.5))
        self.continuous = bool(rospy.get_param("~continuous", False))
        self.min_normal_norm = float(rospy.get_param("~min_normal_norm", 1e-6))

        self.pub_goal = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.pub_dbg = rospy.Publisher("~debug", String, queue_size=10)

        self.sub_plane = rospy.Subscriber(self.plane_topic, PoseStamped, self._cb_plane, queue_size=1)
        self.sub_handle = rospy.Subscriber(self.handle_topic, PoseStamped, self._cb_handle, queue_size=1)
        self.sub_trigger = rospy.Subscriber(self.trigger_topic, Empty, self._cb_trigger, queue_size=10)

        self.last_plane = None
        self.last_handle = None
        self.last_pub_time = rospy.Time(0)

        rospy.loginfo("door_approach_goal_generator_node: plane=%s handle=%s goal=%s trigger=%s",
                      self.plane_topic, self.handle_topic, self.goal_topic, self.trigger_topic)

    def _cb_plane(self, msg):
        self.last_plane = msg
        if self.continuous:
            self._maybe_publish()

    def _cb_handle(self, msg):
        self.last_handle = msg
        if self.continuous:
            self._maybe_publish()

    def _cb_trigger(self, _msg):
        self._publish_once(force=True)

    def _maybe_publish(self):
        now = rospy.Time.now()
        if (now - self.last_pub_time).to_sec() < self.min_update_period_s:
            return
        self._publish_once(force=False)

    def _extract_axes_from_pose(self, pose):
        """
        Returns (x_axis, y_axis, z_axis) of pose orientation in the same frame.
        Z axis is used as door normal.
        """
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R = tft.quaternion_matrix(q)[0:3, 0:3]
        x = R[:, 0]
        y = R[:, 1]
        z = R[:, 2]
        return x, y, z

    def _yaw_from_vector_xy(self, v):
        """
        v: 2D vector (x,y). Returns yaw in radians.
        """
        return math.atan2(v[1], v[0])

    def _publish_once(self, force=False):
        if self.last_plane is None:
            self.pub_dbg.publish(String(data="no_plane"))
            return

        plane = self.last_plane

        if self.use_handle_as_target and (self.last_handle is not None):
            target = self.last_handle
        else:
            target = plane

        frame_plane = plane.header.frame_id
        frame_target = target.header.frame_id
        if not frame_plane or not frame_target:
            self.pub_dbg.publish(String(data="missing_frame_id"))
            return
        if frame_plane != frame_target:
            self.pub_dbg.publish(String(data="frame_mismatch plane=%s target=%s" % (frame_plane, frame_target)))
            return

        x_axis, y_axis, n = self._extract_axes_from_pose(plane.pose)
        n_norm = np.linalg.norm(n)
        if n_norm < self.min_normal_norm:
            self.pub_dbg.publish(String(data="normal_invalid"))
            return
        n = n / n_norm

        x_axis = x_axis / max(1e-12, np.linalg.norm(x_axis))

        p_t = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z], dtype=np.float64)

        p_goal = p_t + float(self.normal_sign) * self.stand_off_m * n + self.lateral_offset_m * x_axis

        p_goal[2] = 0.0

        if self.face_door:
            heading = -n[0:2]
        else:
            heading = n[0:2]

        h_norm = np.linalg.norm(heading)
        if h_norm < 1e-9:
            self.pub_dbg.publish(String(data="heading_invalid"))
            return
        heading = heading / h_norm

        yaw = self._yaw_from_vector_xy(heading)
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)

        goal = PoseStamped()
        goal.header.frame_id = frame_target
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(p_goal[0])
        goal.pose.position.y = float(p_goal[1])
        goal.pose.position.z = float(p_goal[2])
        goal.pose.orientation.x = float(q[0])
        goal.pose.orientation.y = float(q[1])
        goal.pose.orientation.z = float(q[2])
        goal.pose.orientation.w = float(q[3])

        # Rate limit unless forced
        now = rospy.Time.now()
        if (not force) and (now - self.last_pub_time).to_sec() < self.min_update_period_s:
            return

        self.pub_goal.publish(goal)
        self.last_pub_time = now

        self.pub_dbg.publish(String(
            data="published goal (stand_off=%.2f lateral=%.2f use_handle=%s)" %
                 (self.stand_off_m, self.lateral_offset_m, str(self.use_handle_as_target).lower())
        ))


def main():
    rospy.init_node("door_approach_goal_generator_node", anonymous=False)
    _ = DoorApproachGoalGenerator()
    rospy.spin()


if __name__ == "__main__":
    main()