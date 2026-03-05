#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, SetBoolResponse
import tf.transformations as tft


def circular_std(angles_rad):
    """
    Circular standard deviation via mean resultant length.
    angles_rad: numpy array (N,)
    """
    if angles_rad.size <= 1:
        return 0.0

    s = np.sin(angles_rad).mean()
    c = np.cos(angles_rad).mean()
    R = float(np.hypot(s, c))
    R = min(max(R, 1e-12), 1.0)
    return float(np.sqrt(-2.0 * np.log(R)))


class DoorwayPoseNode(object):

    def __init__(self):

        self.hinge_topic = rospy.get_param("~hinge_topic", "/door/hinge_pose_map")
        self.handle_topic = rospy.get_param("~handle_topic", "/door/handle_pose_map")
        self.out_topic = rospy.get_param("~out_topic", "/door/doorway_pose_map")

        self.max_age_s = float(rospy.get_param("~max_age_s", 0.5))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.stamp_now = bool(rospy.get_param("~stamp_now", True))

        self.normal_offset_m = float(rospy.get_param("~normal_offset_m", 0.0))

        self.N = int(rospy.get_param("~stability_window", 10))
        self.pos_std_thresh_m = float(rospy.get_param("~pos_std_thresh_m", 0.02))
        self.yaw_std_thresh_deg = float(rospy.get_param("~yaw_std_thresh_deg", 2.0))
        self.yaw_std_thresh_rad = float(np.deg2rad(self.yaw_std_thresh_deg))

        self.freeze_on_stable = bool(rospy.get_param("~freeze_on_stable", True))
        self._freeze_requested = False
        self._frozen = False

        self.last_hinge = None
        self.last_handle = None

        # ring buffer: [x, y, yaw]
        self.samples = np.full((max(self.N, 2), 3), np.nan, dtype=np.float64)
        self.sample_count = 0
        self.sample_idx = 0

        self.frozen_pose = None

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.hinge_topic, PoseStamped, self._hinge_cb, queue_size=1)
        rospy.Subscriber(self.handle_topic, PoseStamped, self._handle_cb, queue_size=1)

        self.srv = rospy.Service("~freeze", SetBool, self._freeze_srv_cb)

        rospy.loginfo("doorway_pose_node(np): hinge=%s handle=%s out=%s",
                      self.hinge_topic, self.handle_topic, self.out_topic)
        rospy.loginfo("doorway_pose_node(np): service=%s",
                      rospy.resolve_name("~freeze"))

    # SERVICE
    def _freeze_srv_cb(self, req):
        if req.data:
            self._freeze_requested = True

            if self._frozen:
                return SetBoolResponse(True, "already_frozen")

            if self._is_stable():
                self._freeze_now()
                return SetBoolResponse(True, "frozen_now")

            if self.freeze_on_stable:
                return SetBoolResponse(True, "freeze_pending_until_stable")

            return SetBoolResponse(False, "not_stable")

        else:
            self._freeze_requested = False
            self._frozen = False
            self.frozen_pose = None
            return SetBoolResponse(True, "unfrozen")

    # CALLBACKS
    def _hinge_cb(self, msg):
        self.last_hinge = msg
        self._update()

    def _handle_cb(self, msg):
        self.last_handle = msg
        self._update()

    # HELPERS
    def _is_fresh(self, msg):
        if msg is None:
            return False
        if msg.header.stamp == rospy.Time():
            return True
        return (rospy.Time.now() - msg.header.stamp).to_sec() <= self.max_age_s

    def _compute_doorway(self):
        h = self.last_hinge.pose.position
        p = self.last_handle.pose.position

        dx = p.x - h.x
        dy = p.y - h.y

        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None

        yaw = math.atan2(dy, dx)

        cx = 0.5 * (h.x + p.x)
        cy = 0.5 * (h.y + p.y)
        cz = h.z

        if abs(self.normal_offset_m) > 1e-12:
            cx += self.normal_offset_m * math.cos(yaw + math.pi/2.0)
            cy += self.normal_offset_m * math.sin(yaw + math.pi/2.0)

        return cx, cy, cz, yaw

    def _push_sample(self, x, y, yaw):
        self.samples[self.sample_idx, 0] = x
        self.samples[self.sample_idx, 1] = y
        self.samples[self.sample_idx, 2] = yaw

        self.sample_idx = (self.sample_idx + 1) % self.samples.shape[0]
        self.sample_count = min(self.sample_count + 1, self.samples.shape[0])

    def _stable_window(self):
        if self.sample_count < self.N:
            return None

        buf = self.samples[:self.sample_count, :]
        if np.any(np.isnan(buf)):
            return None

        return buf

    def _is_stable(self):
        buf = self._stable_window()
        if buf is None:
            return False

        sx = float(buf[:, 0].std())
        sy = float(buf[:, 1].std())
        syaw = circular_std(buf[:, 2])

        return (sx <= self.pos_std_thresh_m and
                sy <= self.pos_std_thresh_m and
                syaw <= self.yaw_std_thresh_rad)

    def _freeze_now(self):
        if self.frozen_pose is not None:
            self._frozen = True
            self._freeze_requested = False

    def _make_pose_msg(self, cx, cy, cz, yaw):
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)

        out = PoseStamped()
        out.header.frame_id = self.frame_id if self.frame_id else self.last_hinge.header.frame_id
        out.header.stamp = rospy.Time.now() if self.stamp_now else self.last_hinge.header.stamp

        out.pose.position.x = cx
        out.pose.position.y = cy
        out.pose.position.z = cz
        out.pose.orientation.x = q[0]
        out.pose.orientation.y = q[1]
        out.pose.orientation.z = q[2]
        out.pose.orientation.w = q[3]

        return out

    # MAIN UPDATE
    def _update(self):

        if self._frozen:
            if self.frozen_pose is not None:
                self.pub.publish(self.frozen_pose)
            return

        if not (self._is_fresh(self.last_hinge) and self._is_fresh(self.last_handle)):
            return

        res = self._compute_doorway()
        if res is None:
            return

        cx, cy, cz, yaw = res

        pose_msg = self._make_pose_msg(cx, cy, cz, yaw)
        self.frozen_pose = pose_msg

        self._push_sample(cx, cy, yaw)

        if self._freeze_requested and self.freeze_on_stable and self._is_stable():
            self._freeze_now()
            self.pub.publish(self.frozen_pose)
            rospy.loginfo("doorway_pose_node: frozen (stable)")
            return

        self.pub.publish(pose_msg)


def main():
    rospy.init_node("doorway_pose_node", anonymous=False)
    DoorwayPoseNode()
    rospy.spin()


if __name__ == "__main__":
    main()