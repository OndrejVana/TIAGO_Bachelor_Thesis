#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import numpy as np
import rospy

from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_srvs.srv import SetBool, SetBoolResponse

import tf.transformations as tft


class DoorModelFromTagNode(object):
    def __init__(self):
        # Topics
        self.in_topic = rospy.get_param("~in_topic", "/door/tag_pose_map")

        self.out_handle_topic = rospy.get_param("~out_handle_topic", "/door/handle_pose_map")
        self.out_hinge_topic = rospy.get_param("~out_hinge_topic", "/door/hinge_pose_map")
        self.out_hinge_axis_topic = rospy.get_param("~out_hinge_axis_topic", "/door/hinge_axis_map")
        self.out_plane_topic = rospy.get_param("~out_plane_topic", "/door/plane_map")

        # Multi-tag selection
        self.active_tag_id_topic = rospy.get_param("~active_tag_id_topic", "/door/tag_id")
        self._active_tag_id = None

        # Optional forwarding for higher-level behavior
        self.forward_side = bool(rospy.get_param("~forward_side", True))
        self.forward_hinge_side = bool(rospy.get_param("~forward_hinge_side", True))
        self.forward_interaction = bool(rospy.get_param("~forward_interaction", True))
        self.side_topic_in = rospy.get_param("~side_topic_in", "/door/door_side")
        self.hinge_side_topic_in = rospy.get_param("~hinge_side_topic_in", "/door/hinge_side")
        self.interaction_topic_in = rospy.get_param("~interaction_topic_in", "/door/interaction")
        self.pub_side = rospy.Publisher("/door/door_side", String, queue_size=10) if self.forward_side else None
        self.pub_hinge_side = rospy.Publisher("/door/hinge_side", String, queue_size=10) if self.forward_hinge_side else None
        self.pub_interaction = rospy.Publisher("/door/interaction", String, queue_size=10) if self.forward_interaction else None
        self._last_side = ""
        self._last_hinge_side = ""
        self._last_interaction = ""

        if self.forward_side:
            rospy.Subscriber(self.side_topic_in, String, self._side_cb, queue_size=1)
        if self.forward_hinge_side:
            rospy.Subscriber(self.hinge_side_topic_in, String, self._hinge_side_cb, queue_size=1)
        if self.forward_interaction:
            rospy.Subscriber(self.interaction_topic_in, String, self._interaction_cb, queue_size=1)

        # Legacy defaults
        self.legacy_door_normal_axis_tag = self._parse_axis(rospy.get_param("~door_normal_axis_tag", "z"))
        self.legacy_hinge_axis_tag = self._parse_axis(rospy.get_param("~hinge_axis_tag", "y"))
        self.legacy_handle_offset_xyz = rospy.get_param("~handle_offset_xyz", [0.0, 0.0, 0.0])
        self.legacy_handle_offset_rpy = rospy.get_param("~handle_offset_rpy", [0.0, 0.0, 0.0])
        self.legacy_hinge_offset_xyz = rospy.get_param("~hinge_offset_xyz", [0.0, 0.0, 0.0])
        self.legacy_hinge_offset_rpy = rospy.get_param("~hinge_offset_rpy", [0.0, 0.0, 0.0])
        self.legacy_publish_hinge_orientation = bool(rospy.get_param("~publish_hinge_orientation", True))
        self.legacy_door_width_m = float(rospy.get_param("~door_width_m", 0.9))
        self.legacy_handle_radius_m = float(rospy.get_param("~handle_radius_m", 0.7))

        # Per-tag models
        raw_models = rospy.get_param("~models", {})
        self.models = {}
        for k, cfg in raw_models.items():
            try:
                tid = int(k)
            except Exception:
                continue
            self.models[tid] = self._parse_model_cfg(cfg)

        # Publishers/subscribers
        self.pub_handle = rospy.Publisher(self.out_handle_topic, PoseStamped, queue_size=10)
        self.pub_hinge = rospy.Publisher(self.out_hinge_topic, PoseStamped, queue_size=10)
        self.pub_hinge_axis = rospy.Publisher(self.out_hinge_axis_topic, Vector3Stamped, queue_size=10)
        self.pub_plane = rospy.Publisher(self.out_plane_topic, PoseStamped, queue_size=10)

        self.pub_dbg = rospy.Publisher("~debug", String, queue_size=10)
        self.sub = rospy.Subscriber(self.in_topic, PoseStamped, self.transform_callback, queue_size=1)

        if self.active_tag_id_topic:
            rospy.Subscriber(self.active_tag_id_topic, Int32, self._tag_id_cb, queue_size=1)

        rospy.loginfo("door_model_from_tag_node: in=%s", self.in_topic)
        rospy.loginfo("door_model_from_tag_node: active_tag_id_topic=%s", self.active_tag_id_topic)
        rospy.loginfo("door_model_from_tag_node: models=%s", str(sorted(self.models.keys())))
        
    # Selection

    def _tag_id_cb(self, msg):
        self._active_tag_id = int(msg.data)

    def _side_cb(self, msg):
        self._last_side = msg.data

    def _hinge_side_cb(self, msg):
        self._last_hinge_side = msg.data

    def _interaction_cb(self, msg):
        self._last_interaction = msg.data

    def _parse_model_cfg(self, cfg):
        door_normal_axis_tag = self._parse_axis(cfg.get("door_normal_axis_tag", "z"))
        hinge_axis_tag = self._parse_axis(cfg.get("hinge_axis_tag", "y"))

        handle_offset_xyz = cfg.get("handle_offset_xyz", [0.0, 0.0, 0.0])
        handle_offset_rpy = cfg.get("handle_offset_rpy", [0.0, 0.0, 0.0])

        hinge_offset_xyz = cfg.get("hinge_offset_xyz", [0.0, 0.0, 0.0])
        hinge_offset_rpy = cfg.get("hinge_offset_rpy", [0.0, 0.0, 0.0])

        publish_hinge_orientation = bool(cfg.get("publish_hinge_orientation", True))

        door_width_m = float(cfg.get("door_width_m", self.legacy_door_width_m))
        handle_radius_m = float(cfg.get("handle_radius_m", self.legacy_handle_radius_m))

        T_tag_handle = self._T_from_xyz_rpy(handle_offset_xyz, handle_offset_rpy)
        T_tag_hinge = self._T_from_xyz_rpy(hinge_offset_xyz, hinge_offset_rpy)

        return {
            "door_normal_axis_tag": door_normal_axis_tag,
            "hinge_axis_tag": hinge_axis_tag,
            "handle_offset_xyz": handle_offset_xyz,
            "handle_offset_rpy": handle_offset_rpy,
            "hinge_offset_xyz": hinge_offset_xyz,
            "hinge_offset_rpy": hinge_offset_rpy,
            "publish_hinge_orientation": publish_hinge_orientation,
            "door_width_m": door_width_m,
            "handle_radius_m": handle_radius_m,
            "T_tag_handle": T_tag_handle,
            "T_tag_hinge": T_tag_hinge,
        }

    def _get_active_model(self):
        if self._active_tag_id is not None and self._active_tag_id in self.models:
            return self.models[self._active_tag_id], self._active_tag_id

        if self._active_tag_id is None and len(self.models) == 1:
            tid = list(self.models.keys())[0]
            return self.models[tid], tid

        # Legacy fallback
        T_tag_handle = self._T_from_xyz_rpy(self.legacy_handle_offset_xyz, self.legacy_handle_offset_rpy)
        T_tag_hinge = self._T_from_xyz_rpy(self.legacy_hinge_offset_xyz, self.legacy_hinge_offset_rpy)
        legacy = {
            "door_normal_axis_tag": self.legacy_door_normal_axis_tag,
            "hinge_axis_tag": self.legacy_hinge_axis_tag,
            "publish_hinge_orientation": self.legacy_publish_hinge_orientation,
            "door_width_m": self.legacy_door_width_m,
            "handle_radius_m": self.legacy_handle_radius_m,
            "T_tag_handle": T_tag_handle,
            "T_tag_hinge": T_tag_hinge,
        }
        return legacy, -1

    # Math helpers

    def _vec_norm(self, v, eps=1e-12):
        v = np.asarray(v, dtype=np.float64).reshape(3)
        n = np.linalg.norm(v)
        if n < eps:
            return None
        return v / n

    def _parse_axis(self, axis_spec):
        if isinstance(axis_spec, str):
            s = axis_spec.strip().lower()
            sign = 1.0
            if s.startswith("-"):
                sign = -1.0
                s = s[1:]
            if s == "x":
                v = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            elif s == "y":
                v = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            elif s == "z":
                v = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            else:
                raise ValueError("Invalid axis string: %s" % axis_spec)
            return sign * v

        if isinstance(axis_spec, (list, tuple)) and len(axis_spec) == 3:
            v = self._vec_norm(axis_spec)
            if v is None:
                raise ValueError("Axis vector has near-zero norm: %s" % str(axis_spec))
            return v

        raise ValueError("Invalid axis spec: %s" % str(axis_spec))

    def _T_from_pose(self, pose):
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        t = [pose.position.x, pose.position.y, pose.position.z]
        T = tft.quaternion_matrix(q)
        T[0:3, 3] = np.array(t, dtype=np.float64)
        return T

    def _pose_from_T(self, T):
        q = tft.quaternion_from_matrix(T)  # x,y,z,w
        p = PoseStamped()
        p.pose.position.x = float(T[0, 3])
        p.pose.position.y = float(T[1, 3])
        p.pose.position.z = float(T[2, 3])
        p.pose.orientation.x = float(q[0])
        p.pose.orientation.y = float(q[1])
        p.pose.orientation.z = float(q[2])
        p.pose.orientation.w = float(q[3])
        return p

    def _T_from_xyz_rpy(self, xyz, rpy):
        q = tft.quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        T = tft.quaternion_matrix(q)
        T[0:3, 3] = np.array([float(xyz[0]), float(xyz[1]), float(xyz[2])], dtype=np.float64)
        return T

    def _quat_from_two_axes(self, z_axis, x_hint):
        z = self._vec_norm(z_axis)
        if z is None:
            return None

        xh = self._vec_norm(x_hint)
        if xh is None:
            xh = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        x = np.cross(xh, z)
        x = self._vec_norm(x)
        if x is None:
            x = self._vec_norm(np.cross(np.array([0.0, 1.0, 0.0], dtype=np.float64), z))
            if x is None:
                return None

        y = np.cross(z, x)
        y = self._vec_norm(y)
        if y is None:
            return None

        R = np.eye(4, dtype=np.float64)
        R[0:3, 0] = x
        R[0:3, 1] = y
        R[0:3, 2] = z
        return tft.quaternion_from_matrix(R)

    # Callback
    def transform_callback(self, msg):
        if not msg.header.frame_id:
            self.pub_dbg.publish(String(data="input_missing_frame_id"))
            return

        model, used_tid = self._get_active_model()

        T_map_tag = self._T_from_pose(msg.pose)

        # Apply model transforms
        T_map_handle = np.dot(T_map_tag, model["T_tag_handle"])
        T_map_hinge = np.dot(T_map_tag, model["T_tag_hinge"])

        # Publish handle pose
        handle_msg = self._pose_from_T(T_map_handle)
        handle_msg.header = msg.header
        handle_msg.header.frame_id = msg.header.frame_id
        self.pub_handle.publish(handle_msg)

        # Prepare hinge pose
        hinge_msg = self._pose_from_T(T_map_hinge)
        hinge_msg.header = msg.header
        hinge_msg.header.frame_id = msg.header.frame_id

        # Rotate hinge axis + door normal into map
        R_map_tag = T_map_tag[0:3, 0:3]
        u_hinge_map = R_map_tag.dot(model["hinge_axis_tag"].reshape(3, 1)).reshape(3)
        u_hinge_map = self._vec_norm(u_hinge_map)
        if u_hinge_map is None:
            self.pub_dbg.publish(String(data="hinge_axis_invalid"))
            return

        n_map = R_map_tag.dot(model["door_normal_axis_tag"].reshape(3, 1)).reshape(3)
        n_map = self._vec_norm(n_map)
        if n_map is None:
            self.pub_dbg.publish(String(data="door_normal_invalid"))
            return

        # Publish hinge axis vector
        v = Vector3Stamped()
        v.header = msg.header
        v.header.frame_id = msg.header.frame_id
        v.vector.x = float(u_hinge_map[0])
        v.vector.y = float(u_hinge_map[1])
        v.vector.z = float(u_hinge_map[2])
        self.pub_hinge_axis.publish(v)

        # Publish plane pose (normal as Z axis)
        plane_pose = PoseStamped()
        plane_pose.header = msg.header
        plane_pose.header.frame_id = msg.header.frame_id
        plane_pose.pose.position = msg.pose.position

        q_plane = self._quat_from_two_axes(z_axis=n_map, x_hint=u_hinge_map)
        if q_plane is None:
            self.pub_dbg.publish(String(data="plane_quat_failed"))
            return

        plane_pose.pose.orientation.x = float(q_plane[0])
        plane_pose.pose.orientation.y = float(q_plane[1])
        plane_pose.pose.orientation.z = float(q_plane[2])
        plane_pose.pose.orientation.w = float(q_plane[3])
        self.pub_plane.publish(plane_pose)

        # Hinge orientation
        if bool(model.get("publish_hinge_orientation", True)):
            handle_pos = T_map_handle[0:3, 3]
            hinge_pos = T_map_hinge[0:3, 3]
            leaf_xy = handle_pos[0:2] - hinge_pos[0:2]
            if np.linalg.norm(leaf_xy) > 1e-9:
                yaw = np.arctan2(leaf_xy[1], leaf_xy[0])
                q_hinge = tft.quaternion_from_euler(0.0, 0.0, yaw)
                hinge_msg.pose.orientation.x = float(q_hinge[0])
                hinge_msg.pose.orientation.y = float(q_hinge[1])
                hinge_msg.pose.orientation.z = float(q_hinge[2])
                hinge_msg.pose.orientation.w = float(q_hinge[3])

        self.pub_hinge.publish(hinge_msg)

        if self.pub_side is not None:
            self.pub_side.publish(String(data=self._last_side))
        if self.pub_hinge_side is not None:
            self.pub_hinge_side.publish(String(data=self._last_hinge_side))
        if self.pub_interaction is not None:
            self.pub_interaction.publish(String(data=self._last_interaction))

        self.pub_dbg.publish(String(
            data="ok tid=%d handle+hinge+plane (door_width_m=%.3f handle_radius_m=%.3f) side=%s hinge_side=%s interaction=%s" %
                 (used_tid, float(model.get("door_width_m", 0.0)), float(model.get("handle_radius_m", 0.0)),
                  self._last_side, self._last_hinge_side, self._last_interaction)
        ))


def main():
    rospy.init_node("door_model_from_tag_node", anonymous=False)
    _ = DoorModelFromTagNode()
    rospy.spin()


if __name__ == "__main__":
    main()
