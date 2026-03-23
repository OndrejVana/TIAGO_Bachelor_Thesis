#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations as tft


def quat_to_array(q):
    """Convert geometry_msgs/Quaternion to [x, y, z, w] numpy array."""
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def fill_quat_msg(q_msg, arr):
    """Fill Quaternion message fields from [x, y, z, w] array."""
    q_msg.x = float(arr[0])
    q_msg.y = float(arr[1])
    q_msg.z = float(arr[2])
    q_msg.w = float(arr[3])
    return q_msg


def normalize_quaternion(q):
    """Return unit quaternion. Returns q unchanged if near-zero norm."""
    norm = np.linalg.norm(q)
    if norm <= 1e-12:
        return q
    return q / norm


def yaw_from_quat(q):
    return tft.euler_from_quaternion(quat_to_array(q))[2]


def quat_from_yaw(yaw):
    q = Quaternion()
    return fill_quat_msg(q, tft.quaternion_from_euler(0.0, 0.0, yaw))


def pose_stamped(frame_id, x, y, yaw, z):
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation = quat_from_yaw(yaw)
    return ps


def angle_wrap(a):
    """Wrap angle to [-pi, pi]."""
    return (float(a) + np.pi) % (2.0 * np.pi) - np.pi


def clamp(value, low, high):
    return np.maximum(low, np.minimum(high, value))
