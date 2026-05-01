#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import sys
import math

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from tiago_arm_manipulation.srv import MoveToPose


def pose_to_matrix(ps):
    T = tft.quaternion_matrix([
        ps.pose.orientation.x, ps.pose.orientation.y,
        ps.pose.orientation.z, ps.pose.orientation.w,
    ])
    T[0, 3] = ps.pose.position.x
    T[1, 3] = ps.pose.position.y
    T[2, 3] = ps.pose.position.z
    return T


def matrix_to_pose_stamped(T, frame_id):
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose.position.x = T[0, 3]
    ps.pose.position.y = T[1, 3]
    ps.pose.position.z = T[2, 3]
    q = tft.quaternion_from_matrix(T)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def make_T_handle_ee(ox, oy, oz, roll_deg, pitch_deg, yaw_deg, approach):
    T = tft.euler_matrix(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    T[0, 3] = ox + approach
    T[1, 3] = oy
    T[2, 3] = oz
    return T


def publish_viz(pose_pub, marker_pub, ee_pose):
    pose_pub.publish(ee_pose)

    # Arrow along EE x-axis (approach direction)
    m = Marker()
    m.header = ee_pose.header
    m.ns = "grasp_test"
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose = ee_pose.pose
    m.scale.x = 0.15 # shaft length
    m.scale.y = 0.015 # shaft diameter
    m.scale.z = 0.02 # head diameter
    m.color.r = 1.0
    m.color.g = 0.4
    m.color.a = 1.0
    marker_pub.publish(m)


def main():
    rospy.init_node("test_grasp_pose", anonymous=True)

    roll = float(rospy.get_param("~roll", 0.0))
    pitch = float(rospy.get_param("~pitch", 0.0))
    yaw = float(rospy.get_param("~yaw", 0.0))
    execute = bool(rospy.get_param("~execute", True))
    vel = float(rospy.get_param("~vel", 0.2))
    use_handle = bool(rospy.get_param("~use_handle", False))

    pose_pub = rospy.Publisher("/test_grasp_pose/target", PoseStamped, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/test_grasp_pose/target_marker", Marker, queue_size=1, latch=True)
    rospy.sleep(0.3)

    if use_handle:
        ox = float(rospy.get_param("~offset_x", 0.0))
        oy = float(rospy.get_param("~offset_y", 0.0))
        oz = float(rospy.get_param("~offset_z", 0.0))
        approach = float(rospy.get_param("~approach", -0.05))

        rospy.loginfo("[GraspTest] Handle mode: offset=(%.3f,%.3f,%.3f) approach=%.3f RPY=(%.1f,%.1f,%.1f)",
                      ox, oy, oz, approach, roll, pitch, yaw)
        rospy.loginfo("[GraspTest] Waiting for /door/handle_pose_map ...")
        try:
            handle_msg = rospy.wait_for_message("/door/handle_pose_map", PoseStamped, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("[GraspTest] Timed out waiting for /door/handle_pose_map")
            sys.exit(1)

        T_world_handle = pose_to_matrix(handle_msg)
        T_handle_ee = make_T_handle_ee(ox, oy, oz, roll, pitch, yaw, approach)
        T_world_ee = np.dot(T_world_handle, T_handle_ee)
        frame_id = handle_msg.header.frame_id
    else:
        px = float(rospy.get_param("~x", 0.55)) # forward (m)
        py = float(rospy.get_param("~y", -0.20)) # lateral (m), negative = right
        pz = float(rospy.get_param("~z", 1.00)) # height (m)
        frame_id = rospy.get_param("~frame", "base_footprint")

        rospy.loginfo("[GraspTest] Fixed mode: pos=(%.3f,%.3f,%.3f) RPY=(%.1f,%.1f,%.1f) frame=%s",
                      px, py, pz, roll, pitch, yaw, frame_id)

        T_world_ee = tft.euler_matrix(math.radians(roll), math.radians(pitch), math.radians(yaw))
        T_world_ee[0, 3] = px
        T_world_ee[1, 3] = py
        T_world_ee[2, 3] = pz

    ee_pose = matrix_to_pose_stamped(T_world_ee, frame_id)
    p = ee_pose.pose.position
    q = ee_pose.pose.orientation
    rpy = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    rospy.loginfo("[GraspTest] EE target in '%s':", frame_id)
    rospy.loginfo("  position:    x=%.4f  y=%.4f  z=%.4f", p.x, p.y, p.z)
    rospy.loginfo("  orientation: x=%.4f  y=%.4f  z=%.4f  w=%.4f", q.x, q.y, q.z, q.w)
    rospy.loginfo("  RPY (deg):   r=%.1f  p=%.1f  y=%.1f",
                  math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2]))
    rospy.loginfo("  planner.yaml values (rad):  roll=%.4f  pitch=%.4f  yaw=%.4f",
                  rpy[0], rpy[1], rpy[2])

    publish_viz(pose_pub, marker_pub, ee_pose)

    if not execute:
        rospy.loginfo("[GraspTest] execute:=false — visualizing pose, Ctrl-C to exit.")
        rospy.loginfo("[GraspTest] Add in RViz:  PoseStamped -> /test_grasp_pose/target")
        rospy.loginfo("[GraspTest]               Marker      -> /test_grasp_pose/target_marker")
        rospy.spin()
        return

    try:
        rospy.wait_for_service("/tiago_arm_manipulation/move_to_pose", timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("[GraspTest] /tiago_arm_manipulation/move_to_pose service not available")
        sys.exit(1)

    svc = rospy.ServiceProxy("/tiago_arm_manipulation/move_to_pose", MoveToPose)
    resp = svc(
        target=ee_pose,
        position_tolerance=0.02,
        orientation_tolerance=0.1,
        velocity_scaling=vel,
        acceleration_scaling=vel,
        execute=True,
    )

    if resp.ok:
        rospy.loginfo("[GraspTest] Motion succeeded: %s", resp.message)
    else:
        rospy.logerr("[GraspTest] Motion FAILED: %s", resp.message)
        sys.exit(1)


if __name__ == "__main__":
    main()
