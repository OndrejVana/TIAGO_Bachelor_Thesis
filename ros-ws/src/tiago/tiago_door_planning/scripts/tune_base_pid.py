#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import math
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf.transformations as tft


def angle_wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def odom_to_xyyaw(odom):
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return p.x, p.y, yaw


class BasePIDTuner(object):
    def __init__(self):
        rospy.init_node("tune_base_pid", anonymous=True)

        # Test configuration
        self.mode = rospy.get_param("~mode", "linear") # "linear" or "angular"
        self.step = float(rospy.get_param("~step", 0.5)) # m for linear, deg for angular
        self.repeats = int(rospy.get_param("~repeats", 5))
        self.settle_s = float(rospy.get_param("~settle", 0.5))
        self.pos_tol = float(rospy.get_param("~pos_tol", 0.01))
        self.ang_tol = float(rospy.get_param("~ang_tol", 0.05))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        # PI gains
        self.kp_linear = float(rospy.get_param("~kp_linear", 0.8))
        self.ki_linear = float(rospy.get_param("~ki_linear", 0.5))
        self.kp_angular = float(rospy.get_param("~kp_angular", 0.8))
        self.ki_angular = float(rospy.get_param("~ki_angular", 0.5))
        self.max_linear = float(rospy.get_param("~max_linear", 0.2))
        self.max_angular = float(rospy.get_param("~max_angular", 0.6))

        self._odom = None
        rospy.Subscriber("/mobile_base_controller/odom", Odometry,
                         self._odom_cb, queue_size=1)
        self._cmd_pub = rospy.Publisher("/mobile_base_controller/cmd_vel",
                                        Twist, queue_size=1)

        # Diagnostic topics for rqt_plot
        self._pub_lerr = rospy.Publisher("/tune_pid/linear_error", Float64, queue_size=1)
        self._pub_aerr = rospy.Publisher("/tune_pid/angular_error", Float64, queue_size=1)
        self._pub_lcmd = rospy.Publisher("/tune_pid/cmd_linear", Float64, queue_size=1)
        self._pub_acmd = rospy.Publisher("/tune_pid/cmd_angular", Float64, queue_size=1)
        self._pub_pos      = rospy.Publisher("/tune_pid/position", Float64, queue_size=1)
        self._pub_setpoint = rospy.Publisher("/tune_pid/setpoint", Float64, queue_size=1)

        rospy.loginfo("[TunePID] mode=%s  step=%.2f  repeats=%d  rate=%.0f Hz",
                      self.mode, self.step, self.repeats, self.rate_hz)
        rospy.loginfo("[TunePID] kp_lin=%.3f  ki_lin=%.3f  kp_ang=%.3f  ki_ang=%.3f",
                      self.kp_linear, self.ki_linear, self.kp_angular, self.ki_angular)
        rospy.loginfo("[TunePID] max_linear=%.2f m/s  max_angular=%.2f rad/s",
                      self.max_linear, self.max_angular)

    def _odom_cb(self, msg):
        self._odom = msg

    def _wait_for_odom(self, timeout=10.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._odom is not None:
                return True
            if rospy.Time.now() > deadline:
                return False
            r.sleep()
        return False

    def _go_to(self, tx, ty, tyaw, label="", x0=0.0, y0=0.0, yaw0=0.0):
        rate = rospy.Rate(self.rate_hz)
        dt   = 1.0 / self.rate_hz

        max_int_lin = self.max_linear / max(self.ki_linear,  1e-9)
        max_int_ang = self.max_angular / max(self.ki_angular, 1e-9)
        int_lin = 0.0
        int_ang = 0.0

        if self.mode == "angular":
            setpoint_1d = angle_wrap(tyaw - yaw0)
        else:
            setpoint_1d = (tx - x0) * math.cos(yaw0) + (ty - y0) * math.sin(yaw0)

        rospy.loginfo("[TunePID] %s -> (%.3f, %.3f, %.1f deg)",
                      label, tx, ty, math.degrees(tyaw))

        while not rospy.is_shutdown():
            x, y, yaw = odom_to_xyyaw(self._odom)

            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)
            heading_to_goal = math.atan2(dy, dx)
            heading_err = angle_wrap(heading_to_goal - yaw)
            yaw_err = angle_wrap(tyaw - yaw)

            if self.mode == "angular":
                lin_vel = 0.0
                ang_vel = self.kp_angular * yaw_err + self.ki_angular * int_ang
                ang_vel = np.clip(ang_vel, -self.max_angular, self.max_angular)
                int_ang = np.clip(int_ang + yaw_err * dt, -max_int_ang, max_int_ang)

                current_1d = angle_wrap(yaw - yaw0)
                self._pub_lerr.publish(Float64(0.0))
                self._pub_aerr.publish(Float64(yaw_err))
                self._pub_lcmd.publish(Float64(lin_vel))
                self._pub_acmd.publish(Float64(ang_vel))
                self._pub_pos.publish(Float64(current_1d))
                self._pub_setpoint.publish(Float64(setpoint_1d))

                cmd = Twist()
                cmd.angular.z = ang_vel
                self._cmd_pub.publish(cmd)

                if abs(yaw_err) < self.ang_tol:
                    rospy.loginfo("[TunePID] %s reached (yaw_err=%.2f deg)",
                                  label, math.degrees(yaw_err))
                    break

            else:  # linear
                reverse = abs(heading_err) > math.pi / 2

                if dist < self.pos_tol:
                    lin_vel = 0.0
                    ang_vel = 0.0
                    int_ang = np.clip(int_ang + yaw_err * dt, -max_int_ang, max_int_ang)
                elif reverse:
                    eff_ang = angle_wrap(heading_err - math.pi)
                    lin_vel = -(self.kp_linear * dist + self.ki_linear * int_lin)
                    ang_vel = -(self.kp_angular * eff_ang + self.ki_angular * int_ang)
                    int_ang = np.clip(int_ang + eff_ang * dt, -max_int_ang, max_int_ang)
                else:
                    lin_vel = self.kp_linear * dist + self.ki_linear * int_lin
                    ang_vel = self.kp_angular * heading_err + self.ki_angular * int_ang
                    int_ang = np.clip(int_ang + heading_err * dt, -max_int_ang, max_int_ang)

                lin_vel = np.clip(lin_vel, -self.max_linear,  self.max_linear)
                ang_vel = np.clip(ang_vel, -self.max_angular, self.max_angular)
                int_lin = np.clip(int_lin + dist * dt, -max_int_lin, max_int_lin)

                current_1d = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
                self._pub_lerr.publish(Float64(dist))
                self._pub_aerr.publish(Float64(yaw_err))
                self._pub_lcmd.publish(Float64(lin_vel))
                self._pub_acmd.publish(Float64(ang_vel))
                self._pub_pos.publish(Float64(current_1d))
                self._pub_setpoint.publish(Float64(setpoint_1d))

                cmd = Twist()
                cmd.linear.x  = lin_vel
                cmd.angular.z = ang_vel
                self._cmd_pub.publish(cmd)

                if dist < self.pos_tol and abs(yaw_err) < self.ang_tol:
                    rospy.loginfo("[TunePID] %s reached (dist=%.3f m, yaw_err=%.2f deg)",
                                  label, dist, math.degrees(yaw_err))
                    break

            rate.sleep()

        self._cmd_pub.publish(Twist())  # stop

    def run(self):
        if not self._wait_for_odom():
            rospy.logerr("[TunePID] No odometry — is the robot running?")
            sys.exit(1)

        x0, y0, yaw0 = odom_to_xyyaw(self._odom)
        rospy.loginfo("[TunePID] Start pose: (%.3f, %.3f, %.1f deg)",
                      x0, y0, math.degrees(yaw0))

        if self.mode == "linear":
            dx = math.cos(yaw0) * self.step
            dy = math.sin(yaw0) * self.step
            fwd  = (x0 + dx, y0 + dy, yaw0)
            back = (x0,      y0,      yaw0)
            targets = [fwd, back] * self.repeats
            labels  = ["fwd", "back"] * self.repeats

        elif self.mode == "angular":
            step_rad = math.radians(self.step)
            left  = (x0, y0, angle_wrap(yaw0 + step_rad))
            right = (x0, y0, yaw0)
            targets = [left, right] * self.repeats
            labels  = ["left", "right"] * self.repeats

        else:
            rospy.logerr("[TunePID] Unknown mode '%s' — use 'linear' or 'angular'", self.mode)
            sys.exit(1)

        for (tx, ty, tyaw), label in zip(targets, labels):
            if rospy.is_shutdown():
                break
            self._go_to(tx, ty, tyaw, label=label, x0=x0, y0=y0, yaw0=yaw0)
            rospy.sleep(self.settle_s)

        rospy.loginfo("[TunePID] All %d legs done.", len(targets))


if __name__ == "__main__":
    BasePIDTuner().run()
