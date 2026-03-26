#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import sys
import rospy
import actionlib

from std_srvs.srv import Empty
from tiago_door_planning.msg import (
    PlanDoorOpeningAction,
    PlanDoorOpeningGoal,
    ExecuteDoorOpeningAction,
    ExecuteDoorOpeningGoal,
)


def plan(client, angle_rad, push_motion, planning_time, generate_arm=True):
    goal = PlanDoorOpeningGoal()
    goal.goal_open_angle_rad = angle_rad
    goal.push_motion = push_motion
    goal.generate_arm_traj = generate_arm
    goal.publish_paths = True
    goal.allowed_planning_time = planning_time

    rospy.loginfo("[Test] Sending planning goal: angle=%.2f rad (%.1f deg), push=%s",
                  angle_rad, angle_rad * 57.3, push_motion)

    client.send_goal_and_wait(goal)
    result = client.get_result()

    if not result.success:
        rospy.logerr("[Test] Planning FAILED: %s", result.message)
        return None

    rospy.loginfo("[Test] Planning OK: %s", result.message)
    rospy.loginfo("[Test]   Base waypoints : %d", len(result.base_path.poses))
    rospy.loginfo("[Test]   Arm points     : %d", len(result.arm_trajectory.points))
    rospy.loginfo("[Test]   Base times     : %s", [round(t, 3) for t in result.base_times])
    rospy.loginfo("[Test]   Total duration : %.2f s", result.base_times[-1] if result.base_times else 0.0)

    return result


def execute(client, plan_result, velocity_scaling):
    goal = ExecuteDoorOpeningGoal()
    goal.base_path = plan_result.base_path
    goal.base_times = plan_result.base_times
    goal.arm_trajectory = plan_result.arm_trajectory
    goal.velocity_scaling = velocity_scaling

    rospy.loginfo("[Test] Sending execution goal: velocity_scaling=%.2f", velocity_scaling)

    client.send_goal_and_wait(goal)
    result = client.get_result()

    if result.success:
        rospy.loginfo("[Test] Execution SUCCEEDED: %s", result.message)
    else:
        rospy.logerr("[Test] Execution FAILED: %s", result.message)

    return result.success


def main():
    rospy.init_node("test_door_opening", anonymous=True)

    angle_rad = float(rospy.get_param("~angle", 1.57))   # rad
    velocity_scaling = float(rospy.get_param("~scaling", 0.5))  # 0.1 - 1.0
    push_motion = bool(rospy.get_param("~push", True))
    planning_time = float(rospy.get_param("~plan_time", 60.0))  # s
    execute_after = bool(rospy.get_param("~execute", False))   # set False to plan-only
    generate_arm = bool(rospy.get_param("~arm", True))   # set False to skip arm IK

    # Connect to servers
    rospy.loginfo("[Test] Connecting to planning server...")
    plan_client = actionlib.SimpleActionClient(
        "/plan_door_opening", PlanDoorOpeningAction
    )
    if not plan_client.wait_for_server(rospy.Duration(15.0)):
        rospy.logerr("[Test] Planning server not available")
        sys.exit(1)
    rospy.loginfo("[Test] Planning server connected.")

    if execute_after:
        rospy.loginfo("[Test] Connecting to execution server...")
        exec_client = actionlib.SimpleActionClient(
            "/execute_door_opening", ExecuteDoorOpeningAction
        )
        if not exec_client.wait_for_server(rospy.Duration(15.0)):
            rospy.logerr("[Test] Execution server not available")
            sys.exit(1)
        rospy.loginfo("[Test] Execution server connected.")

    # Plan
    result = plan(plan_client, angle_rad, push_motion, planning_time, generate_arm)
    if result is None:
        sys.exit(1)

    if not execute_after:
        rospy.loginfo("[Test] Plan-only mode — not executing. Done.")
        return

    rospy.loginfo("[Test] Switching RTAB-Map to localization mode...")
    try:
        rospy.wait_for_service("/set_mode_localization", timeout=5.0)
        rospy.ServiceProxy("/set_mode_localization", Empty)()
        rospy.loginfo("[Test] RTAB-Map: localization mode active.")
    except Exception as e:
        rospy.logwarn("[Test] Could not call /set_mode_localization: %s", e)

    # Execute
    success = execute(exec_client, result, velocity_scaling)

    rospy.loginfo("[Test] Restoring RTAB-Map to SLAM/mapping mode...")
    try:
        rospy.wait_for_service("/set_mode_mapping", timeout=5.0)
        rospy.ServiceProxy("/set_mode_mapping", Empty)()
        rospy.loginfo("[Test] RTAB-Map: mapping mode restored.")
    except Exception as e:
        rospy.logwarn("[Test] Could not call /set_mode_mapping: %s", e)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()