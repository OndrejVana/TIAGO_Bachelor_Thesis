#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import sys
import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tiago_door_planning.msg import (
    PlanDoorOpeningAction,
    PlanDoorOpeningGoal,
    ExecuteDoorOpeningAction,
    ExecuteDoorOpeningGoal,
)

try:
    _input = raw_input
except NameError:
    _input = input


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


def _slice_arm_trajectory(arm_traj, t_start, t_end):
    """Extract arm trajectory points in [t_start, t_end] and reset times to start from 0."""
    sliced = JointTrajectory()
    sliced.joint_names = arm_traj.joint_names
    for pt in arm_traj.points:
        t = pt.time_from_start.to_sec()
        if t_start <= t <= t_end:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = pt.positions
            new_pt.velocities = pt.velocities
            new_pt.accelerations = pt.accelerations
            new_pt.effort = pt.effort
            new_pt.time_from_start = rospy.Duration(t - t_start)
            sliced.points.append(new_pt)
    return sliced


def step_execute(client, plan_result, velocity_scaling):
    poses = plan_result.base_path.poses
    times = plan_result.base_times
    n = len(poses)

    if n == 0:
        rospy.logwarn("[Step] Empty base path — nothing to execute.")
        return True
    if len(times) != n:
        rospy.logerr("[Step] base_times length mismatch.")
        return False

    for i, pose in enumerate(poses):
        duration = times[i] if i == 0 else times[i] - times[i - 1]
        if duration <= 0:
            rospy.logwarn("[Step] Waypoint %d has zero duration — skipping.", i + 1)
            continue

        rospy.loginfo("[Step] Waypoint %d/%d  x=%.3f y=%.3f  — press Enter to go, Ctrl-C to abort",
                      i + 1, n, pose.pose.position.x, pose.pose.position.y)
        try:
            _input("")
        except (KeyboardInterrupt, EOFError):
            rospy.logwarn("[Step] Aborted at waypoint %d.", i + 1)
            return False

        t_start = 0.0 if i == 0 else times[i - 1]
        t_end = times[i]

        goal = ExecuteDoorOpeningGoal()
        goal.base_path.header = plan_result.base_path.header
        goal.base_path.poses = [pose]
        goal.base_times = [duration]
        goal.arm_trajectory = _slice_arm_trajectory(plan_result.arm_trajectory, t_start, t_end)
        goal.velocity_scaling = velocity_scaling

        client.send_goal_and_wait(goal)
        result = client.get_result()

        if not result.success:
            rospy.logerr("[Step] Waypoint %d FAILED: %s", i + 1, result.message)
            return False
        rospy.loginfo("[Step] Waypoint %d OK.", i + 1)

    rospy.loginfo("[Step] All %d waypoints done.", n)
    return True


def main():
    rospy.init_node("test_door_opening", anonymous=True)

    angle_rad = float(rospy.get_param("~angle", 1.57)) # rad
    velocity_scaling = float(rospy.get_param("~scaling", 0.4)) # 0.1 - 1.0
    push_motion = bool(rospy.get_param("~push", True))
    planning_time = float(rospy.get_param("~plan_time", 60.0)) # s
    execute_after = bool(rospy.get_param("~execute", False)) # set False to plan-only
    generate_arm = bool(rospy.get_param("~arm", True)) # set False to skip arm IK
    step_mode = bool(rospy.get_param("~step", False))

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

    # Execute
    if step_mode:
        rospy.loginfo("[Test] Step mode — pausing between each base waypoint.")
        success = step_execute(exec_client, result, velocity_scaling)
    else:
        success = execute(exec_client, result, velocity_scaling)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
