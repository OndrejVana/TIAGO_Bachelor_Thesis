#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


MOTION_NAME = 'home'
STARTUP_DELAY = 60.0
RETRY_DELAY = 30.0
MAX_RETRIES = 10
SERVER_TIMEOUT = rospy.Duration(60.0)
MOTION_TIMEOUT = rospy.Duration(30.0)


if __name__ == '__main__':
    rospy.init_node('tuck_arm', anonymous=False)
    rospy.loginfo('tuck_arm: waiting %.0fs for bringup to settle...', STARTUP_DELAY)
    time.sleep(STARTUP_DELAY)

    client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

    rospy.loginfo('tuck_arm: waiting for /play_motion action server...')
    if not client.wait_for_server(SERVER_TIMEOUT):
        rospy.logerr('tuck_arm: /play_motion not available, giving up')
        exit(1)

    goal = PlayMotionGoal()
    goal.motion_name   = MOTION_NAME
    goal.skip_planning = False

    for attempt in range(1, MAX_RETRIES + 1):
        rospy.loginfo("tuck_arm: executing play_motion '%s' (attempt %d/%d)",
                      MOTION_NAME, attempt, MAX_RETRIES)
        client.send_goal(goal)

        if client.wait_for_result(MOTION_TIMEOUT):
            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("tuck_arm: '%s' done", MOTION_NAME)
                break
            rospy.logwarn("tuck_arm: attempt %d failed (state=%d), retrying in %.0fs...",
                          attempt, state, RETRY_DELAY)
        else:
            rospy.logwarn("tuck_arm: attempt %d timed out, retrying in %.0fs...",
                          attempt, RETRY_DELAY)
            client.cancel_goal()

        if attempt < MAX_RETRIES:
            time.sleep(RETRY_DELAY)
    else:
        rospy.logerr("tuck_arm: gave up after %d attempts", MAX_RETRIES)
