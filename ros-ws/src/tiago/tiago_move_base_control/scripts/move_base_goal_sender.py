#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray


class MoveBaseGoalSender(object):
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/tiago_move_base_control/goal")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/tiago_move_base_control/cancel")
        self.action_name = rospy.get_param("~action_name", "/move_base")
        self.default_frame = rospy.get_param("~default_frame", "map")
        self.wait_for_server = float(rospy.get_param("~wait_for_server", 5.0))
        self.stamp_now = bool(rospy.get_param("~stamp_now", True))

        self.client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server: %s (timeout %.1fs)",
                      self.action_name, self.wait_for_server)

        if not self.client.wait_for_server(rospy.Duration(self.wait_for_server)):
            rospy.logerr("move_base action server not available: %s", self.action_name)
            rospy.logerr("Is move_base running? Check: rosnode list | grep move_base")
        else:
            rospy.loginfo("Connected to move_base action server: %s", self.action_name)

        self.pub_status = rospy.Publisher("~status", GoalStatusArray, queue_size=10)
        # move_base publishes status on <action_name>/status
        self.sub_status = rospy.Subscriber(self.action_name + "/status", GoalStatusArray, self._status_cb, queue_size=10)

        self.sub_goal = rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=10)
        self.sub_cancel = rospy.Subscriber(self.cancel_topic, Empty, self._cancel_cb, queue_size=10)

    def _status_cb(self, msg):
        self.pub_status.publish(msg)

    def _cancel_cb(self, _msg):
        rospy.logwarn("Cancel requested: canceling all move_base goals")
        self.client.cancel_all_goals()

    def _goal_cb(self, msg):
        if not self.client.wait_for_server(rospy.Duration(0.1)):
            rospy.logerr("move_base server not available when goal received")
            return

        frame_id = msg.header.frame_id.strip() if msg.header.frame_id else ""
        if not frame_id:
            frame_id = self.default_frame

        g = MoveBaseGoal()
        g.target_pose.header.frame_id = frame_id
        g.target_pose.header.stamp = rospy.Time.now() if self.stamp_now else msg.header.stamp
        g.target_pose.pose = msg.pose

        rospy.loginfo("Sending goal to %s in frame '%s': (x=%.3f y=%.3f)",
                      self.action_name, frame_id,
                      g.target_pose.pose.position.x, g.target_pose.pose.position.y)

        self.client.send_goal(g)


def main():
    rospy.init_node("move_base_goal_sender", anonymous=False)
    _ = MoveBaseGoalSender()
    rospy.spin()


if __name__ == "__main__":
    main()