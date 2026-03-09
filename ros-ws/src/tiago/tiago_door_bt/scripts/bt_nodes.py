#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import py_trees

from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
import rosservice
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool


# actionlib_msgs/GoalStatus values:
SUCCEEDED = 3
ABORTED = 4
REJECTED = 5
PREEMPTED = 2
LOST = 9


class WaitForPose(py_trees.behaviour.Behaviour):
    """
    Wait until a PoseStamped topic has been received recently.
    SUCCESS: pose received and (now - pose.stamp) <= max_age_s
    RUNNING: otherwise
    FAILURE: never (unless misconfigured)
    """

    def __init__(self, name, topic, max_age_s=0.5):
        super(WaitForPose, self).__init__(name=name)
        self.topic = topic
        self.max_age_s = float(max_age_s)
        self.last_msg = None
        self.sub = None

    def setup(self, timeout):
        self.sub = rospy.Subscriber(self.topic, PoseStamped, self._cb, queue_size=1)
        return True

    def _cb(self, msg):
        self.last_msg = msg

    def update(self):
        if self.last_msg is None:
            return py_trees.common.Status.RUNNING

        stamp = self.last_msg.header.stamp
        if stamp == rospy.Time():
            rospy.loginfo("  [%s] Received pose (no timestamp)", self.name)
            return py_trees.common.Status.SUCCESS

        age = (rospy.Time.now() - stamp).to_sec()
        if age <= self.max_age_s:
            rospy.loginfo("  [%s] Received fresh pose (age: %.2fs)", self.name, age)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class PublishEmptyOnce(py_trees.behaviour.Behaviour):
    """
    Publish std_msgs/Empty once when this behaviour is ticked the first time.
    Then returns SUCCESS.
    Also stores a timestamp on the blackboard under blackboard_key_stamp.
    """

    def __init__(self, name, topic, blackboard_key_stamp="last_trigger_stamp"):
        super(PublishEmptyOnce, self).__init__(name=name)
        self.topic = topic
        self.pub = None
        self.sent = False
        self.bb = py_trees.blackboard.Blackboard()
        self.blackboard_key_stamp = blackboard_key_stamp

    def setup(self, timeout):
        self.pub = rospy.Publisher(self.topic, EmptyMsg, queue_size=1, latch=False)
        return True

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            rospy.loginfo("  [%s] Publishing Empty to %s", self.name, self.topic)
            self.pub.publish(EmptyMsg())
            self.bb.set(self.blackboard_key_stamp, rospy.Time.now())
            self.sent = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.SUCCESS


class WaitForMoveBaseResult(py_trees.behaviour.Behaviour):
    """
    Waits for move_base to report SUCCEEDED after a trigger timestamp.

    Input:
      - Subscribes to GoalStatusArray (default /tiago_move_base/move_base/status)
      - Reads trigger stamp from blackboard (key blackboard_key_stamp)

    SUCCESS:
      - status array indicates SUCCEEDED for any goal, AND the status message header.stamp
        is >= trigger stamp (so we don't accept old results)

    FAILURE:
      - ABORTED/REJECTED/LOST after trigger stamp
      - timeout exceeded

    RUNNING:
      - otherwise
    """

    def __init__(self, name,
                 status_topic="/tiago_move_base/move_base/status",
                 timeout_s=60.0,
                 blackboard_key_stamp="last_trigger_stamp"):
        super(WaitForMoveBaseResult, self).__init__(name=name)
        self.status_topic = status_topic
        self.timeout_s = float(timeout_s)
        self.bb = py_trees.blackboard.Blackboard()
        self.blackboard_key_stamp = blackboard_key_stamp

        self.sub = None
        self.last_status_msg = None
        self.start_time = None

    def setup(self, timeout):
        self.sub = rospy.Subscriber(self.status_topic, GoalStatusArray, self._cb, queue_size=10)
        return True

    def _cb(self, msg):
        self.last_status_msg = msg

    def initialise(self):
        self.start_time = rospy.Time.now()
        rospy.loginfo("  [%s] Waiting for navigation result (timeout: %.1fs)", self.name, self.timeout_s)

    def update(self):
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed > self.timeout_s:
            rospy.logwarn("  [%s] Navigation timeout after %.1fs", self.name, elapsed)
            return py_trees.common.Status.FAILURE

        trigger_stamp = self.bb.get(self.blackboard_key_stamp)
        if trigger_stamp is None:
            return py_trees.common.Status.RUNNING

        if self.last_status_msg is None:
            return py_trees.common.Status.RUNNING

        msg_stamp = self.last_status_msg.header.stamp
        if msg_stamp != rospy.Time() and msg_stamp < trigger_stamp:
            return py_trees.common.Status.RUNNING

        codes = [s.status for s in self.last_status_msg.status_list]
        
        if self.last_status_msg.status_list:
            for s in self.last_status_msg.status_list:
                goal_stamp = s.goal_id.stamp
                if goal_stamp >= trigger_stamp:
                    if s.status == SUCCEEDED:
                        rospy.loginfo("  [%s] Navigation SUCCEEDED (%.1fs) - Goal: %s", 
                                     self.name, elapsed, s.goal_id.id)
                        return py_trees.common.Status.SUCCESS
                    elif s.status in [ABORTED, REJECTED, LOST]:
                        rospy.logwarn("  [%s] Navigation FAILED (%.1fs): status=%d - Goal: %s", 
                                     self.name, elapsed, s.status, s.goal_id.id)
                        return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
    
class PublishBoolOnce(py_trees.behaviour.Behaviour):
    """
    Publish std_msgs/Bool once on initialise, then SUCCESS.
    """
    def __init__(self, name, topic, value):
        super(PublishBoolOnce, self).__init__(name=name)
        self.topic = topic
        self.value = bool(value)
        self.pub = None
        self.sent = False

    def setup(self, timeout):
        self.pub = rospy.Publisher(self.topic, Bool, queue_size=1, latch=True)
        return True

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            rospy.loginfo("  [%s] Publishing Bool(%s) to %s", self.name, self.value, self.topic)
            self.pub.publish(Bool(data=self.value))
            self.sent = True
        return py_trees.common.Status.SUCCESS
    
class CallEmptyServiceOnce(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, timeout_s=2.0):
        super(CallEmptyServiceOnce, self).__init__(name=name)
        self.service_name = service_name
        self.timeout_s = float(timeout_s)
        self.called = False
        self.proxy = None

    def setup(self, timeout):
        self.proxy = rospy.ServiceProxy(self.service_name, EmptySrv)
        return True

    def initialise(self):
        self.called = False

    def update(self):
        if self.called:
            return py_trees.common.Status.SUCCESS
        try:
            rospy.loginfo("  [%s] Calling service %s", self.name, self.service_name)
            rospy.wait_for_service(self.service_name, timeout=self.timeout_s)
            self.proxy()
            self.called = True
            rospy.loginfo("  [%s] Service call succeeded", self.name)
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            rospy.logwarn("  [%s] Service call failed: %s", self.name, str(e))
            return py_trees.common.Status.FAILURE
        
class CallSetBoolServiceOnce(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, value, timeout_s=2.0):
        super(CallSetBoolServiceOnce, self).__init__(name=name)
        self.service_name = service_name
        self.value = bool(value)
        self.timeout_s = float(timeout_s)
        self.called = False
        self.proxy = None

    def setup(self, timeout):
        self.proxy = rospy.ServiceProxy(self.service_name, SetBool)
        return True

    def initialise(self):
        self.called = False

    def update(self):
        if self.called:
            return py_trees.common.Status.SUCCESS
        try:
            rospy.loginfo("  [%s] Calling service %s with value=%s", self.name, self.service_name, self.value)
            rospy.wait_for_service(self.service_name, timeout=self.timeout_s)
            resp = self.proxy(self.value)
            self.called = True
            if resp.success:
                rospy.loginfo("  [%s] Service call succeeded: %s", self.name, resp.message)
                return py_trees.common.Status.SUCCESS
            else:
                rospy.logwarn("  [%s] Service returned failure: %s", self.name, resp.message)
                return py_trees.common.Status.FAILURE
        except Exception as e:
            rospy.logwarn("  [%s] Service call failed: %s", self.name, str(e))
            return py_trees.common.Status.FAILURE


class DelaySeconds(py_trees.behaviour.Behaviour):
    """
    Wait for a specified duration in seconds.
    RUNNING: while time < duration
    SUCCESS: after duration elapsed
    """
    
    def __init__(self, name, duration_s):
        super(DelaySeconds, self).__init__(name=name)
        self.duration_s = float(duration_s)
        self.start_time = None

    def initialise(self):
        self.start_time = rospy.Time.now()
        rospy.loginfo("  [%s] Starting %.1fs delay", self.name, self.duration_s)

    def update(self):
        if self.start_time is None:
            return py_trees.common.Status.RUNNING
        
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed >= self.duration_s:
            rospy.loginfo("  [%s] Delay complete (%.1fs)", self.name, elapsed)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            return py_trees.common.Status.FAILURE