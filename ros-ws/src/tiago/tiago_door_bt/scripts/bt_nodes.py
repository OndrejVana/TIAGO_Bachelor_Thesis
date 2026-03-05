#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import py_trees

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool
import rosservice
from std_srvs.srv import Empty
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
            return py_trees.common.Status.SUCCESS

        age = (rospy.Time.now() - stamp).to_sec()
        if age <= self.max_age_s:
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
        self.pub = rospy.Publisher(self.topic, Empty, queue_size=1, latch=False)
        return True

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            self.pub.publish(Empty())
            self.bb.set(self.blackboard_key_stamp, rospy.Time.now())
            self.sent = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.SUCCESS


class WaitForMoveBaseResult(py_trees.behaviour.Behaviour):
    """
    Waits for move_base to report SUCCEEDED after a trigger timestamp.

    Input:
      - Subscribes to GoalStatusArray (default /move_base/status)
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
                 status_topic="/move_base/status",
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

    def update(self):
        if (rospy.Time.now() - self.start_time).to_sec() > self.timeout_s:
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

        if SUCCEEDED in codes:
            return py_trees.common.Status.SUCCESS

        if ABORTED in codes or REJECTED in codes or LOST in codes:
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
        self.proxy = rospy.ServiceProxy(self.service_name, Empty)
        return True

    def initialise(self):
        self.called = False

    def update(self):
        if self.called:
            return py_trees.common.Status.SUCCESS
        try:
            rospy.wait_for_service(self.service_name, timeout=self.timeout_s)
            self.proxy()
            self.called = True
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            rospy.logwarn("%s: service call failed: %s", self.name, str(e))
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
            rospy.wait_for_service(self.service_name, timeout=self.timeout_s)
            resp = self.proxy(self.value)
            self.called = True
            return py_trees.common.Status.SUCCESS if resp.success else py_trees.common.Status.FAILURE
        except Exception as e:
            rospy.logwarn("%s: service call failed: %s", self.name, str(e))
            return py_trees.common.Status.FAILURE