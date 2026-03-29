#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math

import rospy
import py_trees
import actionlib

from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool, String
import rosservice
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


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


class CallTriggerServiceOnce(py_trees.behaviour.Behaviour):
    """
    Call a std_srvs/Trigger service once and return SUCCESS/FAILURE based on response.
    """
    
    def __init__(self, name, service_name, timeout_s=2.0):
        super(CallTriggerServiceOnce, self).__init__(name=name)
        self.service_name = service_name
        self.timeout_s = float(timeout_s)
        self.called = False
        self.proxy = None

    def setup(self, timeout):
        self.proxy = rospy.ServiceProxy(self.service_name, Trigger)
        return True

    def initialise(self):
        self.called = False

    def update(self):
        if self.called:
            return py_trees.common.Status.SUCCESS
        try:
            rospy.loginfo("  [%s] Calling service %s", self.name, self.service_name)
            rospy.wait_for_service(self.service_name, timeout=self.timeout_s)
            resp = self.proxy()
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


class CallDoorPregraspOnce(py_trees.behaviour.Behaviour):
    """
    Call tiago_arm_manipulation/door_pregrasp service once to move arm to pregrasp position.
    Uses latest detected handle pose.
    """
    
    def __init__(self, name, service_name="/tiago_arm_manipulation/door_pregrasp", 
                 execute=True, approach_distance=0.12, timeout_s=30.0):
        super(CallDoorPregraspOnce, self).__init__(name=name)
        self.service_name = service_name
        self.execute = execute
        self.approach_distance = approach_distance
        self.timeout_s = float(timeout_s)
        self.called = False
        self.proxy = None

    def setup(self, timeout):
        from tiago_arm_manipulation.srv import DoorPregrasp
        self.proxy = rospy.ServiceProxy(self.service_name, DoorPregrasp)
        return True

    def initialise(self):
        self.called = False

    def update(self):
        if self.called:
            return py_trees.common.Status.SUCCESS
        try:
            from tiago_arm_manipulation.srv import DoorPregraspRequest
            from geometry_msgs.msg import PoseStamped
            
            rospy.loginfo("  [%s] Calling door_pregrasp service (execute=%s)", 
                         self.name, self.execute)
            rospy.wait_for_service(self.service_name, timeout=5.0)
            
            # Create request
            req = DoorPregraspRequest()
            req.use_latest_handle = True
            req.handle_override = PoseStamped()
            req.approach_distance = self.approach_distance
            req.lateral_offset = 0.0
            req.vertical_offset = 0.0
            req.velocity_scaling = 0.2
            req.acceleration_scaling = 0.2
            req.execute = self.execute
            
            resp = self.proxy.call(req)
            self.called = True
            
            if resp.ok:
                rospy.loginfo("  [%s] Door pregrasp succeeded: %s", self.name, resp.message)
                return py_trees.common.Status.SUCCESS
            else:
                rospy.logwarn("  [%s] Door pregrasp failed: %s", self.name, resp.message)
                return py_trees.common.Status.FAILURE
        except Exception as e:
            rospy.logwarn("  [%s] Door pregrasp call failed: %s", self.name, str(e))
            return py_trees.common.Status.FAILURE


class WaitForInteraction(py_trees.behaviour.Behaviour):
    """
    Waits for a fresh /door/interaction message (std_msgs/String, value "push" or "pull").
    Stores push_motion (bool) on the blackboard.

    SUCCESS: received "push" or "pull" within max_age_s
    RUNNING: otherwise
    FAILURE: never
    """

    def __init__(self, name, topic, max_age_s=1.0, blackboard_key="push_motion"):
        super(WaitForInteraction, self).__init__(name=name)
        self.topic = topic
        self.max_age_s = float(max_age_s)
        self.blackboard_key = blackboard_key
        self.last_msg = None
        self.sub = None
        self.bb = py_trees.blackboard.Blackboard()

    def setup(self, timeout):
        from std_msgs.msg import String as StringMsg
        self.sub = rospy.Subscriber(self.topic, StringMsg, self._cb, queue_size=1)
        return True

    def _cb(self, msg):
        self.last_msg = msg

    def update(self):
        if self.last_msg is None:
            return py_trees.common.Status.RUNNING

        value = self.last_msg.data.strip().lower()
        if value not in ("push", "pull"):
            return py_trees.common.Status.RUNNING

        stamp = self.last_msg.header.stamp if hasattr(self.last_msg, 'header') else rospy.Time()
        if stamp != rospy.Time():
            age = (rospy.Time.now() - stamp).to_sec()
            if age > self.max_age_s:
                return py_trees.common.Status.RUNNING

        push_motion = (value == "push")
        self.bb.set(self.blackboard_key, push_motion)
        rospy.loginfo("  [%s] interaction=%s -> push_motion=%s", self.name, value, push_motion)
        return py_trees.common.Status.SUCCESS


class CallPlanDoorOpeningAction(py_trees.behaviour.Behaviour):
    """
    Sends a PlanDoorOpening action goal to the door planning server and waits for the result.

    RUNNING: while the action is in progress or server not yet available
    SUCCESS: planning succeeded — result stored on blackboard under blackboard_key
    FAILURE: planning failed, server rejected goal, or timeout exceeded
    """

    def __init__(self, name, action_ns, goal_open_angle_rad,
                 generate_arm_traj=True, publish_paths=True,
                 allowed_planning_time=30.0, blackboard_key="plan_result",
                 push_motion_blackboard_key="push_motion",
                 timeout_s=60.0):
        super(CallPlanDoorOpeningAction, self).__init__(name=name)
        self.action_ns = action_ns
        self.push_motion_blackboard_key = push_motion_blackboard_key
        self.goal_open_angle_rad = float(goal_open_angle_rad)
        self.generate_arm_traj = bool(generate_arm_traj)
        self.publish_paths = bool(publish_paths)
        self.allowed_planning_time = float(allowed_planning_time)
        self.blackboard_key = blackboard_key
        self.timeout_s = float(timeout_s)
        self.client = None
        self.goal_sent = False
        self.start_time = None
        self.bb = py_trees.blackboard.Blackboard()

    def setup(self, timeout):
        from tiago_door_planning.msg import PlanDoorOpeningAction as PlanAction
        self.client = actionlib.SimpleActionClient(self.action_ns, PlanAction)
        return True

    def initialise(self):
        self.goal_sent = False
        self.start_time = rospy.Time.now()

    def update(self):
        if not self.goal_sent:
            if not self.client.wait_for_server(rospy.Duration(0.5)):
                rospy.logwarn_throttle(5.0, "  [%s] Waiting for action server %s",
                                       self.name, self.action_ns)
                return py_trees.common.Status.RUNNING

            push_motion = self.bb.get(self.push_motion_blackboard_key)
            if push_motion is None:
                rospy.logwarn("  [%s] No push_motion on blackboard key '%s'",
                              self.name, self.push_motion_blackboard_key)
                return py_trees.common.Status.FAILURE

            from tiago_door_planning.msg import PlanDoorOpeningGoal
            goal = PlanDoorOpeningGoal()
            goal.push_motion = bool(push_motion)
            goal.goal_open_angle_rad = self.goal_open_angle_rad
            goal.generate_arm_traj = self.generate_arm_traj
            goal.publish_paths = self.publish_paths
            goal.allowed_planning_time = self.allowed_planning_time
            rospy.loginfo(
                "  [%s] Sending PlanDoorOpening goal (push=%s, angle=%.1f deg, budget=%.1fs)",
                self.name, bool(push_motion),
                math.degrees(self.goal_open_angle_rad), self.allowed_planning_time
            )
            self.client.send_goal(goal)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed > self.timeout_s:
            rospy.logwarn("  [%s] Planning timeout after %.1fs", self.name, elapsed)
            self.client.cancel_goal()
            return py_trees.common.Status.FAILURE

        state = self.client.get_state()
        if state == SUCCEEDED:
            result = self.client.get_result()
            if result and result.success:
                self.bb.set(self.blackboard_key, result)
                rospy.loginfo("  [%s] Planning succeeded: %s", self.name, result.message)
                return py_trees.common.Status.SUCCESS
            else:
                msg = result.message if result else "no result"
                rospy.logwarn("  [%s] Planning returned failure: %s", self.name, msg)
                return py_trees.common.Status.FAILURE
        elif state in (ABORTED, REJECTED, LOST, PREEMPTED):
            rospy.logwarn("  [%s] Planning action ended with state %d", self.name, state)
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class CallExecuteDoorOpeningAction(py_trees.behaviour.Behaviour):
    """
    Reads the door plan from the blackboard and sends an ExecuteDoorOpening action goal.

    RUNNING: while execution is in progress or server not yet available
    SUCCESS: execution finished successfully
    FAILURE: no plan on blackboard, execution failed, or timeout exceeded
    """

    def __init__(self, name, action_ns, blackboard_key="plan_result",
                 velocity_scaling=0.5, timeout_s=120.0):
        super(CallExecuteDoorOpeningAction, self).__init__(name=name)
        self.action_ns = action_ns
        self.blackboard_key = blackboard_key
        self.velocity_scaling = float(velocity_scaling)
        self.timeout_s = float(timeout_s)
        self.client = None
        self.goal_sent = False
        self.start_time = None
        self.bb = py_trees.blackboard.Blackboard()

    def setup(self, timeout):
        from tiago_door_planning.msg import ExecuteDoorOpeningAction as ExecAction
        self.client = actionlib.SimpleActionClient(self.action_ns, ExecAction)
        return True

    def initialise(self):
        self.goal_sent = False
        self.start_time = rospy.Time.now()

    def update(self):
        if not self.goal_sent:
            if not self.client.wait_for_server(rospy.Duration(0.5)):
                rospy.logwarn_throttle(5.0, "  [%s] Waiting for action server %s",
                                       self.name, self.action_ns)
                return py_trees.common.Status.RUNNING

            plan = self.bb.get(self.blackboard_key)
            if plan is None:
                rospy.logwarn("  [%s] No plan on blackboard key '%s'",
                              self.name, self.blackboard_key)
                return py_trees.common.Status.FAILURE

            from tiago_door_planning.msg import ExecuteDoorOpeningGoal
            goal = ExecuteDoorOpeningGoal()
            goal.base_path = plan.base_path
            goal.base_times = list(plan.base_times)
            goal.arm_trajectory = plan.arm_trajectory
            goal.velocity_scaling = self.velocity_scaling
            rospy.loginfo(
                "  [%s] Sending ExecuteDoorOpening goal (%d waypoints, v_scale=%.2f)",
                self.name, len(plan.base_path.poses), self.velocity_scaling
            )
            self.client.send_goal(goal)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed > self.timeout_s:
            rospy.logwarn("  [%s] Execution timeout after %.1fs", self.name, elapsed)
            self.client.cancel_goal()
            return py_trees.common.Status.FAILURE

        state = self.client.get_state()
        if state == SUCCEEDED:
            result = self.client.get_result()
            if result and result.success:
                rospy.loginfo("  [%s] Execution succeeded: %s", self.name, result.message)
                return py_trees.common.Status.SUCCESS
            else:
                msg = result.message if result else "no result"
                rospy.logwarn("  [%s] Execution returned failure: %s", self.name, msg)
                return py_trees.common.Status.FAILURE
        elif state in (ABORTED, REJECTED, LOST, PREEMPTED):
            rospy.logwarn("  [%s] Execution action ended with state %d", self.name, state)
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
