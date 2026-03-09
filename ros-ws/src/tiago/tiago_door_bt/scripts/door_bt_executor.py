#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import py_trees

from bt_nodes import (
    WaitForPose,
    PublishEmptyOnce,
    WaitForMoveBaseResult,
    PublishBoolOnce,
    CallEmptyServiceOnce,
    CallSetBoolServiceOnce,
    DelaySeconds,
)

def make_tree(params):
    # Phase 1: Door model ready
    wait_plane = WaitForPose("WaitForPlanePose", params["plane_topic"], params["pose_max_age_s"])

    children_ready = [wait_plane]
    if params["require_handle_pose"]:
        wait_handle = WaitForPose("WaitForHandlePose", params["handle_topic"], params["pose_max_age_s"])
        children_ready.append(wait_handle)

    door_model_ready = py_trees.composites.Sequence(
        name="DoorModelReady",
        memory=True,
        children=children_ready,
    )

    # Phase 2: Navigate to pregrasp (with retries via Selector)
    nav_attempts = []
    for i in range(params["nav_retries"]):
        trigger_goal = PublishEmptyOnce(
            name="TriggerPregraspGoal_{}".format(i+1),
            topic=params["generate_goal_topic"],
            blackboard_key_stamp="last_trigger_stamp_{}".format(i+1),
        )

        wait_nav = WaitForMoveBaseResult(
            name="WaitForMoveBase_{}".format(i+1),
            status_topic=params["move_base_status_topic"],
            timeout_s=params["nav_timeout_s"],
            blackboard_key_stamp="last_trigger_stamp_{}".format(i+1),
        )

        nav_sequence = py_trees.composites.Sequence(
            name="NavigateToPregrasp_{}".format(i+1),
            memory=True,
            children=[trigger_goal, wait_nav],
        )
        nav_attempts.append(nav_sequence)

    # Selector tries each attempt until one succeeds
    nav_retry = py_trees.composites.Selector(
        name="RetryNavigation",
        memory=False,
        children=nav_attempts,
    )

    # Phase 3: Prep for door interaction (mask + clear)
    enable_mask = PublishBoolOnce("EnableDoorMask", params["door_mask_enable_topic"], True)

    clear_costmaps_1 = CallEmptyServiceOnce(
        "ClearCostmapsBeforeDoor",
        params["clear_costmaps_service"],
        timeout_s=2.0
    )

    prep_door = py_trees.composites.Sequence(
        name="PrepDoor",
        memory=True,
        children=[enable_mask, clear_costmaps_1],
    )
    
    freeze_srv = CallSetBoolServiceOnce(
    name="FreezeDoorwayPose",
    service_name="/doorway_pose_node/freeze",
    value=True,
    timeout_s=2.0,
    )
    
    # Phase 4: Door open (placeholder)
    # TODO: Planning 
    
    # Arm placeholder
    open_door = py_trees.behaviours.Success(name="OpenDoorPlaceholder")

    # Phase 5: Go through door (placeholder)
    go_through = py_trees.behaviours.Success(name="GoThroughPlaceholder")

    # Delay to observe door mask effect (for debugging/visualization)
    observe_mask = DelaySeconds("ObserveDoorMask", duration_s=5.0)

    # Phase 6: Restore (mask off + clear)
    disable_mask = PublishBoolOnce("DisableDoorMask", params["door_mask_enable_topic"], False)

    clear_costmaps_2 = CallEmptyServiceOnce(
        "ClearCostmapsAfterDoor",
        params["clear_costmaps_service"],
        timeout_s=2.0
    )

    restore = py_trees.composites.Sequence(
        name="RestoreNav",
        memory=True,
        children=[disable_mask, clear_costmaps_2],
    )

    # Root mission sequence
    root = py_trees.composites.Sequence(
        name="DoorMission",
        memory=True,
        children=[
            door_model_ready,
            nav_retry,
            prep_door,
            freeze_srv,
            open_door,
            go_through,
            observe_mask,
            restore,
        ],
    )

    return root

def main():
    rospy.init_node("tiago_door_bt_executor", anonymous=False)

    params = {
        "plane_topic": rospy.get_param("~plane_topic", "/door/plane_map"),
        "handle_topic": rospy.get_param("~handle_topic", "/door/handle_pose_map"),
        "require_handle_pose": bool(rospy.get_param("~require_handle_pose", True)),
        "pose_max_age_s": float(rospy.get_param("~pose_max_age_s", 0.5)),

        "generate_goal_topic": rospy.get_param("~generate_goal_topic", "/tiago_move_base_control/generate_goal"),

        "move_base_status_topic": rospy.get_param("~move_base_status_topic", "/tiago_move_base/move_base/status"),

        "nav_timeout_s": float(rospy.get_param("~nav_timeout_s", 60.0)),
        "nav_retries": int(rospy.get_param("~nav_retries", 3)),

        # Door mask + costmap clearing
        "door_mask_enable_topic": rospy.get_param("~door_mask_enable_topic", "/door_mask/enabled"),
        "clear_costmaps_service": rospy.get_param("~clear_costmaps_service", "/tiago_move_base/move_base/clear_costmaps"),

        "tick_hz": float(rospy.get_param("~tick_hz", 10.0)),
    }

    rospy.loginfo("=" * 60)
    rospy.loginfo("TIAGo Door BT Executor Starting")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Waiting for door poses on:")
    rospy.loginfo("  - Plane: %s", params["plane_topic"])
    if params["require_handle_pose"]:
        rospy.loginfo("  - Handle: %s (REQUIRED)", params["handle_topic"])
    rospy.loginfo("Navigation retries: %d (timeout: %.1fs)", params["nav_retries"], params["nav_timeout_s"])
    rospy.loginfo("Tick rate: %.1f Hz", params["tick_hz"])
    rospy.loginfo("=" * 60)

    tree = make_tree(params)
    tree.setup(timeout=2.0)

    # Track previous status to detect changes
    previous_status = {}
    
    def log_status_changes(tree_node):
        """Recursively log when nodes change status"""
        node_name = tree_node.name
        current_status = tree_node.status
        
        if node_name not in previous_status or previous_status[node_name] != current_status:
            if current_status == py_trees.common.Status.RUNNING:
                rospy.loginfo("[BT] >>> %s: RUNNING", node_name)
            elif current_status == py_trees.common.Status.SUCCESS:
                rospy.loginfo("[BT] ✓ %s: SUCCESS", node_name)
            elif current_status == py_trees.common.Status.FAILURE:
                rospy.logwarn("[BT] ✗ %s: FAILURE", node_name)
            previous_status[node_name] = current_status
        
        # Recursively check children
        if hasattr(tree_node, 'children'):
            for child in tree_node.children:
                log_status_changes(child)

    rospy.loginfo("[BT] Behavior tree initialized. Starting execution...")
    
    rate = rospy.Rate(params["tick_hz"])
    tick_count = 0
    mission_complete = False
    
    while not rospy.is_shutdown():
        tree.tick_once()
        log_status_changes(tree)
        
        # Check if mission completed successfully
        if tree.status == py_trees.common.Status.SUCCESS and not mission_complete:
            mission_complete = True
            rospy.loginfo("=" * 60)
            rospy.loginfo("[BT] *** MISSION COMPLETED SUCCESSFULLY ***")
            rospy.loginfo("=" * 60)
            break
        elif tree.status == py_trees.common.Status.FAILURE:
            rospy.logerr("=" * 60)
            rospy.logerr("[BT] *** MISSION FAILED ***")
            rospy.logerr("=" * 60)
            break
        
        # Log periodic heartbeat every 5 seconds
        tick_count += 1
        if tick_count % int(params["tick_hz"] * 5) == 0:
            rospy.loginfo("[BT] Heartbeat - Tree status: %s", tree.status)
        
        rate.sleep()
    
    rospy.loginfo("[BT] Behavior tree executor shutting down.")

if __name__ == "__main__":
    main()