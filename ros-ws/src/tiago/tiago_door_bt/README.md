# tiago_door_bt

ROS 1 (Melodic) package that implements a `py_trees`-based behavior tree (BT) executive for the door-opening pipeline. It orchestrates perception readiness, navigation to a pregrasp pose, costmap preparation, and doorway pose freezing in a structured, retry-capable sequence.

## Package structure

```text
tiago_door_bt/
├── launch/
│   └── door_bt.launch         # Starts the BT executor node
└── scripts/
    ├── door_bt_executor.py    # Builds and ticks the behavior tree
    └── bt_nodes.py            # Custom py_trees behaviour nodes
```

## Dependencies

`rospy`, `std_msgs`, `geometry_msgs`, `actionlib_msgs`, `py_trees`

## Launch

```bash
roslaunch tiago_door_bt door_bt.launch
```

## Behavior tree overview

The tree runs at a configurable tick rate (`tick_hz`, default `10 Hz`) and executes the following phases in sequence:

```text
Root (Sequence)
├── DoorModelReady (Sequence)        # Wait for door plane pose (+ optional handle)
├── RetryNavigation (Retry)          # Navigate to pregrasp, retry up to N times
│   └── NavigateToPregrasp (Sequence)
│       ├── TriggerPregraspGoal      # Publish Empty to generate_goal topic
│       └── WaitForMoveBase          # Wait for move_base to report success/failure
├── PrepDoor (Sequence)              # Enable door mask + clear costmaps
│   ├── EnableDoorMask
│   └── ClearCostmapsBeforeDoor
└── FreezeDoorwayPose                # Call /doorway_pose_node/freeze true
```

## Custom BT nodes (`bt_nodes.py`)

| Node | Type | Description |
| --- | --- | --- |
| `WaitForPose` | Behaviour | Returns SUCCESS when a `PoseStamped` topic has been received within `max_age_s` |
| `PublishEmptyOnce` | Behaviour | Publishes `std_msgs/Empty` once and records a timestamp on the blackboard |
| `WaitForMoveBaseResult` | Behaviour | Polls `move_base` status; returns SUCCESS on SUCCEEDED, FAILURE on ABORTED/REJECTED/LOST |
| `PublishBoolOnce` | Behaviour | Publishes a `std_msgs/Bool` once |
| `CallEmptyServiceOnce` | Behaviour | Calls a `std_srvs/Empty` service once with a timeout |
| `CallSetBoolServiceOnce` | Behaviour | Calls a `std_srvs/SetBool` service once with a configurable value |

## Node parameters (set in `door_bt.launch`)

| Parameter | Default | Description |
| --- | --- | --- |
| `plane_topic` | `/door/plane_map` | Door plane pose topic to wait for |
| `handle_topic` | `/door/handle_pose_map` | Handle pose topic (required if `require_handle_pose=true`) |
| `require_handle_pose` | `true` | Also wait for handle pose before navigating |
| `pose_max_age_s` | `0.5` | Maximum acceptable age of a pose (s) |
| `generate_goal_topic` | `/tiago_move_base_control/generate_goal` | Topic used to trigger goal generation |
| `move_base_status_topic` | `/tiago_move_base/move_base/status` | move_base status topic |
| `nav_timeout_s` | `60.0` | Navigation timeout (s) |
| `nav_retries` | `3` | Number of navigation retry attempts |
| `door_mask_enable_topic` | `/door_mask/enabled` | Topic to enable the door costmap mask |
| `clear_costmaps_service` | `/tiago_move_base/move_base/clear_costmaps` | Service to clear costmaps |
| `tick_hz` | `10.0` | BT tick frequency (Hz) |

## Integration

This package sits at the top of the pipeline and depends on:

- `tiago_door_localization` — provides `/door/plane_map` and `/door/handle_pose_map`
- `tiago_move_base_control` — provides the goal trigger and `move_base` navigation
- `tiago_door_localization` (`doorway_pose_node`) — exposes the `/doorway_pose_node/freeze` service
