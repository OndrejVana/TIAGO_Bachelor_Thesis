# tiago_move_base_control

ROS 1 (Melodic) package that wraps `move_base` navigation for TIAGo and provides a door-approach goal generator. It exposes a simple topic-based interface so other nodes (e.g. the behavior tree) can send navigation goals without directly calling the `move_base` action API.

## Package structure

```text
tiago_move_base_control/
├── config/
│   └── door_approach.yaml              # Door approach generator parameters
├── launch/
│   ├── tiago_move_base.launch          # Full move_base stack with costmaps
│   ├── move_base_control.launch        # Goal sender node only
│   └── door_nav.launch                 # Door approach generator node
├── scripts/
│   ├── move_base_goal_sender.py        # Forwards PoseStamped goals to move_base action
│   └── door_approach_goal_generator_node.py  # Computes pregrasp goal from door pose
├── src/                                # C++ costmap plugin
├── include/                            # C++ plugin headers
└── plugin.xml                          # Costmap layer plugin registration
```

## Dependencies

`rospy`, `roscpp`, `geometry_msgs`, `std_msgs`, `actionlib`, `actionlib_msgs`, `move_base_msgs`, `costmap_2d`, `pluginlib`, `tf2`

## Launch

```bash
# Full move_base stack (planners + costmaps + goal sender)
roslaunch tiago_move_base_control tiago_move_base.launch

# Door approach goal generator only
roslaunch tiago_move_base_control door_nav.launch
```

## Nodes

### `move_base_goal_sender`

Listens for a `PoseStamped` goal on a topic and forwards it to the `move_base` action server. Also relays `move_base` status for monitoring.

| Topic | Type | Description |
| --- | --- | --- |
| `/tiago_move_base_control/goal` (sub) | `geometry_msgs/PoseStamped` | Incoming navigation goal |
| `/tiago_move_base_control/cancel` (sub) | `std_msgs/Empty` | Cancel all active goals |
| `~status` (pub) | `actionlib_msgs/GoalStatusArray` | Forwarded move_base status |

### `door_approach_goal_generator_node`

Computes a pregrasp navigation goal in front of the detected door using the door plane normal and (optionally) the handle pose. Publishes the goal when triggered.

| Topic | Type | Description |
| --- | --- | --- |
| `/door/plane_map` (sub) | `geometry_msgs/PoseStamped` | Door plane normal in map frame |
| `/door/handle_pose_map` (sub) | `geometry_msgs/PoseStamped` | Door handle pose |
| `/tiago_move_base_control/generate_goal` (sub) | `std_msgs/Empty` | Trigger goal computation |
| `/tiago_move_base_control/goal` (pub) | `geometry_msgs/PoseStamped` | Computed navigation goal |

## Parameters (`config/door_approach.yaml`)

| Parameter | Default | Description |
| --- | --- | --- |
| `plane_topic` | `/door/plane_map` | Door plane topic |
| `handle_topic` | `/door/handle_pose_map` | Handle pose topic |
| `goal_topic` | `/tiago_move_base_control/goal` | Navigation goal output topic |
| `trigger_topic` | `/tiago_move_base_control/generate_goal` | Goal trigger topic |
| `stand_off_m` | `0.75` | Standoff distance from door (m) |
| `lateral_offset_m` | `0.0` | Lateral offset from door centre (m) |
| `use_handle_as_target` | `true` | Use handle position as goal target (vs door centre) |
| `face_door` | `true` | Orient robot to face the door at goal |
| `normal_sign` | `-1` | Sign of door normal for goal direction |
| `continuous` | `false` | Continuously publish goals when new poses arrive |
| `min_update_period_s` | `0.5` | Minimum time between continuous goal publications (s) |

## `move_base` configuration (set in `tiago_move_base.launch`)

- **Namespace:** `/tiago_move_base/move_base`
- **Global planner:** `navfn/NavfnROS`
- **Local planner:** `base_local_planner/TrajectoryPlannerROS`
- **Velocity command topic:** remapped to `/mobile_base_controller/cmd_vel`
- **Max linear velocity:** `0.5 m/s` | **Max angular velocity:** `1.0 rad/s`
