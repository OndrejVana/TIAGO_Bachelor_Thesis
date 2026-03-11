# tiago_arm_manipulation

ROS 1 (Melodic) package providing arm motion planning and gripper control for the TIAGo robot. It wraps MoveIt to expose simple ROS services for moving the arm to a Cartesian pose, a named configuration, or a door handle pregrasp position.

## Package structure

```text
tiago_arm_manipulation/
├── config/
│   └── arm_manipulation.yaml     # Default parameters
├── launch/
│   └── arm_manipulation.launch   # Loads config and starts the node
├── scripts/
│   └── arm_manipulation_node.py  # Main ROS node
├── srv/
│   ├── MoveToPose.srv
│   ├── MoveToNamed.svr
│   └── DoorPregrasp.srv
└── package.xml
```

## Dependencies

| Package | Purpose |
| --- | --- |
| `rospy` | Python ROS client |
| `moveit_commander` / `moveit_msgs` | Motion planning |
| `actionlib` / `control_msgs` | Gripper control via `GripperCommandAction` |
| `tf2_ros` / `tf2_geometry_msgs` | Frame transforms |
| `geometry_msgs`, `std_msgs`, `std_srvs`, `trajectory_msgs` | Message types |

## Launching

```bash
roslaunch tiago_arm_manipulation arm_manipulation.launch
```

The launch file loads `config/arm_manipulation.yaml` and starts `arm_manipulation_node.py`.

## Services

All services are advertised under the node's private namespace (`~`), i.e. `/tiago_arm_manipulation/<service>`.

| Service | Type | Description |
| --- | --- | --- |
| `~move_to_pose` | `MoveToPose` | Plan (and optionally execute) a move to a `PoseStamped` target |
| `~move_to_named` | `MoveToNamed` | Plan (and optionally execute) a move to a named SRDF configuration |
| `~door_pregrasp` | `DoorPregrasp` | Compute and optionally execute a pregrasp pose relative to a detected door handle |
| `~home` | `std_srvs/Trigger` | Move arm to the named `home` configuration |
| `~stow` | `std_srvs/Trigger` | Move arm to the named `stow` configuration |
| `~gripper_open` | `std_srvs/Trigger` | Open the gripper |
| `~gripper_close` | `std_srvs/Trigger` | Close the gripper |

### Service definitions

#### MoveToPose.srv

```text
geometry_msgs/PoseStamped target
float64 position_tolerance       # 0 → use default
float64 orientation_tolerance    # 0 → use default
float64 velocity_scaling         # 0 → use default
float64 acceleration_scaling     # 0 → use default
bool execute                     # false → plan only
---
bool ok
string message
```

#### MoveToNamed.srv

```text
string name
float64 velocity_scaling
float64 acceleration_scaling
bool execute
---
bool ok
string message
```

#### DoorPregrasp.srv

```text
bool use_latest_handle                       # use last received handle pose
geometry_msgs/PoseStamped handle_override    # used when use_latest_handle=false
float64 approach_distance
float64 lateral_offset
float64 vertical_offset
float64 velocity_scaling
float64 acceleration_scaling
bool execute
---
bool ok
string message
geometry_msgs/PoseStamped planned_target_base
```

### Service Example 

```bash
rosservice call /tiago_arm_manipulation/door_pregrasp "
> use_latest_handle: true
> handle_override:
>   header:
>     frame_id: ''
>   pose:
>     position: {x: 0.0, y: 0.0, z: 0.0}
>     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
> approach_distance: 0.0
> lateral_offset: 0.0
> vertical_offset: 0.0
> velocity_scaling: 0.0
> acceleration_scaling: 0.0
> execute: true
> "
```

## Subscribed topics

| Topic | Type | Description |
| --- | --- | --- |
| `/door/handle_pose_map` (configurable) | `geometry_msgs/PoseStamped` | Door handle pose used by `door_pregrasp` |

## Parameters (`config/arm_manipulation.yaml`)

| Parameter | Default | Description |
| --- | --- | --- |
| `move_group_name` | `"arm"` | MoveIt move group name |
| `ee_link` | `""` | End-effector link (empty = MoveIt default) |
| `base_frame_fallback` | `"base_footprint"` | Fallback planning frame |
| `handle_topic` | `"/door/handle_pose_map"` | Handle pose topic |
| `position_tolerance` | `0.01` | Goal position tolerance (m) |
| `orientation_tolerance` | `0.05` | Goal orientation tolerance (rad) |
| `velocity_scaling` | `0.2` | MoveIt velocity scaling factor |
| `acceleration_scaling` | `0.2` | MoveIt acceleration scaling factor |
| `approach_distance` | `0.12` | Pregrasp standoff distance (m) |
| `lateral_offset` | `0.0` | Lateral pregrasp offset (m) |
| `vertical_offset` | `0.0` | Vertical pregrasp offset (m) |
| `named_home` | `"home"` | SRDF name for the home pose |
| `named_stow` | `"stow"` | SRDF name for the stow pose |
