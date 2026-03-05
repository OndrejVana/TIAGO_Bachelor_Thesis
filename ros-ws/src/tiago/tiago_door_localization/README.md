# tiago_door_localization

ROS 1 (Melodic) package for AprilTag-based door perception. Detects door-mounted AprilTags, estimates hinge/handle poses in the map frame, derives a stable doorway pose, and optionally publishes a costmap-clearing mask so the door opening can be navigated cleanly.

## Package structure

```text
tiago_door_localization/
├── config/
│   └── apriltag_two_sided_door.yaml   # Tag IDs, sizes, side mapping
├── launch/
│   └── door_preception_tags.launch    # Main launch file
└── scripts/
    ├── door_tag_pose_node.py          # AprilTag → hinge/handle PoseStamped
    ├── door_model_from_tag_node.py    # Builds door plane/model from tag poses
    └── doorway_pose_node.py           # Fuses hinge+handle → stable doorway pose
```

## Dependencies

`rospy`, `tf2_ros`, `tf2_geometry_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `apriltag_ros`

## Launch

```bash
roslaunch tiago_door_localization door_preception_tags.launch
```

## Nodes

### `door_tag_pose_node`

Subscribes to `/tag_detections` (from `apriltag_ros`) and publishes the detected tag pose in both `base_link` and `map` frames. Supports multi-tag doors with per-side semantic labels (front/back, push/pull).

#### Subscribed topics - door tag

| Topic | Type | Description |
| --- | --- | --- |
| `/tag_detections` (configurable) | `apriltag_ros/AprilTagDetectionArray` | Raw tag detections from camera |

#### Published topics - door tag

| Topic | Type | Description |
| --- | --- | --- |
| `/door/handle_pose_map` | `geometry_msgs/PoseStamped` | Handle pose in map frame |
| `/door/hinge_pose_map` | `geometry_msgs/PoseStamped` | Hinge pose in map frame |

### `door_model_from_tag_node`

Builds a geometric door plane model from the detected tag poses and publishes it for use by the navigation and planning nodes.

#### Published topics - door plane

| Topic | Type | Description |
| --- | --- | --- |
| `/door/plane_map` | `geometry_msgs/PoseStamped` | Door plane normal/origin in map frame |

### `doorway_pose_node`

Fuses the hinge and handle poses into a single stable `doorway_pose` using a sliding-window filter with circular statistics. Auto-freezes when pose stability thresholds are met.

#### Subscribed topics - doorway

| Topic | Type | Description |
| --- | --- | --- |
| `/door/hinge_pose_map` | `geometry_msgs/PoseStamped` | |
| `/door/handle_pose_map` | `geometry_msgs/PoseStamped` | |

#### Published topics - doorway

| Topic | Type | Description |
| --- | --- | --- |
| `/door/doorway_pose_map` | `geometry_msgs/PoseStamped` | Fused, stabilized doorway pose |

#### Services - doorway

| Service | Type | Description |
| --- | --- | --- |
| `~freeze` | `std_srvs/SetBool` | Freeze (`true`) or unfreeze (`false`) the published doorway pose |

## Services quick reference

```bash
# Freeze the doorway pose (lock current estimate)
rosservice call /doorway_pose_node/freeze "data: true"

# Unfreeze (resume updating)
rosservice call /doorway_pose_node/freeze "data: false"

# Enable costmap door mask
rostopic pub -1 /door_mask/enabled std_msgs/Bool "data: true"

# Disable costmap door mask
rostopic pub -1 /door_mask/enabled std_msgs/Bool "data: false"
```

## Configuration (`config/apriltag_two_sided_door.yaml`)

| Parameter | Default | Description |
| --- | --- | --- |
| `apriltag/tag_family` | `tag36h11` | AprilTag family |
| `apriltag/tag_size_m` | `0.18` | Physical tag size (m) |
| `apriltag/tag_ids` | `[0, 1]` | IDs of tags mounted on the door |
| `apriltag/side_by_id` | `{0: front, 1: back}` | Map tag ID → logical door side |
| `apriltag/interaction_by_side` | `{front: pull, back: push}` | Expected interaction per side |
| `apriltag/preferred_side` | `""` | Prefer this side when both visible |
