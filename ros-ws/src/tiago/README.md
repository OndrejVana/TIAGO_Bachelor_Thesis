# TIAGo Development Workspace

ROS 1 (Melodic) workspace containing custom packages for autonomous door-opening with the TIAGo robot. The system covers perception, SLAM, navigation, arm manipulation, planning, and high-level behavior tree orchestration.

## Package overview

```text
ros-ws/src/tiago/
├── tiago_door_bringup/        # Unified launch entry point for the full system
├── tiago_door_sim/            # Gazebo world with hinged door + AprilTag models
├── tiago_door_localization/   # AprilTag-based door perception and doorway pose estimation
├── tiago_move_base_control/   # move_base wrapper and door approach goal generator
├── tiago_door_bt/             # py_trees behavior tree executive
├── tiago_door_planning/       # Action server for base/arm door-opening trajectory planning
├── tiago_arm_manipulation/    # MoveIt arm motion planning and gripper control
├── tiago_slam/                # SLAM implementations (slam_toolbox, RTAB-Map, ORB-SLAM2/3)
└── tiago_vicon/               # VICON bridge, position logger, path record/replay
```

## Quick start

```bash
# Source ROS + workspace
source /opt/ros/melodic/setup.bash
source /tiago_public_ws/devel/setup.bash
source /path/to/ros-ws/devel/setup.bash

# Full simulation run
roslaunch tiago_door_bringup main.launch

# Headless (no Gazebo GUI)
roslaunch tiago_door_bringup main.launch gui:=false
```

## Building

```bash
cd /path/to/ros-ws

# Optional: exclude packages you don't need
catkin config --blacklist darknet_ros_msgs darknet_ros

catkin build
source devel/setup.bash
```

## System architecture

```text
tiago_door_sim          -> Gazebo simulation (world + robot)
tiago_slam              -> Map + localization (laser/visual SLAM)
tiago_door_localization -> AprilTag → hinge/handle/doorway poses
tiago_move_base_control -> Door approach goal → move_base navigation
tiago_door_bt           -> Behavior tree: perception → navigate → prep → freeze
tiago_door_planning     -> Action server: base path + arm trajectory for door opening
tiago_arm_manipulation  -> MoveIt services: move_to_pose, door_pregrasp, gripper
tiago_vicon             -> Ground truth (VICON) + position logging + path replay
tiago_door_bringup      -> Orchestrates all of the above via main.launch
```

## Dependencies

| Category | Packages |
| --- | --- |
| Navigation | `move_base`, `amcl`, `map_server`, `navfn`, `base_local_planner` |
| SLAM | `slam_toolbox`, `rtabmap_ros`, `orb_slam2_ros`, `orb_slam3_ros` |
| Perception | `apriltag_ros`, `tf2_ros`, `tf2_geometry_msgs` |
| Manipulation | `moveit_commander`, `moveit_msgs`, `control_msgs`, `actionlib` |
| Behavior tree | `py_trees` |
| Simulation | `gazebo_ros`, PAL TIAGo public stack |

## Selected package READMEs

- [tiago_door_bringup](tiago_door_bringup/README.md) — full system launch guide
- [tiago_slam](tiago_slam/README.md) — all SLAM modes and real robot setup
- [tiago_door_localization](tiago_door_localization/README.md) — AprilTag perception pipeline
- [tiago_move_base_control](tiago_move_base_control/README.md) — navigation nodes
- [tiago_door_bt](tiago_door_bt/README.md) — behavior tree description
- [tiago_door_planning](tiago_door_planning/README.md) — planning action server
- [tiago_arm_manipulation](tiago_arm_manipulation/README.md) — arm + gripper services
- [tiago_door_sim](tiago_door_sim/README.md) — Gazebo simulation setup
- [tiago_vicon](tiago_vicon/README.md) — VICON bridge and position logging
