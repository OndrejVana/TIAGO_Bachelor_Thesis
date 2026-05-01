# TIAGo++ Door Bringup Package

Main entry point for launching the complete TIAGo++ dual-arm door navigation system.

## Overview

This package provides a unified launch file (`main.launch`) that orchestrates all components of the door navigation project:

- **Simulation/Robot**: Gazebo simulation or real robot bringup
- **SLAM**: Mapping and localization using tiago_slam
- **Door Perception**: AprilTag-based door detection and modeling
- **Navigation**: Move_base control for autonomous navigation
- **Behavior Tree**: High-level door approach coordination
- **VICON**: Optional motion capture ground truth

## Quick Start

### Basic Simulation with All Features

```bash
# Launch
roslaunch tiago_door_bringup main.launch

# Clean Map before each run
rm ~/.ros/rtabmap.db
```

This will launch:

- Gazebo simulation with TIAGo and door world
- SLAM system
- AprilTag door detection
- Navigation control
- Behavior tree executor

### Customized Launch

You can enable/disable components and configure settings via arguments:

```bash
# Disable behavior tree, just run perception and navigation
roslaunch tiago_door_bringup main.launch use_behavior_tree:=false

# Run only SLAM, no door perception or navigation
roslaunch tiago_door_bringup main.launch \
  use_door_perception:=false \
  use_navigation:=false \
  use_behavior_tree:=false

# Different robot model (tiago++ is default; use titanium/steel/iron for single-arm)
roslaunch tiago_door_bringup main.launch robot:=titanium

# With VICON ground truth
roslaunch tiago_door_bringup main.launch use_vicon:=true
```

## Launch Arguments

### Global Configuration

- `simulation` (default: `true`) - Use Gazebo simulation vs real robot
- `use_slam` (default: `true`) - Enable SLAM system
- `use_door_perception` (default: `true`) - Enable AprilTag door detection
- `use_navigation` (default: `true`) - Enable navigation control
- `use_behavior_tree` (default: `true`) - Enable BT-based door approach
- `use_vicon` (default: `false`) - Enable VICON motion capture
- `rviz` (default: `true`) - Launch RViz visualization

### Simulation Settings

- `world_file` (default: TIAGo door world) - Gazebo world file path
- `robot` (default: `tiago++`) - Robot model: `tiago++` (dual-arm), `titanium`, `steel`, or `iron`
- `gui` (default: `true`) - Show Gazebo GUI
- `gzpose` (default: origin) - Initial robot spawn pose

### Door Perception

- `door_config` (default: two-sided door config) - AprilTag configuration YAML

### Navigation

- `approach_cfg` (default: door approach config) - Door approach parameters

## Component Packages

The bringup integrates these packages:

- `tiago_door_sim` - Gazebo simulation environment
- `tiago_slam` - SLAM implementations
- `tiago_door_localization` - AprilTag-based door perception and doorway pose estimation
- `tiago_move_base_control` - Navigation goal sender and door approach goal generator
- `tiago_door_bt` - Behavior tree executor for door-opening pipeline
- `tiago_door_planning` - Action server for base/arm door-opening trajectory planning
- `tiago_arm_manipulation` - MoveIt-based arm motion and gripper control
- `tiago_vicon` - Motion capture integration and position logging

## Usage Examples

### 1. Testing Door Detection Only

```bash
# Launch sim + perception, no autonomous navigation
roslaunch tiago_door_bringup main.launch \
  use_navigation:=false \
  use_behavior_tree:=false

# Manually teleop robot to see door detection
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel
```

### 2. SLAM Testing

```bash
# Just SLAM and robot
roslaunch tiago_door_bringup main.launch \
  use_door_perception:=false \
  use_navigation:=false \
  use_behavior_tree:=false
```

```bash
# Just SLAM and robot
roslaunch tiago_door_bringup main.launch use_door_perception:=false
```

### 3. Complete Autonomous Run

```bash
roslaunch tiago_door_bringup main.launch
```

### 4. Real Robot Deployment

```bash
roslaunch tiago_door_bringup main.launch simulation:=false use_door_perception:=false use_navigation:=false use_arm_manipulation:=false
```

- Set ROS master to the robot (see robot manual, pages 63–64):

```bash
export ROS_MASTER_URI=http://tiago-114c:11311

# Ethernet
export ROS_IP=10.68.0.128


# Wi-Fi
export ROS_IP=192.168.251.123
```

- For detailed information about packgage deployment refer to [ROBOT_PKG_DEPLOYMENT.md](../../../../ROBOT_PKG_DEPLOYMENT.md)

## RViz Configuration

A default RViz configuration is available at `rviz/tiago_door_nav.rviz`. Create your own:

```bash
rviz
# Configure displays, save to: src/tiago/tiago_door_bringup/rviz/tiago_door_nav.rviz
```
