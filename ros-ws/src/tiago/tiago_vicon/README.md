# Vicon bridge and position tracking package

Simple ROS package for getting and logging robot's position from multiple sources.

## Vicon

Vicon is used to get ground true position of the robot. Client needs to be running to get all the important information. It needs to be on the same network as vicon with IP 192.168.2.100.

- Starting the bridge

```bash
# Vicon bridge 
roslaunch tiago_vicon tiago_vicon.launch
```

- Vicon position output can be recorded with following

```bash
# Simple rosbag vicon record
rosbag record -O vicon_all \
  /vrpn_client_node/tiago/pose \
  /vrpn_client_node/tiago/twist
```

## Recording Position Data

This is used for analysis of different runs. It saves positions from /slam_pose and /vicon topic.

```bash
# Basic usage (ORB-SLAM2 pose vs Vicon)
roslaunch tiago_vicon position_logger.launch

# Using odometry instead of SLAM pose
roslaunch tiago_vicon position_logger.launch use_odom:=true

# Custom output directory and log rate
roslaunch tiago_vicon position_logger.launch output_dir:=... log_rate:=20.0
```

### Debug

```bash
# Install this
sudo apt-get install -y netbase

# And run this
getent protocols tcp
```

## Path record and replay

These nodes are used for recording and replaing path. Compatible with all slam implementations.

### Recording

- Make it executable file

```bash
# Making it an executable script
chmod +x /tiago_dev_ws/src/custom/tiago/tiago_vicon/src/path_recorder.py
```

- Start the recording node which subscribes to odom and slam_pose

```bash
# Starting the node 
rosrun tiago_vicon path_recorder.py
```

- To start recodring use this service

```bash
# Service to start the recording
rosservice call /path_recorder/start_recording
```

- To stop the recoring and save the path into yaml file use following service

```bash
# Service to stop the recording
rosservice call /path_recorder/stop_recording
```

### Replay

- Start the replaying node which reads the path from provided file

```bash
# Starts the replay node 
rosrun tiago_vicon path_replay.py _path_file:=/tiago_dev_ws/recorded_paths/...
```

- To start replaying use following service

```bash
# Service to start replaying
rosservice call /path_replay/start_replay
```

- To stop replaying use following service

```bash
# Service to stop replaying
rosservice call /path_replay/stop_replay
```

### Rviz

Path can be visualized in Rviz. It's being publish to /path_replay/recorded_path topic.
