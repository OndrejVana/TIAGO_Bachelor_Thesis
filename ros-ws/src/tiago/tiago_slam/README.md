# TIAGo SLAM Foundation

This package provides a foundation for autonomous SLAM (Simultaneous Localization and Mapping) development with the TIAGo robot. It combines laser sensor processing, mapping, and autonomous exploration capabilities.

## Implemented SLAM packages

### Tiago SLAM

- This is a launch file for final SLAM implementation that will be used for door openning.

### Basic Laser SLAM with Manual Control

- Simulation

```bash
# Launch basic SLAM in Small Office
roslaunch tiago_slam slam_foundation.launch world_name:="small_office"
```

- Real robot

```bash
# Launch on real TIAGo (disable simulation)
roslaunch tiago_slam slam_foundation.launch simulation:=false
```

- Common

```bash
# Rviz
rviz -d /tiago_dev_ws/src/custom/tiago/tiago_slam/rviz/slam_view.rviz

# In another terminal, control the robot manually
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel

# For laser SLAM (slam_toolbox)
rosnode kill /map_server

#Saving 2D map
rosrun map_server map_saver -f ~/maps/my_map
```

### RTAB SLAM

- Simulation

```bash
# Launch
roslaunch tiago_slam rtab_slam.launch world_name:="small_office"
```

- Real robot

```bash
# Launch on real TIAGo (disable simulation)
roslaunch tiago_slam rtab_slam.launch simulation:=false
```

- Common

```bash
# Rviz
rviz -d /tiago_dev_ws/src/custom/tiago/tiago_slam/rviz/rtab_slam.rviz

# Manual movement
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel

# For RTAB-Map - reset/clear the map (keeps database)
rosservice call /rtabmap/reset

# Stop rtabmap
rosnode kill /rtabmap

# Delete the database file
rm ~/.ros/rtabmap.db

#Saving 2D map
rosrun map_server map_saver -f ~/maps/my_map
```

### RTAB - MONO SLAM

- Simulation

```bash
# Launch
roslaunch tiago_slam rtab_mono_slam.launch world_name:="small_office"
```

- Real robot

```bash
# Launch on real TIAGo (disable simulation)
roslaunch tiago_slam rtab_mono_slam.launch simulation:=false
```

- Common

```bash
# Rviz
rviz -d /tiago_dev_ws/src/custom/tiago/tiago_slam/rviz/rtab_mono_slam.rviz

# Manual movement
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel

# For RTAB-Map - reset/clear the map (keeps database)
rosservice call /rtabmap/reset

# Stop rtabmap
rosnode kill /rtabmap

# Delete the database file
rm ~/.ros/rtabmap.db

#Saving 2D map
rosrun map_server map_saver -f ~/maps/my_map
```

### ORB2 SLAM - RGBD

- Simulation

```bash
# Launch
roslaunch tiago_slam orb_rgbd_slam.launch world_name:="small_office"
```

- Real robot

```bash
roslaunch tiago_slam orb_rgbd_slam.launch simulation:=false
```

- Common

```bash
# Rviz
rviz -d /tiago_dev_ws/src/custom/tiago/tiago_slam/rviz/orb_slam.rviz
```

```bash
# Maunal movement
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel

#Saving 2D map
rosrun map_server map_saver -f ~/maps/my_map
rosservice call /orb_slam2_rgbd/save_map map.bin
```

### ORB2 SLAM - MONO

- Simulation

```bash
# Launch
roslaunch tiago_slam orb_mono_slam.launch world_name:="small_office"
```

- Real robot

```bash
roslaunch tiago_slam orb_mono_slam.launch simulation:=false
```

- Common

```bash
# Rviz
rviz -d /tiago_dev_ws/src/custom/tiago/tiago_slam/rviz/orb_mono_slam.rviz
```

```bash
# Maunal movement
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel

#Saving 2D map
rosrun map_server map_saver -f ~/maps/my_map
rosservice call /orb_slam2_rgbd/save_map map.bin
```

### ORB3 SLAM - MONO

- Simulation

```bash
# Launch for simulation
roslaunch tiago_slam orb3_mono_slam.launch world_name:=small_office
```

- Real robot

```bash
# Run with real robot
roslaunch tiago_slam orb3_mono_slam.launch simulation:=false
```

- Common

```bash
# Rviz with config
rviz -d /tiago_dev_ws/src/custom/tiago/tiago_slam/rviz/orb3_mono_slam.rviz
```

```bash
# Maunal movement
rosrun key_teleop key_teleop.py /key_vel:=/mobile_base_controller/cmd_vel

#Saving 2D map
rosrun map_server map_saver -f ~/maps/my_map
rosservice call /tiago_orb_slam3_mono/save_map
```

- Debug

```bash
# If fails try this
apt-get remove libopencv-dev libopencv3.2
catkin config --blacklist orb_slam2_ros
```

## Real Robot

- First setup ROS master. For more information read manual from page 63 to 64.

```bash

# On the computer, set ROS master to the robot - Wifi/ Ethernet
export ROS_MASTER_URI=http://tiago-114c:11311

# Check you IP (hostname -I) - Ehternet
export ROS_IP=10.68.0.128

# Check you IP (hostname -I) - Wifi
export ROS_IP=192.168.251.123
```

- Sanity checkes if everything is running.

```bash
# Check laser data
rostopic echo /scan_raw

# Check odometry
rostopic echo /mobile_base_controller/odom

# Check TF tree
rosrun tf tf_echo base_footprint base_laser_link
```

## Troubleshooting

- Other Mapping is running

```bash
# Kill old mapping and maps server
rosnode kill /amcl /map_server /map_aligner_node /map_configuration_server /compressed_map_publisher
```

- If not moving even with teleop

```bash
rosnode kill /twist_mux 
```

- Time fixing

```bash
# After turing off automatic time on laptop
 sudo ntpdate -u tiago-114c

# Or setup chrony
chronyc tracking
 ```
