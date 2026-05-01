# TIAGo++ Robot Deployment Guide

Guide for running bandwidth-heavy nodes (SLAM, perception) on the robot via Docker to reduce WiFi load.

## Architecture

| Node | Where | Reason |
| --- | --- | --- |
| `rtabmap`, `rgbd_odometry`, `depthimage_to_laserscan` | **Robot (Docker)** | Eliminates raw image stream over WiFi |
| `apriltag_detector`, door perception nodes | **Robot (Docker)** | Eliminates raw image stream over WiFi |
| `move_base`, `door_approach_goal_generator`, `move_base_goal_sender` | **Robot (Docker)** | TF lookups require robot clock — laptop clock skew causes 45 s costmap timeouts; `DoorMaskLayer` is a compiled C++ plugin that must run on-robot |
| Behavior tree, planning, arm control | Laptop | Lightweight, no sensor data |
| RViz | Laptop | Needs display |

---

## One-Time Setup

### Step 0 — Check prerequisites on the robot

```bash
ssh pal@tiago-114c

# Check Docker is available
docker --version

# Check ros_docker_wrapper is installed
rospack find ros_docker_wrapper

# If ros_docker_wrapper is missing, upgrade (as root):
# sudo pal_upgrade
exit
```

### Step 1 — Save and transfer Docker image (~10-20 min, use Ethernet)

```bash
# On your laptop — save image to file (~3-5 GB)
docker save -o ~/tiago_dev_melodic.docker tiago_dev_melodic:latest

# Transfer to robot
scp ~/tiago_dev_melodic.docker pal@tiago-114c:/tmp/

# Load on robot (stored in /home/pal/docker_dir — survives reboots)
ssh pal@tiago-114c "docker load -i /tmp/tiago_dev_melodic.docker"

# Verify
ssh pal@tiago-114c "docker images"

# Clean up
ssh pal@tiago-114c "rm /tmp/tiago_dev_melodic.docker"
rm ~/tiago_dev_melodic.docker
```

### Step 2 — Create workspace and transfer code

```bash
# Create workspace on robot
ssh root@tiago-114c "mkdir -p /home/pal/tiago_ws/src"

# Sync code from repo root
rsync -av \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='.pytest_cache/' \
  --exclude='.git' \
  ros-ws/src/tiago/ root@tiago-114c:/home/pal/tiago_ws/src/
```

### Step 3 — Build custom packages on robot (one-time, fast — Python only)

```bash
ssh root@tiago-114c

# Create build script on the robot
cat << 'EOF' > /tmp/build_ws.sh
#!/bin/bash
set -e
cd /tiago_dev_ws
catkin config --extend /opt/ros/melodic --blacklist tiago_door_sim
catkin build
echo 'Build OK'
EOF

# Run it inside Docker
docker run --rm --net=host \
  -v /home/pal/tiago_ws/src:/tiago_dev_ws/src/custom \
  -v /home/pal/tiago_ws/build:/tiago_dev_ws/build \
  -v /home/pal/tiago_ws/devel:/tiago_dev_ws/devel \
  -v /tmp/build_ws.sh:/build_ws.sh \
  tiago_dev_melodic:latest bash /build_ws.sh
```

> Build output is saved in `/home/pal/tiago_ws/build` and `devel` (persistent between reboots).
> Python file changes do **not** require a rebuild — only rebuild if adding new packages or files.

### Step 4 — Create helper scripts on robot

```bash
# SLAM script
ssh root@tiago-114c "cat > /home/pal/run_slam.sh << 'EOF'
#!/bin/bash
docker run --rm --net=host \
  -v /home/pal/tiago_ws/src:/tiago_dev_ws/src/custom \
  -v /home/pal/tiago_ws/build:/tiago_dev_ws/build \
  -v /home/pal/tiago_ws/devel:/tiago_dev_ws/devel \
  tiago_dev_melodic:latest \
  bash -c 'source /tiago_dev_ws/devel/setup.bash && roslaunch tiago_slam tiago_slam.launch'
EOF
chmod +x /home/pal/run_slam.sh"

# Perception script
ssh root@tiago-114c "cat > /home/pal/run_perception.sh << 'EOF'
#!/bin/bash
docker run --rm --net=host \
  -v /home/pal/tiago_ws/src:/tiago_dev_ws/src/custom \
  -v /home/pal/tiago_ws/build:/tiago_dev_ws/build \
  -v /home/pal/tiago_ws/devel:/tiago_dev_ws/devel \
  tiago_dev_melodic:latest \
  bash -c 'source /tiago_dev_ws/devel/setup.bash && roslaunch tiago_door_localization door_preception_tags.launch'
EOF
chmod +x /home/pal/run_perception.sh"

# Navigation script
ssh root@tiago-114c "cat > /home/pal/run_navigation.sh << 'EOF'
#!/bin/bash
docker run --rm --net=host \
  -v /home/pal/tiago_ws/src:/tiago_dev_ws/src/custom \
  -v /home/pal/tiago_ws/build:/tiago_dev_ws/build \
  -v /home/pal/tiago_ws/devel:/tiago_dev_ws/devel \
  tiago_dev_melodic:latest \
  bash -c 'source /tiago_dev_ws/devel/setup.bash && roslaunch tiago_move_base_control door_nav.launch'
EOF
chmod +x /home/pal/run_navigation.sh"
```

> `DoorMaskLayer` is a compiled C++ costmap plugin (`libdoor_mask_layer.so`). It is built during `catkin build` and loaded by `move_base` at runtime via `plugin.xml`. Running on the robot ensures the plugin binary matches the robot's ROS/costmap_2d version and that TF lookups use the robot's clock directly — avoiding the ~45 s clock-skew timeout seen when running `move_base` on a remote laptop.

---

## Daily Workflow

### 1. Sync code changes to robot (if you changed any files)

```bash
# Sync everything
rsync -av \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='.pytest_cache/' \
  --exclude='.git' \
  ros-ws/src/tiago/ root@tiago-114c:/home/pal/tiago_ws/src/

# Or sync specific packages only
rsync -av --exclude='__pycache__/' --exclude='*.pyc' \
  ros-ws/src/tiago/tiago_slam/ root@tiago-114c:/home/pal/tiago_ws/src/tiago_slam/
rsync -av --exclude='__pycache__/' --exclude='*.pyc' \
  ros-ws/src/tiago/tiago_door_localization/ root@tiago-114c:/home/pal/tiago_ws/src/tiago_door_localization/
rsync -av --exclude='__pycache__/' --exclude='*.pyc' \
  ros-ws/src/tiago/tiago_move_base_control/ root@tiago-114c:/home/pal/tiago_ws/src/tiago_move_base_control/
```

**Note:** `tiago_move_base_control` contains a C++ plugin (`DoorMaskLayer`). After syncing, rebuild if you changed any `.cpp` or `.h` file. Python-only changes (scripts/, config/, launch/) do **not** require a rebuild.

```bash
ssh root@tiago-114c "docker run --rm --net=host \
  -v /home/pal/tiago_ws/src:/tiago_dev_ws/src/custom \
  -v /home/pal/tiago_ws/build:/tiago_dev_ws/build \
  -v /home/pal/tiago_ws/devel:/tiago_dev_ws/devel \
  tiago_dev_melodic:latest \
  bash -c 'source /opt/ros/melodic/setup.bash && cd /tiago_dev_ws && catkin build tiago_slam'"
```

### 2. Run nodes on robot (three SSH terminals)

```bash
# Terminal 1 — SLAM
ssh root@tiago-114c "/home/pal/run_slam.sh"

# Terminal 2 — Door perception
ssh root@tiago-114c "/home/pal/run_perception.sh"

# Terminal 3 — Navigation (move_base + door approach)
ssh root@tiago-114c "/home/pal/run_navigation.sh"
```

### 3. Launch remaining nodes on your laptop

```bash
export ROS_MASTER_URI=http://tiago-114c:11311
# over wifi
export ROS_IP=192.168.251.123 
# over ethernet
export ROS_IP=10.68.0.128

roslaunch tiago_door_bringup main.launch \
  simulation:=false \
  use_slam:=false \
  use_door_perception:=false \
  use_navigation:=false
```

> `use_navigation:=false` skips launching `tiago_move_base_control` on the laptop since it is now running on the robot.

---

## Network Reference

| Parameter | Value |
| --- | --- |
| Robot hostname | `tiago-114c` |
| SSH (Ethernet) | `ssh root@tiago-114c` |
| SSH (WiFi) | `ssh root@tiago-114c` |
| ROS master | `http://tiago-114c:11311` |

> See manual §10.4 for ROS communication setup and §9.4 for DNS if hostname resolution fails.

---

## Troubleshooting

**Hostname not resolved by robot:**

```bash
# On robot, add your laptop to its DNS
ssh root@tiago-114c "addLocalDns -u mylaptop -i <laptop-ip>"
# Or just use ROS_IP on your laptop instead of hostname
export ROS_IP=<your-laptop-ip>
```

**ROS nodes can't communicate:**

- Make sure both robot and laptop can ping each other by hostname
- If not, use `ROS_IP` (laptop IP visible to robot) instead of relying on hostname resolution

**Check what is running on the robot:**

```bash
export ROS_MASTER_URI=http://tiago-114c:11311
rosnode list
rostopic list
```

**Chrony time server:**

```bash
# STEP 0:
ssh root@tiago-114c

# STEP 1:
sudo apt update
sudo apt install chrony -y

# STEP 2:
sudo nano /etc/chrony/chrony.conf

# STEP 3:
allow 10.68.0.0/24
local stratum 10

# STEP 4:
sudo systemctl restart chrony
sudo systemctl enable chrony

# STEP 5: (Localy)
sudo chronyc makestep
sudo systemctl restart chrony

# Verify
chronyc sources -v
chronyc tracking
```
