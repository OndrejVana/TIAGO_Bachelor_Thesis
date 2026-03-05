# TIAGo Development Workspace (ROS Melodic)

This workspace contains the development environment for TIAGo robot autonomy development based on ROS Melodic.

## Prerequisites

- Docker installed on your system
- At least 10GB of free disk space (build requires substantial space)
- At least 6GB of RAM (8GB+ strongly recommended for build process)

## Building the Docker Image

To build the Docker image locally with the latest version of all packages:

```bash
cd ros-ws
docker build -t tiago_dev_melodic:latest -f Dockerfile .
```

```bash
docker build -t tiago_dev_melodic_orb3:latest -f Dockerfile.opencv4 .
```

## Running the Container

[Rocker](https://github.com/osrf/rocker) provides the best GUI support and user management for Docker containers:

```bash
# Install rocker if not already installed
pip install rocker

xhost +local:

# Basic docker
rocker --x11 --home \
  --net=host \
  --volume "$(pwd):/workspace:rw" \
  --volume "$(pwd)/src:/tiago_dev_ws/src/custom:rw" \
  -- \
  tiago_dev_melodic:latest

# Docker for ORB3
rocker --x11 --home \
  --net=host \
  --volume "$(pwd):/workspace:rw" \
  --volume "$(pwd)/src:/tiago_dev_ws/src/custom:rw" \
  -- \
  tiago_dev_melodic_orb3:latest
```

## SSH

Connecting to the robot via ethernet.

```bash
  # Ethernet
  ssh root@tiago-114c

  #Over wifi
  ssh root@192.168.130.114

```

## Resources

- **TIAGo Tutorials**: [http://wiki.ros.org/Robots/TIAGo/Tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials)
- **TIAGo Repository**: [https://github.com/pal-robotics/tiago_tutorials](https://github.com/pal-robotics/tiago_tutorials)
- **PAL Robotics Documentation**: [http://wiki.ros.org/Robots/TIAGo](http://wiki.ros.org/Robots/TIAGo)
