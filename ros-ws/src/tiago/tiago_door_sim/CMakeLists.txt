cmake_minimum_required(VERSION 3.0.2)
project(tiago_door_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  play_motion_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tuck_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)