cmake_minimum_required(VERSION 3.0.2)
project(tiago_door_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
)

catkin_package()

install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)