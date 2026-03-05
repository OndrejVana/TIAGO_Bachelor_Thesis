cmake_minimum_required(VERSION 3.0.2)
project(tiago_arm_manipulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  actionlib
  control_msgs
  trajectory_msgs
  moveit_commander
  moveit_msgs
  message_generation
)

add_service_files(
  FILES
    MoveToPose.srv
    MoveToNamed.srv
    DoorPregrasp.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy geometry_msgs std_msgs std_srvs
    tf2_ros tf2_geometry_msgs
    actionlib control_msgs trajectory_msgs
    moveit_commander moveit_msgs
    message_runtime
)

catkin_install_python(PROGRAMS
  scripts/arm_manipulation_node.py
  scripts/test_calls.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)