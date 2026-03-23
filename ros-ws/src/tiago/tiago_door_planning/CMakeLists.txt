cmake_minimum_required(VERSION 3.0.2)
project(tiago_door_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
)

catkin_python_setup()

add_action_files(
  FILES
    PlanDoorOpening.action
    ExecuteDoorOpening.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    nav_msgs
    trajectory_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy actionlib actionlib_msgs message_runtime
    geometry_msgs nav_msgs trajectory_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/door_planning_server.py
  scripts/door_execution_server.py
  scripts/test_door_opening.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
install(DIRECTORY launch config action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
