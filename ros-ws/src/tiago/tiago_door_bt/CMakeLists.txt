cmake_minimum_required(VERSION 3.0.2)
project(tiago_door_bt)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/door_bt_executor.py
  scripts/bt_nodes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)