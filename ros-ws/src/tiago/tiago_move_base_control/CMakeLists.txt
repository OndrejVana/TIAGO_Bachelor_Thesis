cmake_minimum_required(VERSION 3.0.2)
project(tiago_move_base_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  tf2
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES door_mask_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib tf2 geometry_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Costmap layer plugin (C++)
add_library(door_mask_layer src/DoorMaskLayer.cpp)
target_link_libraries(door_mask_layer ${catkin_LIBRARIES})

install(TARGETS door_mask_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Existing Python nodes
catkin_install_python(PROGRAMS
  scripts/move_base_goal_sender.py
  scripts/door_approach_goal_generator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)