cmake_minimum_required(VERSION 3.0.2)
project(tiago_door_localization)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_runtime
)

## Uncomment this if the package has a setup.py
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tiago_door_localization
  CATKIN_DEPENDS rospy tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp tests
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tiago_door_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
