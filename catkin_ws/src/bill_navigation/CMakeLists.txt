cmake_minimum_required(VERSION 2.8.3)
project(bill_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  rospy
  std_msgs
)

include_directories(include/${PROJECT_NAME})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Bill_Navigation
#  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

