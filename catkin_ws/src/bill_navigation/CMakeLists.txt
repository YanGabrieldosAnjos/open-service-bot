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

SET( ${PROJECT_NAME}_HDRS
    ./include/bill_navigation/Robot.h
)

SET( ${PROJECT_NAME}_SRCS
    ./src/Robot.cpp
)

include_directories(include/${PROJECT_NAME})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Bill_Navigation
#  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)



add_executable(Robot_node src/Robot.cpp
)

add_executable(Odometry src/odometry.cpp
)

target_link_libraries(Robot_node
  ${catkin_LIBRARIES}
)

target_link_libraries(Odometry
  ${catkin_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

