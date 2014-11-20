cmake_minimum_required(VERSION 2.8.3)
project(odometry_estimate)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
# CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES})
