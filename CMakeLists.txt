cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(remote_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
)

find_package( mavros_msgs REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(remote_controller 
    src/main.cpp
    src/Remotecontroller.cpp
    src/listop.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(remote_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
