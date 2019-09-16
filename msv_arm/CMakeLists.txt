cmake_minimum_required(VERSION 2.8.3)
project(msv_arm)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/local/include/herkulex_sdk
  ${catkin_INCLUDE_DIRS}
)

add_executable(msv_arm src/msv_arm.cpp)
target_link_libraries(msv_arm ${catkin_LIBRARIES} hklx_linux_x86)

