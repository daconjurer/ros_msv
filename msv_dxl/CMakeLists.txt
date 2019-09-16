cmake_minimum_required(VERSION 2.8.3)
project(msv_dxl)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  include/msv_dxl
)

add_executable(msv_dxl src/msv_dxl.cpp
									src/dxl_handler.cpp)
target_link_libraries(msv_dxl ${catkin_LIBRARIES})
