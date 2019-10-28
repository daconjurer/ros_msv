cmake_minimum_required(VERSION 2.8.3)
project(msv_imu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(msv_main REQUIRED)

## Find System dependencies
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${msv_main_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES msv_porthandler
	CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(msv_imu src/msv_imu_node.cpp)

target_link_libraries(msv_imu ${catkin_LIBRARIES} msv_porthandler)
