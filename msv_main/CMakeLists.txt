cmake_minimum_required(VERSION 2.8.3)
project(msv_main)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

## Find System dependencies
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
	include
	${Boost_INCLUDE_DIRS}
	/usr/include/lightmodbus
	${catkin_INCLUDE_DIRS}
)

add_library(msv_porthandler src/port_handler.cpp)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES msv_porthandler
	CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(msv_robot src/msv_main.cpp
									src/main_publisher.cpp)
									
target_link_libraries(msv_robot ${catkin_LIBRARIES} lightmodbus)

