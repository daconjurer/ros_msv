cmake_minimum_required(VERSION 2.8.3)
project(msv_arm)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(msv_main REQUIRED)
## Find System dependencies
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
	include
	/usr/local/include/herkulex_sdk
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(msv_arm src/msv_arm.cpp
									src/msv_arm_node.cpp)
									
target_link_libraries(msv_arm ${catkin_LIBRARIES} hklx_linux_x86)

