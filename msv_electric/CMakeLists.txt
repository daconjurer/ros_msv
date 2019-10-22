cmake_minimum_required(VERSION 2.8.3)
project(msv_electric)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(msv_main REQUIRED)
find_package(msv_msgs REQUIRED)
## Find System dependencies
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  /usr/include/lightmodbus
  ${catkin_INCLUDE_DIRS}
  ${msv_main_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

add_executable(msv_electric src/msv_electric.cpp
									src/msv_electric_node.cpp)

target_link_libraries(msv_electric ${catkin_LIBRARIES} msv_porthandler lightmodbus)

add_dependencies(msv_electric ${catkin_EXPORTED_TARGETS})
