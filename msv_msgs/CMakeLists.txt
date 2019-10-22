cmake_minimum_required(VERSION 2.8.3)
project(msv_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

# Electric sensors & Actuators messages declaration and generation
add_message_files(FILES Electric.msg Actuators.msg)
												# other message files to be included here
												#Message.msg)
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)
