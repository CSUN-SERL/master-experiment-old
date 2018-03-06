cmake_minimum_required(VERSION 2.8.3)
project(audio_trigger_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Trigger.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs
  CATKIN_DEPENDS message_runtime
)
