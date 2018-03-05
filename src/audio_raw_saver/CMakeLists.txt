cmake_minimum_required(VERSION 2.8.3)
project(audio_raw_saver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  audio_common_msgs
  detection_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS audio_common_msgs detection_msgs
)