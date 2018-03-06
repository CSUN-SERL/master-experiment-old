cmake_minimum_required(VERSION 2.8.3)
project(audio_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  detection_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS detection_msgs
)
