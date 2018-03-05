cmake_minimum_required(VERSION 2.8.3)
project(pseudo_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  detection_msgs
)

catkin_python_setup()

#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)


catkin_package(
  CATKIN_DEPENDS nav_msgs
  CATKIN_DEPENDS sensor_msgs
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS detection_msgs
)
