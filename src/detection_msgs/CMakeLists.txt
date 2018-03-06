cmake_minimum_required(VERSION 2.8.3)
project(detection_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  darknet_ros_msgs
  message_generation
  genmsg
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ProcessedVisualDetection.msg
   BoundingBox.msg
   DetectionId.msg
   Rect.msg
   DetectionIdImage.msg
   DetectionMatch.msg
   CompiledMessage.msg
   CompiledFakeMessage.msg
   AudioDetection.msg
   AudioFileReady.msg
   Human.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
   darknet_ros_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detection_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
