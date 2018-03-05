cmake_minimum_required(VERSION 2.8.3)
project(detection_loggerOLD)

add_compile_options(-std=c++11)

set(
  OpenCV_DIR
  /opt/ros/kinetic/share/OpenCV-3.3.1
)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  detection_msgs
  cv_bridge
  OpenCV REQUIRED
  # cv_bridge
  # image_transport
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  detection_msgs
  cv_bridge
  # image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## VISUAL LOGGER
set(
  VISUAL_LOGGER_SOURCES
  src/socketio_visual_logger.cc
  src/visual_logger.cc
  src/detection_logger.cc
)

add_executable(
  visual_logger_node_real
  src/visual_logger_node.cc
  ${VISUAL_LOGGER_SOURCES}
)

target_link_libraries(
  visual_logger_node_real
  sioclient
  curl
  ${catkin_LIBRARIES}
)

add_dependencies(
  visual_logger_node_real
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

##AUDIO LOGGER
set(
  AUDIO_LOGGER_SOURCES
  src/detection_logger.cc
  src/audio_logger.cc
  src/socketio_audio_logger.cc
)

add_executable(
  audio_logger_node_real
  src/audio_logger_node.cc
  ${AUDIO_LOGGER_SOURCES}
)

target_link_libraries(
  audio_logger_node_real
  sioclient
  curl
  ${catkin_LIBRARIES}
)

add_dependencies(
  audio_logger_node_real
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
