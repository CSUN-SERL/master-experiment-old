cmake_minimum_required(VERSION 2.8.3)
project(image_draw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  detection_msgs
  #OpenCV REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_draw
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs darknet_ros_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  #OpenCV_INCLUDE_DIR
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/image_listener.cpp
  src/image_bounding_box_merger.cc
#  src/visual_detection_tracker.cc
)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  "/usr/local/lib/libopencv_tracking.so"
  ${catkin_LIBRARIES}
)

