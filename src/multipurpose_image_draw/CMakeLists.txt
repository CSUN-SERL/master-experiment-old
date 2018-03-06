cmake_minimum_required(VERSION 2.8.3)
project(multipurpose_image_draw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	image_transport
	roscpp
	sensor_msgs
	std_msgs
	detection_msgs
)

catkin_package(
	CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(
	${PROJECT_NAME}_node
	src/image_listener.cpp
	src/image_bounding_box_merger.cc
)

add_dependencies(
	${PROJECT_NAME}_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
	${PROJECT_NAME}_node
	${catkin_LIBRARIES}
)
