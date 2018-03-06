#include "ros/ros.h"

#include "detection_comparer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "detection_comparer_node");
  sarwai::DetectionComparer detection_comparer;
  ros::spin();
}