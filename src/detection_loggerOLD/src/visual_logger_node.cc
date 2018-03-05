#include <string>

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "detection_msgs/ProcessedVisualDetection.h"
#include "visual_detection_data.h"
#include "visual_logger.h"
#include "socketio_visual_logger.h"


using namespace sarwai;

void LogVisualDetection(const detection_msgs::ProcessedVisualDetection::ConstPtr&);

VisualLogger visual_logger(".");
SocketIOVisualLogger socketio_logger(".", "http://192.168.1.11", 8000);
// local
// SocketIOVisualLogger socketio_logger(".", "http://127.0.0.1", 8000);

int main(int argc, char **argv) {

  // visual_logger = new VisualLogger("./");

  ros::init(argc, argv, "visual_logger");
  ros::NodeHandle nh;
  std::string visual_detection_topic = "/sarwai_detection/detection_processeddetection";
  ros::Subscriber visual_sub = nh.subscribe(visual_detection_topic.c_str(), 1000, LogVisualDetection);

  ros::spin();
}

void LogVisualDetection(const detection_msgs::ProcessedVisualDetection::ConstPtr& msg) {
  // Convert ROS message to logging struct
  struct VisualDetectionData data;
  data.box_height = msg->bounding_box.ymax - msg->bounding_box.ymin;
  data.box_width = msg->bounding_box.xmax - msg->bounding_box.xmin;
  data.left_x_coord = msg->bounding_box.xmin;
  data.top_y_coord = msg->bounding_box.ymin;
  data.timestamp = (int) msg->image.header.stamp.sec;
  data.confidence_rating = msg->bounding_box.probability;
  data.object_class = "Human";
  data.robot_id = msg->robotId;

  // Convert ROS image to OpenCV image
  cv::Mat cvimage = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8)->image;

  std::string image_filename = visual_logger.Log(cvimage, data);
  socketio_logger.Log(image_filename, data);
}
