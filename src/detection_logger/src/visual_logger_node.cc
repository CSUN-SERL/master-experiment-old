#include <string>

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "detection_msgs/ProcessedVisualDetection.h"
#include "detection_msgs/HumanSeen.h"

#include "visual_detection_data.h"
#include "visual_logger.h"
#include "socketio_visual_logger.h"


using namespace sarwai;

void LogVisualDetection(const detection_msgs::ProcessedVisualDetection::ConstPtr&);
void SendHumanViewed(const detection_msgs::HumanSeen::ConstPtr& msg);

VisualLogger visual_logger(".");
SocketIOVisualLogger socketio_logger(".", "http://192.168.1.11", 8000);
// local
// SocketIOVisualLogger socketio_logger(".", "http://127.0.0.1", 8000);

int main(int argc, char **argv) {

  // visual_logger = new VisualLogger("./");
  std::cout << "()()()()()()()()()()()()()()()()()()()VISUAL LOGGER" << std::endl;

  ros::init(argc, argv, "visual_logger");
  ros::NodeHandle nh;
  std::string visual_detection_topic = "/sarwai_detection/detection_processeddetection";
  std::string human_view_topic = "/human_camera_seen";
  ros::Subscriber visual_sub = nh.subscribe(visual_detection_topic.c_str(), 1000, LogVisualDetection);
  ros::Subscriber human_in_view_sub = nh.subscribe(human_view_topic.c_str(), 1000, SendHumanViewed);

  ros::spin();
}

void LogVisualDetection(const detection_msgs::ProcessedVisualDetection::ConstPtr& msg) {
  // Convert ROS message to logging struct
  std::cout << "***********************LOG VISUAL DETECTION" << std::endl;
  struct VisualDetectionData data;
  data.box_height = msg->bounding_box.ymax - msg->bounding_box.ymin;
  data.box_width = msg->bounding_box.xmax - msg->bounding_box.xmin;
  data.left_x_coord = msg->bounding_box.xmin;
  data.top_y_coord = msg->bounding_box.ymin;
  data.timestamp = (int) msg->image.header.stamp.sec;
  data.object_class = "Human";
  data.robot_id = msg->robotId;
  data.human_id = msg->human_id;

  // Convert ROS image to OpenCV image
  cv::Mat cvimage = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8)->image;

  std::string image_filename = visual_logger.Log(cvimage, data);
  if(msg->forced) {
    socketio_logger.SendHuman(msg->human_id, msg->robotId, true);
  }
  else {
    socketio_logger.Log(image_filename, data);
  }
}

void SendHumanViewed(const detection_msgs::HumanSeen::ConstPtr& msg) {
  socketio_logger.SendHuman(msg->human_id, msg->robot_id, false);
}