#include <iostream>

#include "image_bounding_box_merger.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {

  ImageBoundingBoxMerger::ImageBoundingBoxMerger() {
    //Subscribes to darknet_ros/detection_image
    this->nh_ = new ros::NodeHandle();

    this->image_frame_sub_ = this->nh_->subscribe(
      "/compiled_ros_message", 1000, &ImageBoundingBoxMerger::RunImageProcess, this);


    //Publishes to visual_detection topic
      this->visual_detection_pub_ = this->nh_->advertise<detection_msgs::ProcessedVisualDetection>(
        "/sarwai_detection/detection_processeddetection", 1000);
  }
  

  ImageBoundingBoxMerger::~ImageBoundingBoxMerger() {
    //empty
  }
  
  void ImageBoundingBoxMerger::PublishMergedData(
    sensor_msgs::Image image, darknet_ros_msgs::BoundingBox box, unsigned int robotId) {
      detection_msgs::ProcessedVisualDetection outgoing_detection_msg;
      //Set image info to custom message detection_msgs::ProcessedVisualDetection
      outgoing_detection_msg.image = image; 
      //Set bounding box info to custom message detection_msgs::ProcessedVisualDetection
      //utgoing_detection_msg.bounding_box = box;
      outgoing_detection_msg.robotId = robotId;
      this->visual_detection_pub_.publish(outgoing_detection_msg);  
  }

  void ImageBoundingBoxMerger::RunImageProcess(const detection_msgs::CompiledMessageConstPtr& msg) {
    std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = msg->boxes.boundingBoxes;
    sensor_msgs::Image master_image = msg->image;
    unsigned robotId = msg->robotId;
    for (int i = 0; i < bounding_boxes.size(); i++) {
      DrawRectAndPublishImage(bounding_boxes[i], master_image, robotId);    
    }
  }

  // Function draws box around the detected image
  void ImageBoundingBoxMerger::DrawRectAndPublishImage( 
    const darknet_ros_msgs::BoundingBox &box, const sensor_msgs::Image &image, unsigned robotId) {
      // Create a value copy of the image, to be modified later
      sensor_msgs::Image image_copy = image; // @TODO: is this an unnecessary copy?
      // Create an OpenCV image matrix from the ROS Image msg
      cv_bridge::CvImagePtr cv_image;
      cv_image = cv_bridge::toCvCopy(image_copy, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_matrix = cv_image->image;
      cv::Point top_left_corner = cv::Point(box.xmin, box.ymin);  
      cv::Point bottom_right_corner = cv::Point(box.xmax, box.ymax);
      // Draw the bounding box on the image
      cv::rectangle(image_matrix, top_left_corner, bottom_right_corner, 2);
      sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matrix).toImageMsg();
      image_copy = *image_msg;
      // Reassigns header value as the transition to OpenCV and back drops the header data.
      // In particular, we are interested in the timestamp of the image.
      image_copy.header = image.header;
      PublishMergedData(image_copy, box, robotId);
  }
}
