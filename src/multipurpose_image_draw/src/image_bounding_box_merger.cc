#include <iostream>
#include "image_bounding_box_merger.h"
#include <cv_bridge/cv_bridge.h>

namespace sarwai {

  ImageBoundingBoxMerger::ImageBoundingBoxMerger(std::string subscriptionTopic) {
    m_nh = new ros::NodeHandle();

    m_trackingSub = m_nh->subscribe(subscriptionTopic, 1000, &ImageBoundingBoxMerger::drawBoxesCallback, this);
    m_visualDetectionPub = m_nh->advertise<detection_msgs::ProcessedVisualDetection>("/sarwai_detection/detection_processeddetection", 1000);
    m_boxStreamPubOne = m_nh->advertise<sensor_msgs::Image>("/robot1/camera/rgb/image_boxed", 1000);
    m_boxStreamPubTwo = m_nh->advertise<sensor_msgs::Image>("/robot2/camera/rgb/image_boxed", 1000);
    m_boxStreamPubThree = m_nh->advertise<sensor_msgs::Image>("robot3/camera/rgb/image_boxed", 1000);
    m_boxStreamPubFour = m_nh->advertise<sensor_msgs::Image>("robot4/camera/rgb/image_boxed", 1000);

  }
  

  ImageBoundingBoxMerger::~ImageBoundingBoxMerger() {
    //empty
  }
  
  void ImageBoundingBoxMerger::drawBoxesCallback(const detection_msgs::CompiledFakeMessageConstPtr& msg) {
    // Draw boxes for each new query
    for(unsigned i = 0; i < msg->humanQueries.size(); ++i) {
      for(unsigned h = 0; h < msg->humans.size(); ++h) {
        if(msg->humans[h].id == msg->humanQueries[i]) {
          drawBoxAndSendQuery(msg, msg->humans[h]);
          break;
        }
      }
    }
    // Draw boxes around each person in one frame
	sendBoxedStream(msg);
  }

  void ImageBoundingBoxMerger::drawBoxAndSendQuery(const detection_msgs::CompiledFakeMessageConstPtr& msg, detection_msgs::Human human) const {
    sensor_msgs::Image imageCopy(msg->img);

    detection_msgs::BoundingBox box = drawBoxAroundHuman(imageCopy, human, msg->fov);

    detection_msgs::ProcessedVisualDetection queryMsg;
    queryMsg.image = imageCopy;
    queryMsg.robotId = msg->robot;

    this->m_visualDetectionPub.publish(queryMsg);
  }

  detection_msgs::BoundingBox ImageBoundingBoxMerger::drawBoxAroundHuman(sensor_msgs::Image& image, detection_msgs::Human human, float fov) const {
    detection_msgs::BoundingBox ret;
	
	unsigned yCoord = image.height - (BOXLENGTH / 2);
    unsigned xCoord = ((-1 * (human.angleToRobot / (fov / image.width))) + (image.width / 2)) - (BOXLENGTH / 2);
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv::Mat imageMatrix = cvImage->image;
    cv::Point topLeftCorner = cv::Point(xCoord, yCoord);
    cv::Point bottomRightCorner = cv::Point(xCoord + BOXLENGTH, yCoord + BOXLENGTH);
    cv::rectangle(imageMatrix, topLeftCorner, bottomRightCorner, 3);
    image = *(cv_bridge::CvImage(image.header, "bgr8", imageMatrix).toImageMsg());

    ret.xmin = xCoord;
    ret.ymin = yCoord;
    ret.xmax = bottomRightCorner.x;
    ret.ymax = bottomRightCorner.y;
    return ret;
  }

  void ImageBoundingBoxMerger::sendBoxedStream(const detection_msgs::CompiledFakeMessageConstPtr& msg) const {
    sensor_msgs::Image imageCopy(msg->img);
    for(unsigned i = 0; i < msg->humans.size(); ++i) {
      drawBoxAroundHuman(imageCopy, msg->humans[i], msg->fov);
    }
    
    unsigned id = msg->robot;
    if(id == 1) {
      m_boxStreamPubOne.publish(imageCopy);
    }
    else if(id == 2) {
      m_boxStreamPubTwo.publish(imageCopy);
    }
    else if(id == 3) {
      m_boxStreamPubThree.publish(imageCopy);
    }
    else {
      m_boxStreamPubFour.publish(imageCopy);
    }
  }

}
