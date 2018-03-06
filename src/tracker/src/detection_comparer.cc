#include "detection_comparer.h"

#include <cv_bridge/cv_bridge.h>

namespace sarwai {
  DetectionComparer::DetectionComparer() {
    nh_ = new ros::NodeHandle();
    detection_image_id_sub_ = nh_->subscribe("labeled_detection_images",
        10, &DetectionComparer::ProcessDetectionCallback, this);

    detection_match_pub_ = nh_->advertise<detection_msgs::DetectionMatch>(
        "detection_match", 100);
  }

  void DetectionComparer::ProcessDetectionCallback(
      const detection_msgs::DetectionIdImage::ConstPtr &msg) {

    // Construct local datatypes from the received message
    DetectionFrameId* detection_id = new DetectionFrameId(
        msg->detectionId.id, msg->detectionId.frameId
    );
    
    cv::Mat image = RosImageToCvImage(msg->image);
    cv::Rect roi = RosRectToCvRect(msg->regionOfInterest);

    if ((bool)msg->isDetectionConcluded) {
      // Indicates that the data populating a model has ended.
      // If the model is not being dynamically trained, train it now.
      face_comparer_.DeactivateModel(detection_id->DetectionId());
    } else {
      // Add the image frame to the model's data store.
      face_comparer_.RunFacialDetection(image, detection_id, roi);

      DetectionSimilarityAssociation face_assoc = face_comparer_.FindDoppelganger(
          image, roi, detection_id);
      
      if (face_assoc.confidence != 0.0 && face_assoc.confidence < 115) {
        ROS_INFO("%d == %d confidence: %f", face_assoc.compared_against->DetectionId(),
            face_assoc.to_be_associated->DetectionId(), face_assoc.confidence);

        
        detection_msgs::DetectionId past_detection;
        past_detection.id = face_assoc.compared_against->DetectionId();
        past_detection.frameId = face_assoc.compared_against->FrameId();
        detection_msgs::DetectionMatch detection_match;
        detection_match.activeDetection = msg->detectionId;
        detection_match.pastDetection = past_detection;
        detection_match.confidence = face_assoc.confidence;
        
        detection_match_pub_.publish(detection_match);
      }
    }
  }

  cv::Mat DetectionComparer::RosImageToCvImage(sensor_msgs::Image ros_image) {
    return cv_bridge::toCvCopy(ros_image,
        sensor_msgs::image_encodings::BGR8)->image;
  }

  cv::Rect DetectionComparer::RosRectToCvRect(detection_msgs::Rect rect) {
    cv::Rect r(rect.xpos, rect.ypos, rect.width, rect.height);
    return r;
  }

  DetectionComparer::~DetectionComparer() {
    // delete nh_;
    for (auto e : detection_ids_) {
      delete e;
    }
  }
}