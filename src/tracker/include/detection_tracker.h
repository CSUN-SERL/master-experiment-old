#ifndef SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_
#define SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_

#include <string>
#include <vector>
#include <memory>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "sensor_msgs/Image.h"
#include "detection_msgs/DetectionIdImage.h"
#include "detection_msgs/DetectionMatch.h"
// #include "detection_msgs/ProcessedVisualDetection.h"

// #include "face_classifier_manager.h"
#include "detection_frame_id.h"
// #include "detection_similarity_association.h"
#include "detection_aggregator.h"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/core/ocl.hpp>

#include <detection_msgs/CompiledMessage.h>


namespace sarwai {

  enum TrackingAlgorithm {BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN};
  
  class VisualDetectionTracker {

  public:
    //void TrackFrame(const cv::Mat &image_matrix, std::vector<cv::Rect2d>, std::vector<darknet_ros_msgs::BoundingBox>);
    darknet_ros_msgs::BoundingBoxes TrackFrame(const cv::Mat &image_matrix, std::vector<cv::Rect2d>, std::vector<darknet_ros_msgs::BoundingBox>);
    //void AddTrackers(const cv::Mat &image_matrix, std::vector<cv::Rect2d>, std::vector<darknet_ros_msgs::BoundingBox>);
    darknet_ros_msgs::BoundingBoxes AddTrackers(const cv::Mat &image_matrix, std::vector<cv::Rect2d>, std::vector<darknet_ros_msgs::BoundingBox>);
    VisualDetectionTracker();
    ~VisualDetectionTracker();
    
  private:
    ros::NodeHandle* nh_;
    ros::NodeHandle* _nh; // to get the private params

    TrackingAlgorithm tracking_algorithm_;
    //darknet_ros_msgs::BoundingBoxes out_going_bb;

    std::vector<DetectionAggregation> active_detections_;
    // std::vector<DetectionAggregation> past_detections_;

    std::vector<cv::Ptr<cv::Tracker> > trackers_;
    std::vector<cv::Rect2d> tracking_boxes_;
    std::vector<DetectionFrameId*> detection_ids_;

    ros::Subscriber compiled_msg_;

    ros::Subscriber detection_match_sub_;

    ros::Publisher compiled_messages_;

    ros::Publisher detection_id_image_pub_;
 
    // std::queue<sensor_msgs::Image> video_image_frames_; 
    // std::queue<std::vector<darknet_ros_msgs::BoundingBox>> bounding_boxes_matrix_;  

    void ImageCallback(const detection_msgs::CompiledMessageConstPtr& msg);
    // void DetectionMatchCallback(const detection_msgs::DetectionMatch &msg);
    void PropagateToDetectionComparer(cv::Mat, cv::Rect, DetectionFrameId*, bool);
    void Process(int RoboId, sensor_msgs::Image video_image_frame, std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes);
    bool CheckIfRectMatchesRectVector(cv::Rect2d, std::vector<cv::Rect2d>);
    float ComputeFractionOfIntersection(cv::Rect2d, cv::Rect2d);
    float ComputeDistance(cv::Rect2d, cv::Rect2d);
    float ComputeRectArea(cv::Rect2d);
    float getAverageArea(cv::Rect2d, cv::Rect2d);
    void MarkDetectionComplete(int index);
  };
}

#endif
