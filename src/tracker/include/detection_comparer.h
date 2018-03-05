#include <vector>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "sensor_msgs/Image.h"
#include "face_classifier_manager.h"
#include "detection_msgs/DetectionIdImage.h"
#include "detection_msgs/Rect.h"
#include "detection_msgs/DetectionMatch.h"

namespace sarwai {

  class DetectionComparer {
    public:
    DetectionComparer();
    ~DetectionComparer();

    private:
    ros::NodeHandle* nh_;
    ros::Subscriber detection_image_id_sub_;
    ros::Publisher detection_match_pub_;

    cv::Mat RosImageToCvImage(sensor_msgs::Image);
    cv::Rect RosRectToCvRect(detection_msgs::Rect);
    void ProcessDetectionCallback(const detection_msgs::DetectionIdImage::ConstPtr &msg);

    
    FaceClassifierManager face_comparer_;

    std::vector<DetectionFrameId*> detection_ids_;
  };
}