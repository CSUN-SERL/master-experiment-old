#ifndef SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_
#define SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_


#include <string>

#include "ros/ros.h"
#include "detection_msgs/CompiledFakeMessage.h"
#include "detection_msgs/ProcessedVisualDetection.h"
#include "detection_msgs/BoundingBox.h"
#include "detection_msgs/Human.h"


namespace sarwai {

  class ImageBoundingBoxMerger {
  public:

    ImageBoundingBoxMerger(std::string subscriptionTopic);
    ~ImageBoundingBoxMerger();

  private:
    const unsigned BOXLENGTH = 70;

    ros::NodeHandle* m_nh;

    ros::Subscriber m_trackingSub;
    ros::Publisher m_visualDetectionPub;
    ros::Publisher m_boxStreamPubOne;
    ros::Publisher m_boxStreamPubTwo;
    ros::Publisher m_boxStreamPubThree;
    ros::Publisher m_boxStreamPubFour;

    void drawBoxesCallback(const detection_msgs::CompiledFakeMessageConstPtr& msg);
    void drawBoxAndSendQuery(const detection_msgs::CompiledFakeMessageConstPtr& msg, detection_msgs::Human human) const;
    detection_msgs::BoundingBox drawBoxAroundHuman(sensor_msgs::Image& image, detection_msgs::Human human, float fov) const;
    void sendBoxedStream(const detection_msgs::CompiledFakeMessageConstPtr& msg) const;

    sensor_msgs::Image drawBox(sensor_msgs::Image image, detection_msgs::Human human);

  };
}

#endif
