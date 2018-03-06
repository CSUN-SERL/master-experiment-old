#include "ros/ros.h"
#include "detection_tracker.h"



/********************************* MAIN ***************************************************************/

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "image_track");
  sarwai::VisualDetectionTracker track;
  ros::spin();

}
