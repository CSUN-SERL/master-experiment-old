#include "detection_tracker.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <iostream>

namespace sarwai {

int gRobotId = 0;

  VisualDetectionTracker::VisualDetectionTracker() {
    this->nh_ = new ros::NodeHandle();
    this->_nh =  new ros::NodeHandle("~");

    std::string topic_name_;
    _nh -> getParam("topic_name_", topic_name_);
    
    //"/detection/compiled_ros_msg"

    //topic_name_ can be used in terminal to set parameters of choice
    this->compiled_msg_ = this->nh_->subscribe(topic_name_ , 30, &VisualDetectionTracker::ImageCallback, this);

    this->detection_id_image_pub_ = nh_->advertise<detection_msgs::DetectionIdImage>("labeled_detection_images", 100);

    this->compiled_messages_ = this->nh_->advertise<detection_msgs::CompiledMessage>("compiled_ros_message", 1000);        

    //this->tracking_algorithm_ = TrackingAlgorithm::BOOSTING;
    //this->tracking_algorithm_ = TrackingAlgorithm::MIL;
    this->tracking_algorithm_ = TrackingAlgorithm::MEDIANFLOW;
  }


  VisualDetectionTracker::~VisualDetectionTracker() {
    for (auto e : detection_ids_) {
      delete e;
    }
  }

  void VisualDetectionTracker::ImageCallback(const detection_msgs::CompiledMessageConstPtr& msg){
    std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = msg->boxes.boundingBoxes;
    sensor_msgs::Image master_image = msg->image;
    unsigned robotId = msg->robotId;

    //this->bounding_boxes_matrix_.push(bounding_boxes);
    //this->video_image_frames_.push(master_image);
    Process(robotId, master_image, bounding_boxes);
  }

  // void VisualDetectionTracker::DetectionMatchCallback(const detection_msgs::DetectionMatch   &msg) {
  //   DetectionFrameId active_detection(msg.activeDetection.id, msg.activeDetection.frameId);
  //   DetectionFrameId past_detection(msg.pastDetection.id, msg.pastDetection.frameId);
  //   ROS_INFO("%d == %d at confidence: %f", active_detection.DetectionId(),
  //       past_detection.DetectionId(), msg.confidence);
  //   for (int i = 0; i < active_detections_.size(); i++) {
  //     if (active_detections_[i].id->DetectionId() == active_detection.DetectionId()) {
  //       // Get box color of previous detection
  //       for (int j = 0; j < past_detections_.size(); j++) {
  //         if (past_detections_[j].id->DetectionId() == past_detection.DetectionId()) {
  //           past_detection.SetBoxColor(past_detections_[j].id->BoxColor());
  //           break;
  //         }
  //       }
  //       active_detections_[i].id->SetBoxColor(past_detection.BoxColor());
  //     }
  //   }
  // }

  /*
   * Process controls the process of receiving messages on incoming ROS topics
   * and the publishing of data after running the tracking redundancy detection system
   */
  void VisualDetectionTracker::Process(int roboId,
    sensor_msgs::Image video_image_frame,
    std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes) {

    gRobotId = roboId;
    // // This check only lets this function run if there are also elements in the detection flag and bounding box queues
    // if (this->bounding_boxes_matrix_.size() == 0) {
    //   // If you aren't ready to process all 3 queues, we have to prune them to make sure they don't get backlogged
    //   if (this->video_image_frames_.size() > 0) {
    //     this->video_image_frames_.pop();
    //   }
    // }

    // while (this->video_image_frames_.size() > 0 &&
    //     this->bounding_boxes_matrix_.size() > 0 ) { //&& this->detection_flag_.size() > 0

    std::vector<cv::Rect2d> detection_bbs;
    // std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = bounding_boxes_matrix_.front();
    for (int i = 0; i < bounding_boxes.size(); i++) {
      darknet_ros_msgs::BoundingBox bb = bounding_boxes.at(i);
      cv::Rect2d bb_rect(bb.xmin, bb.ymin, bb.xmax - bb.xmin, bb.ymax - bb.ymin);
      detection_bbs.push_back(bb_rect);
    }

    // TrackFrame(
    //   cv_bridge::toCvCopy(
    //     video_image_frame,
    //     sensor_msgs::image_encodings::BGR8)->image,
    //   detection_bbs, bounding_boxes);

    
    darknet_ros_msgs::BoundingBoxes out_going_bb = TrackFrame(
      cv_bridge::toCvCopy(video_image_frame,sensor_msgs::image_encodings::BGR8)->image, detection_bbs, bounding_boxes
      );

    detection_msgs::CompiledMessage outmsg;
    // Send data along in the ROS node chain
    //std_msgs::Int8 msg;
    //darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;
    outmsg.robotId = roboId;
    outmsg.boxes = out_going_bb;
    outmsg.image = video_image_frame;
    compiled_messages_.publish(outmsg);

    // this->out_going_bb.boundingBoxes.clear();
    // this->video_image_frames_.pop();
    // this->bounding_boxes_matrix_.pop();
    // }
  }

  /*
   * TrackFrame receives a image and a vector of boxes demarking visual detection.
   * This is the entry point to the tracking system whose purpose is reducing redundant detections.
   * 
   * Upon receiving a new detection, a tracking box is placed on top of the detection box.
   * With every new frame and new set of detection boxes, the tracking boxes are updated
   * and compared with the set of detection boxes. If the tracking box is determined to come close to
   * being the same as a detection box, we conclude the incoming detection box is redundant, 
   * and we do not need to send it to logging.
   * 
   * TODO: Modify TrackFrame to return a vector of bounding boxes to send to logging
   */
  darknet_ros_msgs::BoundingBoxes VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix,
        std::vector<cv::Rect2d> detect_bbs,
        std::vector<darknet_ros_msgs::BoundingBox> original_bb) {

    cv::Mat image_copy = image_matrix.clone();
    cv::Rect2d bb;
    for (int i = 0; i < this->active_detections_.size(); i++) {

      if (!CheckIfRectMatchesRectVector(active_detections_[i].bb, detect_bbs)) {
        // There is no longer a detection box found with this tracking box
        PropagateToDetectionComparer(image_copy, active_detections_[i].bb,
            active_detections_[i].id, true);
        MarkDetectionComplete(i);

        continue;
      }

      bool object_tracked = active_detections_[i].tracker->update(image_copy, bb);

      if (object_tracked) {
        ROS_INFO("buffer: %d\n", active_detections_[i].frame_buffer_count);
        // With a combination of low framerate and fast camera pans, the detection box
        // and tracking box may become completely disjointed. We expand the tracker box in attempt
        // to recapture the object
        // if (!object_tracked) {
        //   active_detections_[i].frame_buffer_count += 1;
        // }

        // ROS_INFO("tracker succeeded or buffer count check");

        active_detections_[i].id->IncFrame();
        active_detections_[i].bb = bb;

        PropagateToDetectionComparer(image_copy, active_detections_[i].bb,
            active_detections_[i].id, false);

        cv::rectangle(image_copy, active_detections_[i].bb, active_detections_[i].id->BoxColor(), 10, 143);
        cv::waitKey(1);
      }
      else {
        // The tracking algorithm has failed to continue tracking the given object
        PropagateToDetectionComparer(image_copy, active_detections_[i].bb,
            active_detections_[i].id, true);
        ROS_INFO("Tracker failed");
        MarkDetectionComplete(i);
      }
    }

    std::string imshow_name = "tracking " + std::to_string(gRobotId);
    //AddTrackers(image_matrix, detect_bbs, original_bb);
    darknet_ros_msgs::BoundingBoxes out_going_bb = AddTrackers(image_matrix, detect_bbs, original_bb);
    std::cout << "Showing image" << std::endl;
    cv::imshow(imshow_name, image_copy);
    std::cout << "Done showing" << std::endl;
    return out_going_bb;
  }


  darknet_ros_msgs::BoundingBoxes VisualDetectionTracker::AddTrackers(const cv::Mat &image,
      std::vector<cv::Rect2d> detection_bbs,
      std::vector<darknet_ros_msgs::BoundingBox> original_bb) {
    
    darknet_ros_msgs::BoundingBoxes out_going_bb;
    std::vector<cv::Rect2d> active_bbs;
    for (int i = 0; i < active_detections_.size(); i++) {
      active_bbs.push_back(active_detections_[i].bb);
    }

    for (int i = 0; i < detection_bbs.size(); i++) {
      if (!CheckIfRectMatchesRectVector(detection_bbs[i], active_bbs)) {
        // At this point, we can assume that we see a new, unique detection
        ROS_INFO("adding new tracker");
        cv::Ptr<cv::Tracker> new_tracker;
        switch (this->tracking_algorithm_) {
          case TrackingAlgorithm::BOOSTING:
            new_tracker = cv::TrackerBoosting::create();
            break;
          case TrackingAlgorithm::MIL:
            new_tracker = cv::TrackerMIL::create();
            break;
          case TrackingAlgorithm::KCF:
            new_tracker = cv::TrackerKCF::create();
            break;
          case TrackingAlgorithm::TLD:
            new_tracker = cv::TrackerTLD::create();
            break;
          case TrackingAlgorithm::MEDIANFLOW:
            new_tracker = cv::TrackerMedianFlow::create();
            break;
          case TrackingAlgorithm::GOTURN:
            new_tracker = cv::TrackerGOTURN::create();
            break;
          default:
            darknet_ros_msgs::BoundingBoxes default_boxes;
            return default_boxes;
        }


        new_tracker->init(image, detection_bbs.at(i));
        // this->trackers_.push_back(new_tracker);
        // this->tracking_boxes_.push_back(detection_bbs.at(i));

        DetectionFrameId* new_detection_id = new DetectionFrameId();
        int blue_color = rand() % 255;
        int red_color = rand() % 255;
        int green_color = rand() % 255;
        new_detection_id->SetBoxColor(cv::Scalar(blue_color, red_color, green_color));

        DetectionAggregation detection;
        detection.tracker = new_tracker;
        detection.bb = detection_bbs[i];
        detection.id = new_detection_id;

        active_detections_.push_back(detection);

        PropagateToDetectionComparer(image, detection.bb, detection.id, false);
        
        //darknet_ros_msgs::BoundingBox temp = original_bb.at(i);
        out_going_bb.boundingBoxes.push_back(original_bb.at(i)); //Testing
      }
    }

    return out_going_bb;
  }

  void VisualDetectionTracker::PropagateToDetectionComparer(cv::Mat image,
      cv::Rect bbox, DetectionFrameId* detection_id, bool detection_concluded) {
    
    // // ROS_INFO("propagating image");
    // detection_msgs::DetectionId ros_detection_id;
    // ros_detection_id.id = detection_id->DetectionId();
    // ros_detection_id.frameId = detection_id->FrameId();

    // sensor_msgs::Image image_msg = *(cv_bridge::CvImage(std_msgs::Header(),
    //     "bgr8", image).toImageMsg());

    // detection_msgs::Rect rect;
    // rect.xpos = bbox.x;
    // rect.ypos = bbox.y;
    // rect.width = bbox.width;
    // rect.height = bbox.height;

    // detection_msgs::DetectionIdImage image_id;
    // image_id.isDetectionConcluded = detection_concluded;
    // image_id.regionOfInterest = rect;
    // image_id.detectionId = ros_detection_id;
    // image_id.image = image_msg;

    // detection_id_image_pub_.publish(image_id);
  }

  void VisualDetectionTracker::MarkDetectionComplete(int i) {
    // DetectionFrameId* detection_id = active_detections_[i].id;
    // PropagateToDetectionComparer
    // face_manager_.DeactivateModel(active_detections_[i].id->DetectionId());
    // past_detections_.push_back(active_detections_[i]);
    active_detections_.erase(active_detections_.begin() + i);
  }

  /*
   * CheckIfRectMatchesRecetVector receives a single bounding box and a vector of bounding boxes.
   * It then runs a set of comparison tests beteween the single box and each element of the vector
   * to determine if the single box is similar (to an extent).
   * 
   * It returns true if the bb matches one of the elements in bbs.
   */
  bool VisualDetectionTracker::CheckIfRectMatchesRectVector(cv::Rect2d bb, std::vector<cv::Rect2d> bbs) {
    for (int i = 0; i < bbs.size(); i++) {
      cv::Rect2d vect_bb = bbs.at(i);
      std::cout<<bb.x<<std::endl;
      if (ComputeDistance(bb, vect_bb) < bb.x) {
        return true;
      }
    }

    return false;
  }


  float VisualDetectionTracker::ComputeDistance(cv::Rect2d a, cv::Rect2d b) {
    float center_incoming_x = (a.x + a.width / 2.0);
    float center_incoming_y = (a.y + a.height / 2.0);
    float curent_incoming_x = (b.x + b.width / 2.0);
    float curent_incoming_y = (b.y + b.height / 2.0);

    float distance_x = std::pow(center_incoming_x - curent_incoming_x, 2);
    float distance_y = std::pow(center_incoming_x - curent_incoming_x, 2);
    float distance_between_points = sqrt(distance_x + distance_y);
    
    return distance_between_points;
  }

  float VisualDetectionTracker::getAverageArea(cv::Rect2d a, cv::Rect2d b){
    float getAreaCurent = ((a.x + a.width) - a.x) * ((a.y + a.height) - a.y);
    float getAreaTracker = ((b.x + b.width) - b.x) * ((b.y + b.height) - b.y);

    return getAreaCurent / getAreaTracker;

  }


    /*  
   * Given two areas A_a, A_b and A_i which is the intersection rect of a and b,
   * this function returns A_i / (A_a + A_b - 2A_i).
   * If the 2 factor is removed, the comparison between detection boxes and tracking boxes
   * will nearly always fail.
   * It will return 0 if there is no intersection between rects a and b
   */

  // float VisualDetectionTracker::ComputeFractionOfIntersection(cv::Rect2d a, cv::Rect2d b) {
  //   float top_left_x = std::max(a.x, b.x);
  //   float top_left_y = std::max(a.y, b.y);
  //   float bot_right_x = std::min((a.x + a.width), (b.x + b.width));
  //   float bot_right_y = std::min((a.y + a.height), (b.y + b.height));
    
  //   // Check and see if the two rects a and b are actually intersecting
  //   if (top_left_x >= bot_right_x || top_left_y >= bot_right_y) {
  //     return 0.0;
  //   }

  //   cv::Rect2d intersection(top_left_x, top_left_y, bot_right_x - top_left_x, bot_right_y - top_left_y);
    
  //   float intersection_area = ComputeRectArea(intersection);
    
  //   float fraction_of_intersection = intersection_area / (ComputeRectArea(a) + ComputeRectArea(b) - 2*intersection_area);
  //   return fraction_of_intersection;
  // }




  // float VisualDetectionTracker::ComputeRectArea(cv::Rect2d a) {
  //   float area = a.width * a.height;
  //   return area;
  // }


}
