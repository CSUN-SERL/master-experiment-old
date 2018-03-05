#ifndef SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_
#define SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/core/ocl.hpp>

namespace sarwai {

  enum TrackingAlgorithm {BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN};
  
  class VisualDetectionTracker {
  public:
    VisualDetectionTracker(TrackingAlgorithm tracking_algorithm);
    void TrackFrame(const cv::Mat &image_matrix);
    void AddTrackers(const cv::Mat &image_matrix, std::vector<cv::Rect2d>);
    bool HasActiveTrackers();
    // These two method should be private, but I can't get it to compile when they are...
    
    ~VisualDetectionTracker();
    
  private:
    
    TrackingAlgorithm tracking_algorithm_;
    // The primary trackers that log data is pulled from
    std::vector<cv::Ptr<cv::Tracker> > trackers_;
    std::vector<cv::Rect2d> tracking_boxes_;

    std::mutex tracking_mtx_;
  };
}

#endif
