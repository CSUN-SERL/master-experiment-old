#include"detection_frame_id.h"
#include "ros/ros.h"
namespace sarwai {

  int DetectionFrameId::detection_id_count_ = 0;

  DetectionFrameId::DetectionFrameId() {
    detection_id_ = DetectionFrameId::detection_id_count_;
    DetectionFrameId::detection_id_count_ += 1;
    nth_frame_ = 0;
  }

  DetectionFrameId::DetectionFrameId(int existing_id,
      int existing_frame_id) {

    detection_id_ = existing_id;
    nth_frame_ = existing_frame_id;
  }

  // DetectionFrameId& DetectionFrameId::operator=(const DetectionFrameId& frame_id) {
  //   this->detection_id_ = frame_id.detection_id_;
  //   this->nth_frame_ = frame_id.nth_frame_;
  //   return *this;
  // }

  int DetectionFrameId::DetectionId() {
    return detection_id_;
  }

  void DetectionFrameId::SetBoxColor(cv::Scalar color) {
    box_color_ = color;
  }

  cv::Scalar DetectionFrameId::BoxColor() {
    return box_color_;
  }

  int DetectionFrameId::FrameId() {
    return nth_frame_;
  }

  void DetectionFrameId::IncFrame() {
    nth_frame_ += 1;
  }
}