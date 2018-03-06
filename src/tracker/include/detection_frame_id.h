#ifndef SARWAI_DETECTION_TRACKER_DETECTION_FRAME_ID_
#define SARWAI_DETECTION_TRACKER_DETECTION_FRAME_ID_

#include <opencv2/opencv.hpp>

namespace sarwai {

  class DetectionFrameId {
    public:
    DetectionFrameId();
    DetectionFrameId(int existing_id, int existing_frame_id);
    // DetectionFrameId& operator=(const DetectionFrameId&);
    int DetectionId();
    int FrameId();
    void IncFrame();
    cv::Scalar BoxColor();
    void SetBoxColor(cv::Scalar);
    private:
    static int detection_id_count_;
    int detection_id_;
    int nth_frame_;
    cv::Scalar box_color_;
  };

}

#endif