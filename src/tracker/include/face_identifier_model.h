#ifndef SARWAI_DETECTION_TRACKER_FACE_IDENTIFIER_MODEL_H_
#define SARWAI_DETECTION_TRACKER_FACE_IDENTIFIER_MODEL_H_

#include <vector>
#include <map>

// #include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
// #include "opencv2/highgui.hpp"


#include "detection_frame_id.h"
#include "detection_similarity_association.h"

namespace sarwai {

  class FaceIdentifierModel {
    public:
    FaceIdentifierModel();
    // ~FaceIdentifierModel();

    void ReceiveImage(cv::Mat, DetectionFrameId*, cv::Rect roi, std::vector<cv::Rect> faces);
    void DoneReceivingImages();
    
    // double GetSimilarity(cv::Mat, cv::Rect roi);
    DetectionSimilarityAssociation RunFacePrediction(cv::Mat, DetectionFrameId*);

    bool IsDoneTraining();

    private:

    void TrainModel();

    bool done_training_ = false;
    bool done_receiving_ = false;
    cv::Ptr<cv::face::LBPHFaceRecognizer> similarity_model_;

    //cv::Ptr<cv::face::FaceRecognizer> similarity_model_;

    std::vector<cv::Mat> images_;
    std::vector<cv::Rect> initial_rois_;
    std::vector<std::vector<cv::Rect> > faces_per_image_;
    std::vector<DetectionFrameId*> frame_ids_;

    std::map<int, DetectionFrameId*> frame_id_map_;
    std::map<int, std::string> label_map_;
    std::vector<int> labels_;
  };
}

#endif
