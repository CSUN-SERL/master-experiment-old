#include "face_identifier_model.h"

#include <math.h>

#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>

#include "detection_frame_id.h"
#include "face_classifier_manager.h"

namespace sarwai {

  FaceIdentifierModel::FaceIdentifierModel() {
    this->similarity_model_ = cv::face::LBPHFaceRecognizer::create(2,10,8,8);
//    this->similarity_model_ = cv::face::createLBPHFaceRecognizer(
//        2,
//        10,
//        8,
//        8
//    );
  }

  DetectionSimilarityAssociation FaceIdentifierModel::RunFacePrediction(
    cv::Mat image, DetectionFrameId* detection_id) {
    
    DetectionSimilarityAssociation association;
    association.to_be_associated = detection_id;

    int label = 0;
    double confidence;
    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);      
    cv::imshow("grey face", greyImage);
    cv::waitKey(1);
    this->similarity_model_->predict(greyImage, label, confidence);
    association.compared_against = frame_id_map_[label];
    association.confidence = confidence;
    return association;
  }

  void FaceIdentifierModel::ReceiveImage(
    cv::Mat image,
    DetectionFrameId* frame_id,
    cv::Rect roi,
    std::vector<cv::Rect> detected_faces) {

    if (this->done_receiving_) {
      return;
    }

    images_.push_back(image);
    initial_rois_.push_back(roi);
    faces_per_image_.push_back(detected_faces);
    frame_ids_.push_back(frame_id);
    
  }

  void FaceIdentifierModel::DoneReceivingImages() {
    this->done_receiving_ = true;
    TrainModel();
  
  }

  void FaceIdentifierModel::TrainModel() {
    std::vector<int> labels;
    std::vector<cv::Mat> face_images;

    std::string str_label;


    // Flatten face 2d vector and generate labels
    for (int i = 0; i < this->frame_ids_.size(); i++) {

        std::vector<cv::Rect> crop_rects = faces_per_image_.at(i);
        for (int j = 0; j < crop_rects.size() && j == 0; j++) {

          cv::Mat cropped_image(images_.at(i), crop_rects.at(j));
          cv::Mat copied_cropped_image = cropped_image.clone();
          // Concert to grey scale
          cv::Mat greyMat;
          cv::cvtColor(copied_cropped_image, greyMat, cv::COLOR_BGR2GRAY);        
          cv::imshow("training on", greyMat);
          cv::waitKey(1);
          face_images.push_back(greyMat);

          str_label = FaceClassifierManager::GenerateImageLabel(
            frame_ids_[i]->DetectionId(), frame_ids_[i]->FrameId(), j);
          
          int int_label = labels.size();

          frame_id_map_[int_label] = frame_ids_[i];

          label_map_[int_label] = str_label;
          labels.push_back(int_label);
        }
    }
    labels_ = labels;

    if (labels.size() == 0) {
      return;
    }

    try {
      ROS_INFO("Training %s", str_label.c_str());
      this->similarity_model_->train(face_images, labels);
      this->done_training_ = true;
      ROS_INFO("Done Training %s", str_label.c_str());
    }
    catch (cv::Exception e) {
      // don't stop... belieeeeeving
    }
  }
  bool FaceIdentifierModel::IsDoneTraining() {
    return done_training_;
  }
}
