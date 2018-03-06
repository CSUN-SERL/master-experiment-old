#include "face_classifier_manager.h"
#include "ros/ros.h"

namespace sarwai {

FaceClassifierManager::FaceClassifierManager() {
  this->front_face_cascade_ = new cv::CascadeClassifier();

  this->front_face_cascade_->load("/home/kenny/programming/sarwai/detection/haarcascade_frontalface_alt2.xml");
  ROS_INFO("classifier model loaded");
}

FaceClassifierManager::~FaceClassifierManager() {
  delete this->front_face_cascade_;
}

std::vector<cv::Rect> FaceClassifierManager::RunFacialDetection(
    cv::Mat image, DetectionFrameId* image_id, cv::Rect roi) {

  if (roi.x <= 0) {
    roi.x = 1;
  }

  if (roi.y <= 0) {
    roi.y = 1;
  }

  if (roi.x + roi.width > image.cols) {
    roi.width = image.cols - roi.x - 1;
  }

  if (roi.y + roi.height > image.rows) {
    roi.height = image.rows - roi.y - 1;
  }
  
  cv::Mat cropped_image(image, roi);
  cv::Mat cloned_cropped_image = cropped_image.clone();
  roi.x = 1;
  roi.y = 1;
  std::vector<cv::Rect> classified_faces;
  cv::rectangle(cloned_cropped_image, roi, cv::Scalar(0,255,0), 5,5);
  cv::imshow("image before factial detection", cloned_cropped_image);
  cv::waitKey(1);
  this->front_face_cascade_->detectMultiScale(
    cloned_cropped_image,
    classified_faces,
    1.1,
    3,
    0,
    cv::Size(50,50)
  );

  for (int i = 0; i < classified_faces.size(); i++) {
    cv::rectangle(cloned_cropped_image, classified_faces[i], cv::Scalar(0,0,255), 10,10);
    cv::imshow("freshly detected faces", cloned_cropped_image);
    cv::waitKey(1);
  }

  if (face_identifiers_.find(image_id->DetectionId()) == face_identifiers_.end()) {
    // New detection
    face_identifiers_[image_id->DetectionId()];
  }
  
  FaceIdentifierModel receiving_model = face_identifiers_[image_id->DetectionId()];

  receiving_model.ReceiveImage(cloned_cropped_image, image_id, roi, classified_faces);
  face_identifiers_[image_id->DetectionId()] = receiving_model;

  return classified_faces;
}

void FaceClassifierManager::DeactivateModel(int detection_id) {

  FaceIdentifierModel model = face_identifiers_[detection_id];
  model.DoneReceivingImages();
  face_identifiers_[detection_id] = model;
}


std::string FaceClassifierManager::GenerateImageLabel(
  int detection_id,
  int nth_frame,
  int classification_id) {
    std::string image_label = std::to_string(detection_id) +
    "." + std::to_string(nth_frame) + "." + std::to_string(classification_id);
    return image_label;
}

DetectionSimilarityAssociation FaceClassifierManager::FindDoppelganger(cv::Mat image,
    cv::Rect roi, DetectionFrameId* detection_id) {
  
  cv::imshow("finddoppleganger", image);
  if (roi.x <= 0) {
    roi.x = 1;
  }

  if (roi.y <= 0) {
    roi.y = 1;
  }

  if (roi.x + roi.width > image.cols) {
    roi.width = image.cols - roi.x - 1;
  }

  if (roi.y + roi.height > image.rows) {
    roi.height = image.rows - roi.y - 1;
  }

  roi.x = 1;
  roi.y = 1;
  cv::Mat cropped_image(image, roi);
  cv::Mat cloned_cropped_image = cropped_image.clone();
  std::vector<cv::Rect> classified_faces;
  this->front_face_cascade_->detectMultiScale(
    cloned_cropped_image,
    classified_faces,
    1.1,
    3,
    0,
    cv::Size(50,50)
  );

  DetectionSimilarityAssociation most_similar;
  most_similar.confidence = 1000000;
  if (classified_faces.size() == 0) {
    return most_similar;
  }

  std::map<int, FaceIdentifierModel>::iterator iter;
  for (iter = face_identifiers_.begin(); iter != face_identifiers_.end(); ++iter) {
    int key =  iter->first;
    FaceIdentifierModel model = face_identifiers_[key];
    if (model.IsDoneTraining()) {
      cv::Mat face_image(cloned_cropped_image, classified_faces[0]);
      cv::Mat cloned_face_image = face_image.clone();
      DetectionSimilarityAssociation association = model.RunFacePrediction(cloned_face_image, detection_id);
      if (association.confidence < most_similar.confidence) {
        most_similar = association;
      }
    }
  }

  return most_similar;
}

}