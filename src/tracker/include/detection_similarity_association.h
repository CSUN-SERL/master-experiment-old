#ifndef SARWAI_DETECTION_TRACKER_DETECTION_SIMILARITY_ASSOCIATION_H_
#define SARWAI_DETECTION_TRACKER_DETECTION_SIMILARITY_ASSOCIATION_H_

#include "detection_frame_id.h"

namespace sarwai {

class DetectionSimilarityAssociation {
  public:
  DetectionSimilarityAssociation() {
    this->compared_against = nullptr;
    this->to_be_associated = nullptr;
  }

  DetectionFrameId* compared_against;
  DetectionFrameId* to_be_associated;
  double confidence;
  private:
};

}

#endif