#ifndef SARWAI_DETECTION_LOGGER_VISUAL_DETECTION_DATA_H_
#define SARWAI_DETECTION_LOGGER_VISUAL_DETECTION_DATA_H_

#include <string>

namespace sarwai {
  struct VisualDetectionData {
    std::string object_class;
    float confidence_rating;
    int timestamp;
    int left_x_coord;
    int top_y_coord;
    int box_width;
    int box_height;
    unsigned int robot_id;
  };
}

#endif