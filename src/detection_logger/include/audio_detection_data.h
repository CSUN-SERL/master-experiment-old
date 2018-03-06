#ifndef SARWAI_DETECTION_LOGGER_AUDIO_DETECTION_DATA_H_
#define SARWAI_DETECTION_LOGGER_AUDIO_DETECTION_DATA_H_

#include <string>

namespace sarwai {
  struct AudioDetectionData {
    int robot_id;
    int timestamp;
    float confidence;
    std::string transcript;
    std::string audio_filename;
  };
}

#endif