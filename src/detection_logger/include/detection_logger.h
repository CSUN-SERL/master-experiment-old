#ifndef SARWAI_DETECTION_LOGGER_DETECTION_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_DETECTION_LOGGER_H_

#include <string>

namespace sarwai {
  
  class DetectionLogger {
    public:
    DetectionLogger(std::string base_filepath, std::string mission_id);
    DetectionLogger(std::string base_filepath);
    // To be used when logging is done remotely
    DetectionLogger();
    protected:
    std::string log_filepath_;
    // Identifying suffix string to append to the log directory name.
    std::string mission_id_;
    void CreateLogDirectory(std::string full_dir_path);
  };
}

#endif