#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

#include <boost/filesystem.hpp>

#include "visual_logger.h"

namespace sarwai {
  
  VisualLogger::VisualLogger(std::string base_filepath) : DetectionLogger(base_filepath) {
    image_suffix_iterator_ = 1;
  }

  VisualLogger::VisualLogger() {}

  std::string VisualLogger::Log(cv::Mat image, struct VisualDetectionData data) {
    std::string image_filename = SaveImage(image, data.robot_id);
    LocalSaveDetectionData(data, image_filename);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return image_filename;
  }

  std::string VisualLogger::SaveImage(cv::Mat image, int robot_id) {
    std::string image_filename = GenerateImageFilename(robot_id);
    std::string full_image_path = log_filepath_ + boost::filesystem::path::preferred_separator + image_filename;
    cv::imwrite(full_image_path, image);
    
    // BURN THIS WITH FIRE!!!
    std::string command = "scp " + full_image_path + " station@station5:/home/station/visual-detection-images/";

    system(command.c_str());

    return image_filename;
  }

  void VisualLogger::LocalSaveDetectionData(struct VisualDetectionData data, std::string saved_image_filename) {
    if (log_filepath_ == "") {
      std::cout << "No log directory specified. Will not log to filesystem\n";
      return;
    }

    std::string csv_line = GenerateStringCSV(data, saved_image_filename);
    std::string full_text_log_filepath = log_filepath_ + boost::filesystem::path::preferred_separator + "visual-detections.csv";
    
    std::cout << "Writing: " << csv_line << "To dir: " << full_text_log_filepath << "\n";

    std::ofstream outfile(full_text_log_filepath, std::ofstream::app | std::ofstream::out);
    if(!outfile.is_open()) {
      std::cerr << "Couldn't open " << full_text_log_filepath << std::endl;
      return;
    }

    outfile << csv_line;
    outfile.close();
  }

  std::string VisualLogger::GenerateStringCSV(struct VisualDetectionData data, std::string saved_image_filename) {
    std::stringstream csv_line;
    csv_line <<
    // "visual" field added for integration to the socketio server
    "visual" << "," <<
    data.object_class << "," <<
    data.robot_id << "," <<
    data.confidence_rating << "," <<
    data.timestamp << "," <<
    data.left_x_coord << "," <<
    data.top_y_coord << "," <<
    data.box_width << "," <<
    data.box_height << "," <<
    saved_image_filename <<
    "\n";

    return csv_line.str();
  }

  std::string VisualLogger::GenerateImageFilename(int robot_id) {
    std::string image_name = "image-robot-" + std::to_string(robot_id) + "-" + std::to_string(ImageSuffixIterator()) + ".png";
    return image_name;
  }

  int VisualLogger::ImageSuffixIterator() {
    // increment after return
    return image_suffix_iterator_++;
  }
}
