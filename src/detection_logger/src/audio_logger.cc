#include <string>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>

#include "audio_logger.h"

namespace sarwai {

  AudioLogger::AudioLogger(std::string base_filepath) : DetectionLogger(base_filepath) {}

  AudioLogger::AudioLogger() {}

  void AudioLogger::Log(struct AudioDetectionData data) {
    LocalSaveDetectionData(data);
  }

  void AudioLogger::LocalSaveDetectionData(struct AudioDetectionData data) {
    std::string csv_line = GenerateStringCSV(data);
    std::string full_text_log_filepath = log_filepath_ + boost::filesystem::path::preferred_separator + "audio-detections.csv";

    std::ofstream outfile(full_text_log_filepath, std::ofstream::app | std::ofstream::out);
    if (!outfile.is_open()) {
      // std::cout << "Couldn't open '" << full_text_log_filepath << "'\n";
      return; 
    }


    //std::string command = "scp -i ~/.ssh/hmtec2.pem ~/programming/sarwai/audio/" + data.audio_filename +" ubuntu@52.24.126.225:/home/ubuntu/visual-detection-images/";
    //system(command.c_str());
    outfile << csv_line;
    outfile.close();
  }

  std::string AudioLogger::GenerateStringCSV(struct AudioDetectionData data) {
    std::stringstream csv_line;
    csv_line <<
    // "audio" field added for integration to the socketio server
    "audio" << "," <<
    data.robot_id << "," <<
    data.timestamp << "," <<
    data.confidence << "," <<
    // Commas may possibly appear in this field. Enclosed in quotes to maintain CSV format.
    // Here if necessary
    // "\"" << data.transcript << "\"" << "," <<
    data.transcript << "," <<
    data.audio_filename << 
    "\n";

    return csv_line.str();

  }
}
