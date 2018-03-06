#include <string>
#include <sstream>
#include <stdlib.h>

#include "socketio_visual_logger.h"

namespace sarwai {
  
  SocketIOVisualLogger::SocketIOVisualLogger(std::string base_filepath,
      std::string host_addr,
      int port) {

    host_addr_ = host_addr;
    port_ = port;

    visual_detection_event_name_ = "detection-insert-query";
    query_emit_event_name_ = "irisevaluatequery";

    std::string connection_string = host_addr_ + ":" + std::to_string(port_);
    std::cout << connection_string << "\n";
    socket_client_.connect(connection_string);
    socket_client_.socket("/socket.io")->on("query-id",
        std::bind( &SocketIOVisualLogger::ReceiveQueryId, this, std::placeholders::_1));
  }

  void SocketIOVisualLogger::Log(cv::Mat image, struct VisualDetectionData data) {
    std::string image_filename = SaveImage(image, data.robot_id);
    SendData(data, image_filename);
  }

  void SocketIOVisualLogger::Log(std::string image_filename, struct VisualDetectionData data) {
    std::cout << "#################Sending visual query via socket.io################################" << std::endl;
    SendData(data, image_filename);
  }

  void SocketIOVisualLogger::ReceiveQueryId(sio::event &queryIdEvent) {
    int query_id = queryIdEvent.get_message()->get_int();
    std::cout << "received query id: " << query_id << "\n";
    socket_client_.socket("/socket.io")->emit(query_emit_event_name_, std::to_string(query_id));
  }

  void SocketIOVisualLogger::SendData(struct VisualDetectionData data, std::string image_filename) {
    std::string json_data = GenerateJSONString(data, image_filename);
    std::string csv_data = GenerateStringCSV(data, image_filename);
    std::cout << "JSON: " << json_data << "\n";
    socket_client_.socket("/socket.io")->emit(visual_detection_event_name_, json_data);
  }

  std::string SocketIOVisualLogger::GenerateJSONString(struct VisualDetectionData data, std::string image_filename) {
    std::stringstream json;
    json << "{"
    <<"\""<<"type"<<"\":" << "\"" << "visual-detection" << "\"" << "," 
    <<"\""<<"timestamp"<<"\":" << "\"" << data.timestamp << "\"" << "," 
    <<"\""<<"robotId"<<"\":" << "\"" << data.robot_id << "\"" << "," 
    <<"\""<<"confidence"<<"\":" << "\"" << data.confidence_rating << "\"" << "," 
    <<"\""<<"filePath"<<"\":" << "\"" << image_filename << "\"" 
    <<"}";

    return json.str();
  }
}