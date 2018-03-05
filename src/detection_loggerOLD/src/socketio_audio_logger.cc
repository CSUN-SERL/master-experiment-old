#include <iostream>
#include <sstream>
#include <stdlib.h>

#include "socketio_audio_logger.h"
#include "audio_detection_data.h"

namespace sarwai {
  
  SocketIOAudioLogger::SocketIOAudioLogger(std::string host_addr, int port) {
    host_addr_ = host_addr;
    port_ = port;

    audio_detection_event_name_ = "detection-insert-query";

    std::string connection_string = host_addr_ + ":" + std::to_string(port_);
    socket_client_.connect(connection_string);
    socket_client_.socket("/socket.io")->on("query-id",
        std::bind(&SocketIOAudioLogger::ReceiveQueryId, this, std::placeholders::_1));
  }

  void SocketIOAudioLogger::ReceiveQueryId(sio::event &queryIdEvent) {
    int query_id = queryIdEvent.get_message()->get_int();
    std::cout << "received query id: " << query_id << "\n";
    std::stringstream iris_script_cmd;
    iris_script_cmd << "python ~/programming/sarwai/adaptation/clustering/evaluate.py " << query_id << " >> audio-iris-log.py";
    system(iris_script_cmd.str().c_str());
  }

  void SocketIOAudioLogger::Log(struct AudioDetectionData data) {
    SendData(data);
  }

  void SocketIOAudioLogger::SendData(struct AudioDetectionData data) {
    std::cout << "#################Sending audio query via socket.io################################" << std::endl;
    std::string json_data = GenerateJSONString(data);
    socket_client_.socket("/socket.io")->emit(audio_detection_event_name_, json_data);
  }

  std::string SocketIOAudioLogger::GenerateJSONString(struct AudioDetectionData data) {
    std::stringstream json;
    json << "{"
    <<"\""<<"type"<<"\":" << "\"" <<"audio-detection"<<"\""<<","
    << "\"" << "robotId" << "\":" << "\"" << data.robot_id << "\"" << ","
    << "\"" << "timestamp" << "\":" << "\"" << data.timestamp << "\"" << ","
    << "\"" << "confidence" << "\":" << "\"" << data.confidence << "\"" << ","
    << "\"" << "transcript" << "\":" << "\"" << data.transcript << "\"" << ","
    << "\"" << "filePath" << "\":" << "\"" << data.audio_filename << "\""
    << "}";

    return json.str();
  }
}