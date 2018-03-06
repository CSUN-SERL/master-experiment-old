#ifndef SARWAI_DETECTION_LOGGER_SOCKETIO_VISUAL_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_SOCKETIO_VISUAL_LOGGER_H_

#include <string>

#include "sio_client.h"
#include "sio_message.h"
#include "sio_socket.h"

#include "visual_logger.h"

namespace sarwai {
  class SocketIOVisualLogger : public VisualLogger {
    public:
    SocketIOVisualLogger(std::string base_filepath,
        std::string host_addr,
        int port);

    void Log(cv::Mat image, struct VisualDetectionData);
    void Log(std::string image_filename, struct VisualDetectionData);

    private:
    std::string visual_detection_event_name_;
    std::string query_emit_event_name_;
    std::string host_addr_;
    int port_;
    sio::client socket_client_;

    void ReceiveQueryId(sio::event &);
    void SendData(struct VisualDetectionData data, std::string image_filename);
    std::string GenerateJSONString(struct VisualDetectionData data, std::string image_filename);
  };
}

#endif