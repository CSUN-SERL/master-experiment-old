#ifndef SARWAI_DETECTION_LOGGER_SOCKETIO_AUDIO_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_SOCKETIO_AUDIO_LOGGER_H_

#include <string>

#include <sio_client.h>

#include "audio_logger.h"
#include "audio_detection_data.h"

namespace sarwai {
  class SocketIOAudioLogger : public AudioLogger {
    public:
    SocketIOAudioLogger(std::string host_addr, int port);

    void Log(struct AudioDetectionData);

    private:
    // Identifying event name socketio will emit on
    std::string audio_detection_event_name_;

    std::string host_addr_;
    int port_;

    sio::client socket_client_;
    // ?
    // sio::client

    std::string GenerateJSONString (struct AudioDetectionData);
    void SendData(struct AudioDetectionData data);
    void ReceiveQueryId(sio::event &);
  };
}

#endif