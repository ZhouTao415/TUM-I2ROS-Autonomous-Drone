#pragma once

#include <string>
#include "TCPStreamReader.h"

enum UnityMessageType : uint32_t {
  UNITY_STATE = 0,
  UNITY_CAMERA,
  UNITY_IMU,
  UNITY_DEPTH,
  UNITY_FISHEYE,
  UNITY_DETECTIONS,
  MESSAGE_TYPE_COUNT
};

struct UnityHeader {
  UnityMessageType type;
  double timestamp;
  std::string name;
};

class UnityStreamParser {
public:
  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader, 
                            double time_offset = 0) = 0;
};
