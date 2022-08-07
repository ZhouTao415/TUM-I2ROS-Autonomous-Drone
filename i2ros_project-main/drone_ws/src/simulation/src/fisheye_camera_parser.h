#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include "rgb_camera_parser.h"

class FisheyeCameraParser : public RGBCameraParser {
public:
  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    if(!image_server_) {
      image_server_ = std::unique_ptr<TCPImageServer>(new TCPImageServer(&stream_reader, true));
    }
    float alpha = stream_reader.ReadFloat();
    float chi = stream_reader.ReadFloat();
    float focal_length = stream_reader.ReadFloat();
    auto img = ParseImage(stream_reader);
    PublishImage(header, img, time_offset);
    return true;
  }
};