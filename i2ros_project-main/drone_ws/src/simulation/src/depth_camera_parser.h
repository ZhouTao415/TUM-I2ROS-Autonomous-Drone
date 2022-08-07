#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include "rgb_camera_parser.h"

class DepthCameraParser : public RGBCameraParser {
public:
  DepthCameraParser() = default;
  DepthCameraParser(float inifinite_distance){
    inifinite_distance_ = inifinite_distance;
  }

protected: 

  virtual void ImageToRos(const ImageData& img, 
                          const std::string& frame, 
                          sensor_msgs::Image* img_msg) const override {
    
    img_msg->width = img.width;
    img_msg->height = img.height;
    img_msg->encoding = "32FC1";
    img_msg->header.frame_id = frame;
    img_msg->step = img.width * 4;

    const int pixels = img.width * img.height;
    img_msg->data = std::vector<uint8_t>(img.width * img.height * 4);

    auto img_data_ptr = img.data.get();
    
    float max_range_meters = GetMaxRange();

    for(int i = 0; i < pixels; i++) {
      uint8_t r = img_data_ptr[3*i];
      uint8_t g = img_data_ptr[3*i+1];
      uint8_t b = img_data_ptr[3*i+2];
      float depth = 1.0 * static_cast<float>(r)/255.0f + static_cast<float>(g)/255.0f * 1.0f/255.0f + static_cast<float>(b)/255.0f * 1.0f/65025.0f;

      float_t float_meters;
      if (depth > 0.99f) {
        float_meters = (float_t) (inifinite_distance_);
      }else{
        float_meters = (float_t) (depth * max_range_meters);
      }

      uint8_t *array;
      array = reinterpret_cast<uint8_t*>(&float_meters);

      img_msg->data[4*i] = array[0];
      img_msg->data[4*i+1] = array[1];
      img_msg->data[4*i+2] = array[2];
      img_msg->data[4*i+3] = array[3];
    }
  }

private:
  float inifinite_distance_;
};