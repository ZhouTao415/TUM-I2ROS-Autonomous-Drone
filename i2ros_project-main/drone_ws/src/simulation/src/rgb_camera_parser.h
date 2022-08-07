#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "unity_stream_parser.h"
#include "TCPImageServer.h"

class RGBCameraParser : public UnityStreamParser {
public:
  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    auto img = ParseImage(stream_reader);
    PublishImage(header, img, time_offset);
    return true;
  }

  float GetFieldOfView() const {
    return fov_;
  }

  float GetMaxRange() const {
    return clip_plane_;
  }

protected: 

  virtual void PublishImage(const UnityHeader& header, const ImageData& img, double time_offset) {
    if(image_publishers_.find(header.name) == image_publishers_.end()) {
      image_publishers_.insert(std::make_pair(header.name, nh_.advertise<sensor_msgs::Image>(header.name + "/image_raw", 10)));
      info_publishers_.insert(std::make_pair(header.name, nh_.advertise<sensor_msgs::CameraInfo>(header.name + "/camera_info", 10)));
    }

    sensor_msgs::Image img_msg;
    sensor_msgs::CameraInfo info_msg;
    ImageToRos(img, header.name, &img_msg);
    img_msg.header.stamp = ros::Time(header.timestamp + time_offset);
    BuildCameraInfoMessage(img_msg, info_msg);

    image_publishers_[header.name].publish(img_msg);
    info_publishers_[header.name].publish(info_msg);
  }

  virtual void ImageToRos(const ImageData& img, 
                  const std::string& frame, 
                  sensor_msgs::Image* img_msg) const {
    img_msg->width = img.width;
    img_msg->height = img.height;
    img_msg->header.stamp = ros::Time(img.time_sec, img.time_nsec);
    img_msg->header.frame_id = frame;
    img_msg->encoding = "rgb8";
    img_msg->step = img.width * 3;
    img_msg->data = std::vector<uint8_t>(img.data.get(), img.data.get() + img.height * img_msg->step);

    if(monochrome_) {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(*img_msg, sensor_msgs::image_encodings::MONO8);
      *img_msg = *cv_ptr->toImageMsg();
    }
  }

  virtual void BuildCameraInfoMessage(sensor_msgs::Image const& img_msg,
                                 sensor_msgs::CameraInfo & info_msg) {

    info_msg.width = img_msg.width;
    info_msg.height = img_msg.height;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = img_msg.header.frame_id;
    info_msg.header.stamp = img_msg.header.stamp;

    double cx = img_msg.width * 0.5;
    double cy = img_msg.height * 0.5;
    double fx = 0.5 * img_msg.height / tan(fov_ * M_PI_2 / 180.0);
    double fy = fx;

    info_msg.K = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    info_msg.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    info_msg.D = {0, 0, 0, 0, 0};
    info_msg.P = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1};
  }

  ImageData ParseImage(TCPStreamReader& stream_reader) {
    if(!image_server_) {
      image_server_ = std::unique_ptr<TCPImageServer>(new TCPImageServer(&stream_reader, true));
    }
    monochrome_ = stream_reader.ReadInt() == 1;
    fov_ = stream_reader.ReadFloat();
    clip_plane_ = stream_reader.ReadFloat();
    return image_server_->GetImage();
  }


  std::unique_ptr<TCPImageServer> image_server_;
private:
  float fov_;
  float clip_plane_;
  bool monochrome_;
  std::unordered_map<std::string, ros::Publisher> image_publishers_;
  std::unordered_map<std::string, ros::Publisher> info_publishers_;
  ros::NodeHandle nh_;
};
