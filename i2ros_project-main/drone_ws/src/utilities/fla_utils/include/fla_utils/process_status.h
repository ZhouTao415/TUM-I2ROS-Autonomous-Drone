// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <thread>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include "fla_msgs/ProcessStatus.h"

namespace fla_utils {

class ProcessStatus {
 public:
  ProcessStatus(const std::string name, const double rate)
    : name_(name),
      pid_(getpid()),
      publish_rate_(rate),
      message_(""),
      status_(0) {
    ros::NodeHandle nh;
    SetMessage("");
    SetStatus(0);
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    hostname_ = std::string(hostname);
    status_pub_ = nh.advertise<fla_msgs::ProcessStatus>("/globalstatus", 0);
    status_thread_ = std::thread(&ProcessStatus::Heartbeat, this);
  }

  void SetMessage(std::string message) {
    std::lock_guard<std::mutex> lg(mutex_);
    message_ = message;
  }

  void SetStatus(uint8_t status) {
    std::lock_guard<std::mutex> lg(mutex_);
    status_ = status;
  }

private:
  void Heartbeat() {
    while (ros::ok()) {
      fla_msgs::ProcessStatus ps;
      ps.pid = pid_;
      ps.name = name_;
      ps.hostname = hostname_;

      std::unique_lock<std::mutex> ul(mutex_);
      ps.status = status_;
      ps.message = message_;
      ul.unlock();
      
      status_pub_.publish(ps);

      publish_rate_.sleep();
    }

    return;
  }

  std::mutex mutex_;
  ros::Rate publish_rate_;
  std::thread status_thread_;
  int pid_;
  std::string name_;
  std::string hostname_;
  uint8_t status_;
  std::string message_;
  ros::Publisher status_pub_;
};
}  // namespace fla_utils
