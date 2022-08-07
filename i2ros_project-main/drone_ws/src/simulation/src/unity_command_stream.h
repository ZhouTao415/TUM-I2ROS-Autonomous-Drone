#pragma once

#include <ros/ros.h>
#include <libsocket/inetclientstream.hpp>

class UnityCommandStream {
public:
  UnityCommandStream(std::string host, std::string port) {
    ros::NodeHandle nh("~");
    command_sub_ = nh.subscribe("command_topic", 100, &UnityCommandStream::CommandCallback, this);
    socket_ptr_ = std::unique_ptr<libsocket::inet_stream>(new libsocket::inet_stream(host,
                                                             port,
                                                             LIBSOCKET_IPv4, 0));
  }
  
  void SendCommand(std::string command) {
    (*socket_ptr_) << command;
  }

private:
  void CommandCallback(const std_msgs::String &command) {
    SendCommand(command.data);
  }

  ros::Subscriber command_sub_;
  std::unique_ptr<libsocket::inet_stream> socket_ptr_;
};