#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "unity_stream_parser.h"

class IMUParser : public UnityStreamParser {
public:
  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    float ax, ay, az;
    float gx, gy, gz;

    ax = stream_reader.ReadFloat();
    ay = stream_reader.ReadFloat();
    az = stream_reader.ReadFloat();
    
    gx = stream_reader.ReadFloat();
    gy = stream_reader.ReadFloat();
    gz = stream_reader.ReadFloat();

    if(publishers_.find(header.name) == publishers_.end()) {
      publishers_.insert(std::make_pair(header.name, nh_.advertise<sensor_msgs::Imu>(header.name, 10)));
    } 

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = header.name;
    imu_msg.header.stamp = ros::Time(header.timestamp + time_offset);
    imu_msg.angular_velocity.x = -gx;
    imu_msg.angular_velocity.y = -gz;
    imu_msg.angular_velocity.z = -gy;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = az;
    imu_msg.linear_acceleration.z = ay;

    publishers_[header.name].publish(imu_msg);
    return true;
  }

private:
  std::unordered_map<std::string, ros::Publisher> publishers_;
  ros::NodeHandle nh_;
};