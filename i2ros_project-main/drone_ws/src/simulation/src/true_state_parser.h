#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "unity_stream_parser.h"

class TrueStateParser : public UnityStreamParser {
public:
  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float rx, ry, rz;

    px = stream_reader.ReadFloat();
    py = stream_reader.ReadFloat();
    pz = stream_reader.ReadFloat();
    
    qx = stream_reader.ReadFloat();
    qy = stream_reader.ReadFloat();
    qz = stream_reader.ReadFloat();
    qw = stream_reader.ReadFloat();

    vx = stream_reader.ReadFloat();
    vy = stream_reader.ReadFloat();
    vz = stream_reader.ReadFloat();

    rx = stream_reader.ReadFloat();
    ry = stream_reader.ReadFloat();
    rz = stream_reader.ReadFloat();

    if(pose_publishers_.find(header.name) == pose_publishers_.end()) {
      pose_publishers_.insert(std::make_pair(header.name, nh_.advertise<geometry_msgs::PoseStamped>(header.name + "/pose", 10)));
      twist_publishers_.insert(std::make_pair(header.name, nh_.advertise<geometry_msgs::TwistStamped>(header.name + "/twist", 10)));
    }    

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "body"; //"odom_nav";
    pose_msg.header.stamp = ros::Time(header.timestamp + time_offset);
    
    pose_msg.pose.position.x = px;
    pose_msg.pose.position.y = pz;
    pose_msg.pose.position.z = py;

    pose_msg.pose.orientation.x = -qx;
    pose_msg.pose.orientation.y = -qz;
    pose_msg.pose.orientation.z = -qy;
    pose_msg.pose.orientation.w = qw;

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = "body"; //"odom_nav";
    twist_msg.header.stamp = pose_msg.header.stamp;
    
    twist_msg.twist.linear.x = vx;    
    twist_msg.twist.linear.y = vz;
    twist_msg.twist.linear.z = vy;

    twist_msg.twist.angular.x = rx;
    twist_msg.twist.angular.y = rz;
    twist_msg.twist.angular.z = ry;

    pose_publishers_[header.name].publish(pose_msg);
    twist_publishers_[header.name].publish(twist_msg);
    return true;
  }

private:
  std::unordered_map<std::string, ros::Publisher> pose_publishers_;
  std::unordered_map<std::string, ros::Publisher> twist_publishers_;
  ros::NodeHandle nh_;
};
