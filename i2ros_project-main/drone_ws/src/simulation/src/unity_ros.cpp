#include <vector>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include "unity_stream_parser.h"
#include "rgb_camera_parser.h"
#include "depth_camera_parser.h"
#include "fisheye_camera_parser.h"
#include "imu_parser.h"
#include "true_state_parser.h"
#include "unity_command_stream.h"


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unity_ros");
  ros::NodeHandle n;
  ros::NodeHandle nh_param("~");

  float infinite_distance;
  nh_param.param("infinite_distance", infinite_distance, 0.0f);

  ROS_DEBUG("Putting pixel out of max range to %f.", infinite_distance);
  ROS_INFO("Starting TCPStreamReader");

  if(argc!=3){
    ROS_ERROR("Invalid number of parameters\nusage: unity_ros host port");
    return -1;
  }

  char *host = argv[1];
  char *port = argv[2];
  
  TCPStreamReader stream_reader(host, port);
  ROS_INFO("Waiting for connection...");
  stream_reader.WaitConnect();
  ROS_INFO("Got a connection...");

  // IMUParser imu_parser;
  // UnityCommandStream command_stream("127.0.0.1", "9999");
  ros::Publisher pub = n.advertise<rosgraph_msgs::Clock>("clock", 100);
  rosgraph_msgs::Clock sim_time; 

  std::vector<std::shared_ptr<UnityStreamParser>> stream_parsers(UnityMessageType::MESSAGE_TYPE_COUNT);

  stream_parsers[UnityMessageType::UNITY_STATE] = std::make_shared<TrueStateParser>();
  stream_parsers[UnityMessageType::UNITY_IMU] = std::make_shared<IMUParser>();
  stream_parsers[UnityMessageType::UNITY_CAMERA] = std::make_shared<RGBCameraParser>();
  stream_parsers[UnityMessageType::UNITY_DEPTH] = std::make_shared<DepthCameraParser>(infinite_distance);
  stream_parsers[UnityMessageType::UNITY_FISHEYE] = std::make_shared<FisheyeCameraParser>();
  
  while (stream_reader.Good() && ros::ok()) {    
    uint32_t magic = stream_reader.ReadUInt();


    if(magic == 0xDEADC0DE) {
      double ros_time = ros::Time::now().toSec();
      UnityHeader header;
      header.type = static_cast<UnityMessageType>(stream_reader.ReadUInt());
      uint64_t timestamp_raw = stream_reader.ReadUInt64();
      header.timestamp = static_cast<double>(timestamp_raw) * 1e-7;
      header.name = stream_reader.ReadString();
      sim_time.clock = ros::Time(header.timestamp);
      pub.publish(sim_time);
      if(header.type < UnityMessageType::MESSAGE_TYPE_COUNT) {
        stream_parsers[header.type]->ParseMessage(header, stream_reader);
      }
    } else {
      ROS_ERROR("Stream corrupted, could not parse unity message");
    }

    ros::spinOnce();
  }

  return 0;
}
