#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>

#include "libsocket/inetclientdgram.hpp"
#include "libsocket/exception.hpp"

mav_msgs::Actuators w_msg2;
float Arr[4];

class UDPPoseStreamer {
  public:
  
  UDPPoseStreamer(const std::string & udp_address,
                  const std::string & udp_port, 
                  const float x_offset, 
                  const float y_offset) 
  : dgram_client(LIBSOCKET_IPv4), 
    ip_address(udp_address), 
    port(udp_port), 
    nh("~"),
    x_shift(x_offset),
    y_shift(y_offset) {
    global_frame="true_body";
    sub = nh.subscribe("pose_topic", 1, &UDPPoseStreamer::PublishPose, this);
  }
  
  virtual ~UDPPoseStreamer() { dgram_client.destroy(); }
  
  //private:
  void PublishPose(const mav_msgs::Actuators& msg) {

      // TODO what is the purpose of this function?
      // note msg does not even seem to be used...
    float w[4] = {
      static_cast<float>(Arr[0]),
      static_cast<float>(Arr[1]),
      static_cast<float>(Arr[2]),
      static_cast<float>(Arr[3]),
    };
    
    bool accept = true;
    for(uint i=0;i<4;i++){
      if(std::isnan(w[i])){
          accept=false;
          break;
      }
    }

    if(accept){
        static const size_t int32_size = sizeof(uint32_t);
        static const size_t pose_size = sizeof(float) * 4;
        static const size_t packet_size = pose_size;

        uint8_t packet_data[packet_size];
        memcpy(packet_data , &w, pose_size);
        dgram_client.sndto(&packet_data, packet_size, ip_address, port);
    }else{
        std::cout <<"Received nans, not sending! \n";
    }
  }

  ros::Subscriber sub;
  libsocket::inet_dgram_client dgram_client;

  std::string ip_address;
  std::string port;
  std::string global_frame;

  float x_shift, y_shift;
  ros::NodeHandle nh;
};

void wCallback(const mav_msgs::Actuators& cmd)
{
  //streamer.PublishPose(msg);
  // print all the remaining numbers
  Arr[0]=cmd.angular_velocities[0];
  Arr[1]=cmd.angular_velocities[1];
  Arr[2]=cmd.angular_velocities[2];
  Arr[3]=cmd.angular_velocities[3];

  std::cout <<"Message received: "<< Arr[0] << " "  << Arr[1] << " "<< Arr [2] << " "  << Arr[3] << " " << "\n";

  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "w_to_unity");

  //ros::NodeHandle nh("~");
  // ros::NodeHandle n("~");
  ros::NodeHandle n;

  ros::Rate loop_rate(1000);

  std::string ip_address, port;
  float offset_x, offset_y;

  if(argc!=3){
    ROS_ERROR("Invalid number of parameters\nusage: w_to_unity host port");
    return -1;
  }

  port=argv[2];
  ip_address=argv[1];
  offset_x=0;
  offset_y=0;
  std::cout << "This node enables send communication to Unity\n";

  UDPPoseStreamer streamer(ip_address, port, offset_x, offset_y);

  float counter=0;
  ros::Subscriber sub = n.subscribe("rotor_speed_cmds", 1, wCallback);

  while (ros::ok())
    {
      streamer.PublishPose(w_msg2);

      ros::spinOnce();

      loop_rate.sleep();
    }
  return 0;
}
