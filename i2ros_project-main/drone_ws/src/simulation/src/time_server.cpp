#include <vector>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>

class TimeServerNode{

  public:
    TimeServerNode(int num_subs, char * names[]){
      pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 100);

      subs = std::vector<ros::Subscriber>(num_subs);
      for(int i=0;i<num_subs;i++){
        std::string name = std::string(names[i]);
        subs[i] = nh.subscribe("/"+name+"/clock", 1, &TimeServerNode::OnTime, this);
      }

      sim_time.clock = ros::Time(0);

    }

  private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    std::vector<ros::Subscriber> subs;
    rosgraph_msgs::Clock sim_time; 

    void OnTime(rosgraph_msgs::Clock const& new_time) {
      if (new_time.clock.sec > sim_time.clock.sec || (new_time.clock.sec == sim_time.clock.sec && new_time.clock.nsec > sim_time.clock.nsec)){
        sim_time = new_time;
        pub.publish(sim_time);
      }
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "time_server");
  TimeServerNode time_server_node(argc-1, argv+1);
  ros::spin();
}