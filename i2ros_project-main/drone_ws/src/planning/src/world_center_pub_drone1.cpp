# include <ros/ros.h>
# include <geometry_msgs/PoseStamped.h>

class WorldCenterPublishNode1{

  public:
    WorldCenterPublishNode1(){
      pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple_drone1/goal", 100);

      world_center.header.frame_id = "world";
      world_center.pose.position.x = 20;
      world_center.pose.position.y = 30;
      world_center.pose.position.z = 0;
      world_center.pose.orientation.x = 0;
      world_center.pose.orientation.y = 0;
      world_center.pose.orientation.z = 0;
      world_center.pose.orientation.w = 1;
    }

    void publish_odom(){
      world_center.header.stamp = ros::Time::now();
      pub.publish(world_center);
    }

  private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    geometry_msgs::PoseStamped world_center;
    
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "world_center_pub_drone1");

	WorldCenterPublishNode1 world_center_publish_node1;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    world_center_publish_node1.publish_odom();

    ros::spinOnce();

    loop_rate.sleep();
  }


	ros::spin();
}
