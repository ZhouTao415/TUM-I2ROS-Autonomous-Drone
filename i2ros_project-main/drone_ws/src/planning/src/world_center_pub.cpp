# include <ros/ros.h>
# include <geometry_msgs/PointStamped.h>
# include <geometry_msgs/PoseStamped.h>
# include <nav_msgs/Odometry.h>

class WorldCenterPublishNode{

  public:
    WorldCenterPublishNode():nh_param_("~"){
      pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

      std::string child_frame_id;
      nh_param_.param("child_frame_id", child_frame_id, std::string("Quadrotor"));

      world_center_.header.frame_id = "world";
      world_center_.pose.pose.position.x = 0;
      world_center_.pose.pose.position.y = 0;
      world_center_.pose.pose.position.z = 0;
      world_center_.pose.pose.orientation.x = 0;
      world_center_.pose.pose.orientation.y = 0;
      world_center_.pose.pose.orientation.z = 0;
      world_center_.pose.pose.orientation.w = 1;
      world_center_.child_frame_id = child_frame_id;
      world_center_.twist.twist.linear.x = 0;
      world_center_.twist.twist.linear.y = 0;
      world_center_.twist.twist.linear.z = 0;
      world_center_.twist.twist.angular.x = 0;
      world_center_.twist.twist.angular.y = 0;
      world_center_.twist.twist.angular.z = 0;
    }

    void publish_odom(){
      world_center_.header.stamp = ros::Time::now();
      pub_.publish(world_center_);
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param_;
    ros::Publisher pub_;
    nav_msgs::Odometry world_center_;

};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "world_center_pub");

	WorldCenterPublishNode world_center_publish_node;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    world_center_publish_node.publish_odom();

    ros::spinOnce();

    loop_rate.sleep();
  }


	ros::spin();
}
