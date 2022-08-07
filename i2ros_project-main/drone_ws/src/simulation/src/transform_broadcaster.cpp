#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "geometry_msgs/PoseStamped.h"

class TransfromBroadcaster{
  public:
	TransfromBroadcaster(char * topic, char * parent, char * frame) {
    topic_ = topic;
    parent_ = parent;
    frame_ = frame;

		pose_sub_ = nh_.subscribe(topic, 1, &TransfromBroadcaster::OnPose, this);
	}

  private:
  char *topic_, *parent_, *frame_;
  ros::NodeHandle nh_;
	ros::Subscriber pose_sub_;
  tf2_ros::TransformBroadcaster br_;

	void OnPose(geometry_msgs::PoseStamped const& pose) {
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = pose.header.stamp;
		transformStamped.header.frame_id = parent_;
		transformStamped.child_frame_id = frame_;
		transformStamped.transform.translation.x = pose.pose.position.x;
		transformStamped.transform.translation.y = pose.pose.position.y;
		transformStamped.transform.translation.z = pose.pose.position.z;
		transformStamped.transform.rotation.x = pose.pose.orientation.x;
		transformStamped.transform.rotation.y = pose.pose.orientation.y;
		transformStamped.transform.rotation.z = pose.pose.orientation.z;
		transformStamped.transform.rotation.w = pose.pose.orientation.w;

		br_.sendTransform(transformStamped);
		}
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "transform_broadcaster");
  if(argc != 4)
  {
    ROS_ERROR("Invalid number of parameters\nusage: transform_broadcaster topic parent frame");
    return -1;
  }
  char* topic = argv[1];
  char* parent = argv[2];
  char* frame = argv[3];

	TransfromBroadcaster transform_broadcaster(topic, parent, frame);

	ros::spin();

}
