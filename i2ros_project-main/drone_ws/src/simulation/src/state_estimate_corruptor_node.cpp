#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <fla_utils/param_utils.h>

#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>
#include <random>

class StateEstimateCorruptorNode {
 public:
	StateEstimateCorruptorNode() {
		// Subscribers
		pose_sub     = nh.subscribe("/true_pose", 1, &StateEstimateCorruptorNode::OnPose, this);
		velocity_sub = nh.subscribe("/true_twist", 1, &StateEstimateCorruptorNode::OnVelocity, this);

		// Publishers
		corrupted_pose_pub     = nh.advertise<geometry_msgs::PoseStamped>("/pose_est", 1);
		corrupted_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/twist_est", 1);
		corrupted_state_pub    = nh.advertise<nav_msgs::Odometry>("/current_state_est", 1);

		ros::NodeHandle pnh("~");
		fla_utils::SafeGetParam(pnh, "pos_white_sig", pos_white_sigma_);
		fla_utils::SafeGetParam(pnh, "drift_rw_factor", drift_rw_factor_);

		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
		srand(0);  // initialize the random seed

		ROS_INFO("Finished constructing the state estimate corruptor node");
	}

 private:

 	geometry_msgs::PoseStamped pose_corrupted_;

	void OnPose(geometry_msgs::PoseStamped const& pose) {
		++pose_count;

		// Apply drift / jumps
		if (virgin) {
			prev_jump_pose_ = pose.pose;
			drift_.position.x = 0.0;
			drift_.position.y = 0.0;
			drift_.position.z = 0.0;
		} else if (jump_seconds_ > 0.0) {
			double pose_dt = (pose.header.stamp - prev_pose_time_).toSec();

			// Calculate jump probability as a bernoulli parameter
			// With parameter p drawn based on the CDF of an exponential distribution
			const double bernoulli_p = 1.0 - std::exp(-pose_dt / jump_seconds_);
			const double uniform_random = (rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1);

			// If jump
			if (uniform_random < bernoulli_p) {
				// Time for a jump
				// Drift integrates as random walk over distance
				// Compute displacement since last update
				const double dx = pose.pose.position.x - prev_jump_pose_.position.x;
				const double dy = pose.pose.position.y - prev_jump_pose_.position.y;
				const double dz = pose.pose.position.z - prev_jump_pose_.position.z;

				// Sample jump
				geometry_msgs::Pose jump;
				jump.position.x = whiteNoise(drift_rw_factor_ * std::abs(dx));
				jump.position.y = whiteNoise(drift_rw_factor_ * std::abs(dy));
				jump.position.z = whiteNoise(drift_rw_factor_ * std::abs(dz));

				// Apply to current drift
				drift_.position.x += jump.position.x;
				drift_.position.y += jump.position.y;
				drift_.position.z += jump.position.z;

				prev_jump_pose_ = pose.pose;
			}
		}

		// Corrupt pose with instantaneous white noise and current drift
		geometry_msgs::PoseStamped pose_corrupted(pose);
		pose_corrupted.pose.position.x += drift_.position.x + whiteNoise(pos_white_sigma_);
		pose_corrupted.pose.position.y += drift_.position.y + whiteNoise(pos_white_sigma_);
		pose_corrupted.pose.position.z += drift_.position.z + whiteNoise(pos_white_sigma_);

		PublishCorruptedPose(pose_corrupted);

		// Update latest timestamps
		virgin = false;
		prev_pose_time_ = pose.header.stamp;
	}

	double whiteNoise(double sigma) {
		static std::random_device rd;
		static std::mt19937 gen(rd());

		std::normal_distribution<> d(0.0, sigma);
		return d(gen);
	}

	void OnVelocity(geometry_msgs::TwistStamped const& twist) {
		actual_velocity_global_x = twist.twist.linear.x;
		actual_velocity_global_y = twist.twist.linear.y;
		actual_velocity_global_z = twist.twist.linear.z;

		PublishCorruptedTwist(twist);
		PublishCorruptedState(twist);
	}

	void PublishCorruptedPose(geometry_msgs::PoseStamped const& corrupt_pose) {
		corrupted_pose_pub.publish(corrupt_pose);
		pose_corrupted_ = corrupt_pose;
	}

	void PublishCorruptedState(geometry_msgs::TwistStamped const& corrupt_twist) {
    nav_msgs::Odometry corrupted_state;
    //corrupted_state.pose = corrupt_pose;

    corrupted_state.header.stamp = corrupt_twist.header.stamp;
		corrupted_state.header.frame_id = "world";
		corrupted_state.child_frame_id = "body";

    corrupted_state.twist.twist = corrupt_twist.twist;
    corrupted_state.pose.pose = pose_corrupted_.pose;

    corrupted_state_pub.publish(corrupted_state);
	}

	void PublishCorruptedTwist(geometry_msgs::TwistStamped const& twist) {

		geometry_msgs::TwistStamped corrupted_twist;
		corrupted_twist = twist;
		corrupted_twist.twist.linear.x = twist.twist.linear.x * (1 + whiteNoise(drift_rw_factor_));
		corrupted_twist.twist.linear.y = twist.twist.linear.y * (1 + whiteNoise(drift_rw_factor_));
		corrupted_velocity_pub.publish(corrupted_twist);
	}

	void PublishBodyTransform(geometry_msgs::PoseStamped const& pose) {
		static tf2_ros::TransformBroadcaster br;
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = pose.header.stamp;
		transformStamped.header.frame_id = "world";
		transformStamped.child_frame_id = "body";
		transformStamped.transform.translation.x = pose.pose.position.x;
		transformStamped.transform.translation.y = pose.pose.position.y;
		transformStamped.transform.translation.z = pose.pose.position.z;
		transformStamped.transform.rotation.x = pose.pose.orientation.x;
		transformStamped.transform.rotation.y = pose.pose.orientation.y;
		transformStamped.transform.rotation.z = pose.pose.orientation.z;
		transformStamped.transform.rotation.w = pose.pose.orientation.w;

		br.sendTransform(transformStamped);
	}

	bool virgin = true;
	size_t pose_count = 0;

	ros::NodeHandle nh;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;

	ros::Publisher corrupted_pose_pub;
	ros::Publisher corrupted_velocity_pub;
	ros::Publisher corrupted_state_pub;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	tf2_ros::Buffer tf_buffer_;

	ros::Time prev_pose_time_;
	geometry_msgs::Pose prev_jump_pose_;
	geometry_msgs::Pose drift_;

	double actual_velocity_global_x = 0;
	double actual_velocity_global_y = 0;
	double actual_velocity_global_z = 0;

	double drift_rw_factor_ = 0.0;
	double pos_white_sigma_ = 0.0;
	double jump_seconds_ = 0.0;
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing state_estimate_corruptor node" << std::endl;

	ros::init(argc, argv, "StateEstimateCorruptorNode");

	StateEstimateCorruptorNode state_estimate_corruptor_node;

	ros::spin();

}
