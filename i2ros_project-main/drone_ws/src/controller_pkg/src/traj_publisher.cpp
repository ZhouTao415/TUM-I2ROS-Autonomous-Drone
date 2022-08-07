#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>
#include <math.h>
#include "traj_planner.h"


#define FLY_TOWARDS_TARGET 1

#define PI M_PI

#define TFOUTPUT 0


class TrajPublisher
{
private:
    ros::NodeHandle n_;
    ros::Publisher desired_state_pub;
    ros::Publisher drone_speeds; 
    ros::Publisher current_delta_z;
    ros::Publisher current_offset_z;

    // hearing from topic state machine
    ros::Subscriber received_state_machine;
    ros::Subscriber expected_height_subscriber;
    ros::Subscriber follow_path;

    // represent the current state
    int drone_state;

    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;

    int count;
    double expected_height;
    bool rotation_first_;
    bool rotation_while_explore_;
    double rot_factor;
    tf::Vector3 origin;
    tf::Transform desired_pose;
    double t;
    TrajectoryPlanner trajectory_planner_;
    std_msgs::Float64 delta_z;
    std_msgs::Float64 offset_z;

public:
    // constructor
    TrajPublisher(bool rotation_first, bool rotation_while_explore)
    {
        ROS_INFO("initialize trajectory publisher.");
        desired_state_pub = n_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("Commands/desired_state", 1);
        drone_speeds = n_.advertise<geometry_msgs::Twist>("Commands/drone_speeds", 1);
        current_delta_z = n_.advertise<std_msgs::Float64>("current_delta_z", 10);
        current_offset_z = n_.advertise<std_msgs::Float64>("current_offset_z", 10);


        // Timer
        ros::Rate loop_rate(20);
        int count = 0;
        rot_factor = 0;

        drone_state = 1;
        rotation_first_ = rotation_first;
        rotation_while_explore_ = rotation_while_explore;

        // initialize drone state
        origin.setX(0);
        origin.setY(0);
        origin.setZ(0);
        desired_pose = tf::Transform::getIdentity();

        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;

        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


        received_state_machine  = n_.subscribe("state_machine/state", 1, &TrajPublisher::State_callback, this);
        expected_height_subscriber = n_.subscribe("state_machine/expected_height", 1, &TrajPublisher::height_callback, this);

    }

    void State_callback(const std_msgs::Int64& state_msg)
    { 
        
        if(!trajectory_planner_.cur_pos_initialized_){
          // ROS_INFO("cur_pos not initialized.");
          return;
        }
        ROS_INFO("cur_pos initialized.");
        rot_factor += 0.1;   // increment rotation factor to let drone has a continuous self rotation
        tf::TransformBroadcaster br;
        double timeScale = 1.0;
        tf::Quaternion q;
        q.setRPY(0,0,0);
        drone_state = state_msg.data;
        double target_height = expected_height;

        // initial state
        if (drone_state == 1)
        {
            return;
            // double cur_x{trajectory_planner_.cur_pos.x()};
            // double cur_y{trajectory_planner_.cur_pos.y()};
            // trajectory_planner_.des_pos << cur_x, cur_y, 0.2;
            // trajectory_planner_.des_pos_initialized_ = true;
            // ROS_INFO("Initial State");
        }

        // launch state
        else if (drone_state == 2)
        {
            q.setRPY(0,0,-rot_factor/timeScale);
            ROS_INFO_STREAM_ONCE("User press Space key, launching");
            double cur_x{trajectory_planner_.cur_pos.x()};
            double cur_y{trajectory_planner_.cur_pos.y()};
            

            trajectory_planner_.des_pos << cur_x, cur_y, expected_height;
            trajectory_planner_.des_pos_initialized_ = true;
            // ToDo still has bug here, can not continuous rotate
            if (rotation_first_)
            {
                q.setRPY(0, 0, -rot_factor/timeScale);
                trajectory_planner_.des_vel.angular.x = 0.0;
                trajectory_planner_.des_vel.angular.y = 0.0;
                trajectory_planner_.des_vel.angular.z = -1.0;
            }

        }

        // follow move base/explore state
        else if (drone_state == 3)
        {
            follow_path = n_.subscribe("move_base/TrajectoryPlannerROS/global_plan", 2, &TrajectoryPlanner::pathCallBack, &trajectory_planner_);
            
            trajectory_planner_.des_vel.angular.x = 0.0;
            trajectory_planner_.des_vel.angular.y = 0.0;

            // keep rotation if explore node is not start
            if (!trajectory_planner_.start_explore) { 
                q.setRPY(0, 0, -rot_factor/timeScale); // rotate in yaw
                trajectory_planner_.des_vel.angular.z = -1.0;
            } else {
                if (rotation_while_explore_)
                {
                    q.setRPY(0, 0, -rot_factor/timeScale); // rotate in yaw
                    trajectory_planner_.des_vel.angular.z = -1.0;
                }
                else
                {
                    q.setRPY(0, 0, trajectory_planner_.des_yaw); // rotate in yaw
                    trajectory_planner_.des_vel.angular.z = trajectory_planner_.des_vel.angular.z;
                }
            }    
        }

        // mapping ends and landed flag received
        else if (drone_state == 4)
        {
            double cur_x{trajectory_planner_.cur_pos.x()};
            double cur_y{trajectory_planner_.cur_pos.y()};
            trajectory_planner_.des_pos << cur_x, cur_y, 0.5;
            trajectory_planner_.des_pos_initialized_ = true;
            target_height = 0.5;
        }

        // landed keep the drones still
        else if (drone_state == 5)
        {
            q.setRPY(0,0,0);
            double cur_x{trajectory_planner_.cur_pos.x()};
            double cur_y{trajectory_planner_.cur_pos.y()};
            trajectory_planner_.des_pos << cur_x, cur_y, -1.0;
            trajectory_planner_.des_pos_initialized_ = true;
            target_height = -1.0;
            // trajectory_planner_.des_vel.linear.x = 0.0;
            // trajectory_planner_.des_vel.linear.y = 0.0;
            // trajectory_planner_.des_vel.linear.z = 0.0;
            // trajectory_planner_.des_acc.angular.x = 0.0;
            // trajectory_planner_.des_acc.angular.y = 0.0;
            // trajectory_planner_.des_acc.angular.z = 0.0;
        }

        if(!trajectory_planner_.des_pos_initialized_){
          return;
        }

        // add additional d-control in z direction
        // double cur_z{trajectory_planner_.cur_pos.z()};
        // double z_diff = expected_height - cur_z;
        // if (std::abs(z_diff) > 0.3){
        //   z_offset_ += 0.1*z_diff;
        //   z_offset_ *= 0.9;
        // }

        tf::Vector3 displacement;
        tf::vectorEigenToTF(trajectory_planner_.des_pos, displacement);
        desired_pose.setOrigin(origin + displacement);

        desired_pose.setRotation(q);

        velocity.linear.x = trajectory_planner_.des_vel.linear.x;
        velocity.linear.y = trajectory_planner_.des_vel.linear.y;
        velocity.linear.z = trajectory_planner_.des_vel.linear.z;

        velocity.angular.x = 0;
        velocity.angular.y = 0;
        velocity.angular.z = trajectory_planner_.des_vel.angular.z;

        acceleration.angular.x = trajectory_planner_.des_acc.angular.x;
        acceleration.angular.y = trajectory_planner_.des_acc.angular.y;
        acceleration.angular.z = trajectory_planner_.des_acc.angular.z;

        // publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg_traj;
        msg_traj.transforms.resize(1);
        msg_traj.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg_traj.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg_traj.transforms[0].translation.z = desired_pose.getOrigin().z() + trajectory_planner_.z_offset;
        msg_traj.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg_traj.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg_traj.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg_traj.transforms[0].rotation.w = desired_pose.getRotation().getW();

        msg_traj.velocities.resize(1);
        msg_traj.velocities[0] = velocity;
        msg_traj.accelerations.resize(1);
        msg_traj.accelerations[0] = acceleration;
        desired_state_pub.publish(msg_traj);
        drone_speeds.publish(velocity);

        delta_z.data = trajectory_planner_.des_pos[2] - trajectory_planner_.cur_pos[2];
        current_delta_z.publish(delta_z);
        offset_z.data = trajectory_planner_.z_offset;
        current_offset_z.publish(offset_z);

        std::stringstream ss;
        ss << "desired position"
            << " x:" << desired_pose.getOrigin().x()
            << " y:" << desired_pose.getOrigin().y()
            << " z:" << desired_pose.getOrigin().z();
        
        // ROS_INFO("%s", ss.str().c_str());

// #if TFOUTPUT
//         if (av_desired == "Quadrotor")
//         {
//             br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
//                                             "world", "Quadrotor/av_desired"));
//         }
//         else if (av_desired == "Quadrotor2")
//         {
//             br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
//                                             "world", "Quadrotor2/av_desired"));
//         }
// #endif
    }
    void height_callback(const std_msgs::Float64& ehmsg)
    {
            expected_height = ehmsg.data;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_publisher");
    // TrajectoryPlanner trajectory_planner(n);
    bool rotation_first;
    bool rotation_while_explore;
    ros::NodeHandle nh_private("~");
    nh_private.param<bool>("rotation_first", rotation_first, true);
    nh_private.param<bool>("rotation_while_explore", rotation_while_explore, true);
    TrajPublisher Traj_publisher(rotation_first, rotation_while_explore);

    ros::spin();
    return 0;

}
