#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>
#include <math.h>


class TrajectoryPlanner {
public:
    //friend class TrajPublisher;
    ros::Subscriber current_position;
    ros::Subscriber planned_path;
    ros::Subscriber expected_height_subscriber;

    Eigen::Vector3d kp, ki, kd;             // controller gains of PID controller
    double kp_yaw, ki_yaw, kd_yaw;         

    Eigen::Vector3d cur_pos;       // current position in world coordinate system
    Eigen::Vector3d des_pos;       // desired postiion in world coordinate system

    double cur_yaw;                // current orientation in world coordinate system
    double des_yaw;                // desired orientation in world coordinate system
    double expected_height{4.0};
    double z_offset{0.0};         // offset in z achse

    bool cur_pos_initialized_;
    bool des_pos_initialized_;
    bool start_explore{false};

    geometry_msgs::Twist des_vel;  // desired velocity
    geometry_msgs::Twist des_acc;  // desired acceleration
    

    TrajectoryPlanner();


    void currentStateCallBack(const nav_msgs::Odometry& cur_state);

    void setParam();

    void pathCallBack(const nav_msgs::Path& path);

    void height_callback(const std_msgs::Float64& ehmsg);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle n_;

    Eigen::Vector3d integrated_e{0.0, 0.0, 0.0};
    Eigen::Vector3d filtered_e_dot{0.0, 0.0, 0.0};
    
    double integrated_e_yaw{0.0};
    double filtered_e_dot_yaw{0.0};

    Eigen::Vector3d min_bound{-0.02, -0.02, -0.02}; 
    Eigen::Vector3d max_bound{0.02, 0.02, 0.02}; 

    double min_bound_yaw{-0.2};
    double max_bound_yaw{0.2};

};
