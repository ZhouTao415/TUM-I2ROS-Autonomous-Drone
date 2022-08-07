#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include "traj_planner.h"


TrajectoryPlanner::TrajectoryPlanner(): n_("~"), kp(1.2, 1.2, 0.5), ki(0, 0, 0), kd(0, 0, 0){
    // init parameters form launch file
    setParam();
    ROS_INFO("initialize trajectory planner.");

    des_pos << 0.0, 0.0, 0.0;
    cur_pos_initialized_ = false;
    des_pos_initialized_ = false;
    current_position = nh_.subscribe("Estimation/current_state", 100, &TrajectoryPlanner::currentStateCallBack, this);
    // planned_path = nh_.subscribe("/local_path", 1, &TrajectoryPlanner::pathCallBack, this);
    expected_height_subscriber = nh_.subscribe("state_machine/expected_height", 100, &TrajectoryPlanner::height_callback, this);
}

void TrajectoryPlanner::currentStateCallBack(const nav_msgs::Odometry& cur_state) {
    if(!std::isnan(cur_state.pose.pose.position.x) && 
       !std::isnan(cur_state.pose.pose.position.y) && 
       !std::isnan(cur_state.pose.pose.position.z)) {  // avoid nan values
        
        cur_pos << cur_state.pose.pose.position.x, 
                   cur_state.pose.pose.position.y, 
                   cur_state.pose.pose.position.z;
        cur_pos_initialized_ = true;
        Eigen::Vector3d e;
        e = des_pos - cur_pos;
        ROS_INFO("current z: %.3f, dist from desired z: %.3f", cur_pos[2], e[2]);

        Eigen::Vector3d e_dot;
        e_dot << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;

        filtered_e_dot = 0.8 * filtered_e_dot + 0.2 * e_dot;

        Eigen::Vector3d output;
        output = kp.array() * e.array() + ki.array() * integrated_e.array() + kd.array() * filtered_e_dot.array();

        for (size_t i = 0; i < 3; ++i) {
            if (output[i] < min_bound[i]) {
                output[i] = min_bound[i];
            }
            else if (output[i] > max_bound[i]) {
                output[i] = max_bound[i];
            }
            else {
                integrated_e += e;
            }
        }

        // pid controller for yaw
        cur_yaw = std::atan(cur_state.pose.pose.position.y / cur_state.pose.pose.position.x);
        if (std::isnan(cur_yaw)) {
            cur_yaw = 0;
        }

        double e_yaw = des_yaw - cur_yaw;
        double e_dot_yaw = cur_state.twist.twist.angular.z;

        filtered_e_dot_yaw = 0.8 * filtered_e_dot_yaw + 0.2 * e_dot_yaw;

        double output_yaw = kp_yaw * e_yaw + ki_yaw * integrated_e_yaw + kd_yaw * filtered_e_dot_yaw;

        // output_yaw = output_yaw > max_bound_yaw? max_bound_yaw : output_yaw;
        // output_yaw = output_yaw < min_bound_yaw? min_bound_yaw : output_yaw;
        if (output_yaw < min_bound_yaw) {
                output_yaw = min_bound_yaw;
        }
        else if (output_yaw > max_bound_yaw) {
            output_yaw = max_bound_yaw;
        }
        else {
            integrated_e += e;
        }

        des_vel.linear.x = output.x();
        des_vel.linear.y = output.y();
        des_vel.linear.z = output.z();

        des_vel.angular.z = output_yaw;
    }
}

void TrajectoryPlanner::setParam() {
    std::vector<double> p;
    std::vector<double> i;
    std::vector<double> d;

    if (n_.getParam("kp", p)) {
        kp << p[0], p[1], p[2];
        kp_yaw = p[3]; 
        ROS_INFO("set kp: %.2f, %.2f, %.2f, %.2f", kp[0], kp[1], kp[2], kp_yaw);
    }
    if (n_.getParam("ki", i)) {
        ki << i[0], i[1], i[2];
        ki_yaw = i[3]; 
        ROS_INFO("set ki: %.2f, %.2f, %.2f, %.2f", ki[0], ki[1], ki[2], ki_yaw);
    }
    if (n_.getParam("kd", d)) {
        kd << d[0], d[1], d[2];
        kd_yaw = d[3];
        ROS_INFO("set kd: %.2f, %.2f, %.2f, %.2f", kd[0], kd[1], kd[2], kd_yaw);
    }
    if (n_.getParam("z_offset", z_offset)) {
        ROS_INFO("set offset for z: %.2f", z_offset);
    }
}

void TrajectoryPlanner::pathCallBack(const nav_msgs::Path& path) {
    start_explore = true;
    geometry_msgs::PoseStamped way_point = path.poses.back();
    // ROS_INFO("update path");

    des_pos << way_point.pose.position.x,
            way_point.pose.position.y,
            expected_height;

    des_pos_initialized_ = true;

    geometry_msgs::PoseStamped goal = path.poses.back();
    geometry_msgs::PoseStamped start = path.poses.front();

    double delta_x{ goal.pose.position.x - start.pose.position.x };
    double delta_y{ goal.pose.position.y - start.pose.position.y };

    double tan{delta_y / delta_x};
    des_yaw = std::atan(tan);

    if (std::isnan(des_yaw)) {
        des_yaw = cur_yaw;
    }
    if (delta_x < 0) {
        des_yaw = M_PI + des_yaw;
    }     
    ROS_INFO("desired yaw: %.2f", des_yaw * 180 / M_PI); 
}

void TrajectoryPlanner::height_callback(const std_msgs::Float64& ehmsg)
{
    expected_height = ehmsg.data;
}