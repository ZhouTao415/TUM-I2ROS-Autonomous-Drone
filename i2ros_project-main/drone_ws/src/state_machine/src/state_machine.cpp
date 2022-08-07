#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <service_pkg/stop_service.h>
#include <service_pkg/switch_explore.h>
class State_Machine{
    // Node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_relative_;

    //Timer controll the main node
    ros::Timer time_mission;

    // Mission Flag
    // 1- Initial state
    // 2- Launch state
    // 3- Follow path
    // 4- Landing
    // 5- Landed
    int mission_flag;

    // True when user push key space
    bool launch_flag;

    // True when quadrotor reaches expexted_height
    bool explore_flag;

    // For checking launching
    double current_height_;

    // True when server send the massage
    bool landing_flag;

    // For checking landed
    bool landed_flag;

    bool call_explore_service;
    bool success;
    // Specified in State Machine Launch file
    double expected_height_;
    double landed_height_;

    ros::Publisher state_publisher; 
    std_msgs::Int64 mission_flag_msgs;
    std_msgs::Float64 expected_height_msg;
    ros::Publisher expected_height_publisher;
    ros::Subscriber current_state_subscriber;
    ros::Subscriber launch_keyboard_subscriber;
    ros::ServiceClient switch_client;
    service_pkg::switch_explore srv;
    ros::ServiceServer land_server;


public:
    State_Machine(double expected_height = 4.0, double landed_height = 0.2):nh_("~")
    {   
        // set initial state flag
        mission_flag = 1;
        launch_flag = false;
        explore_flag = false;
        landing_flag = false;
        landed_flag = false;

        expected_height_ = expected_height;
        landed_height_ = landed_height;

        call_explore_service = true;
        success = false;
        // publish to controller
        state_publisher = nh_.advertise<std_msgs::Int64>("state", 10);
        expected_height_publisher = nh_.advertise<std_msgs::Float64>("expected_height",10);

        // subscribe the keyboard launch
        launch_keyboard_subscriber = nh_.subscribe("/launch_keyboard", 1, &State_Machine::launch_keyboard_callback, this);

        // for height checking
        current_state_subscriber = nh_relative_.subscribe("Estimation/current_state", 1, &State_Machine::current_height_check_callback, this);

        // subscribing the landing service from move base
        switch_client = nh_relative_.serviceClient<service_pkg::switch_explore>("/explore/switch_explore");
        
        // subscribing the landing service from move base
        land_server = nh_.advertiseService("stop_service", &State_Machine::stop_flag, this);
        
        // use timer to schedule events
        time_mission = nh_.createTimer(ros::Duration(0.1), &State_Machine::state_machine_loop, this);
    }

    void state_machine_loop(const ros::TimerEvent& t){
        if(mission_flag == 1){initial_state();}
        if(mission_flag == 2){launch_state();}
        if(mission_flag == 3){explore();}
        if(mission_flag == 4){landing();}
        if(mission_flag == 5){landed();}
    }

    void initial_state()
    {
        mission_flag_msgs.data = 1;
        state_publisher.publish(mission_flag_msgs);

        if (launch_flag)
        {
            mission_flag = 2;
        }
    }

    void launch_state()
    {
        mission_flag_msgs.data = 2;
        state_publisher.publish(mission_flag_msgs);

        expected_height_msg.data = expected_height_;
        expected_height_publisher.publish(expected_height_msg);

        
        if (explore_flag)
        {
            mission_flag = 3;
        }
    }

    void explore()
    {
        // expected_height_publisher = nh_.advertise<std_msgs::Float64>("/expected_height",10);

        mission_flag_msgs.data = 3;
        state_publisher.publish(mission_flag_msgs);

        expected_height_msg.data = expected_height_;
        expected_height_publisher.publish(expected_height_msg);

        // land_server = nh_.advertiseService("stop_service", &State_Machine::stop_flag, this);
        if (landing_flag)
        {
            mission_flag = 4;
        }
    }

    void landing()
    {
        mission_flag_msgs.data = 4;
        state_publisher.publish(mission_flag_msgs);
        ROS_INFO_STREAM_ONCE("Explore end, landing");
        if (landed_flag)
        {
            mission_flag = 5;
        }
    }

    void landed()
    {
        mission_flag_msgs.data = 5;
        state_publisher.publish(mission_flag_msgs);
    }

    bool stop_flag(service_pkg::stop_service::Request &req, service_pkg::stop_service::Response &res)
    {
        
        landing_flag = true;
        return true;
    }
    void launch_keyboard_callback(const std_msgs::String& launch_msg)
    {
        if (launch_msg.data == "R")
        {
            launch_flag = true;
        }

        // if (launch_msg.data == "Land")
        // {
        //     land_server = nh_.advertiseService("stop_service", &State_Machine::stop_flag, this);
        // }
    }


    void current_height_check_callback(const nav_msgs::Odometry& cur_state)
    {
        current_height_ = cur_state.pose.pose.position.z;

        if ((current_height_ - expected_height_ < 0.3) && (current_height_ - expected_height_ > -0.3))
        {
            ROS_INFO_STREAM_ONCE("Quadrotor has reached expected height, ready for exploring");
            if (call_explore_service)
            {
                srv.request.switch_on = true;
                // switch_client.call(srv);
                success = switch_client.call(srv);
                if (success)
                {
                    ROS_INFO("call success");
                    call_explore_service = false;
                }
                else
                {
                    ROS_INFO("Try to call again");
                }
            }
            // if (success)
            //     {
            //         ROS_INFO("call success");
            //     }
            // else
            //     {
            //         ROS_INFO("Try to call again");
            //     }
            explore_flag = true;
        }

        if ((current_height_ - landed_height_ < 0.3) && (current_height_ - landed_height_ > -0.3) && (mission_flag == 4))
        {
            ROS_INFO_STREAM_ONCE("Quadrotor has reached landed height, landed");
            landed_flag = true;
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh_private("~");

    // char *expected_height_char = argv[0];
    // char *landed_height_char = argv[1];

    double expected_height;
    double landed_height;
    bool rotation_first;
    bool rotation_while_explore;
    nh_private.param<double>("expected_height", expected_height, 4.0);
    nh_private.param<double>("landed_height", landed_height, 0.2);
    State_Machine state_machine(expected_height, landed_height);

    ros::spin();
}
