#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define KEYCODE_SPACE 0x20
#define KEYCODE_D 0x64

int keyboard_command = 0x00;
void getch()
{
    struct termios new_settings;
	struct termios stored_settings;
    // set the terminal parameter
	tcgetattr(0,&stored_settings);

    // bkeep the output in the terminal (for tracking)
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0,&stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&new_settings);
	keyboard_command = getchar();
	tcsetattr(0,TCSANOW,&stored_settings);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "launcher_keyboard");
    ros::NodeHandle n;
    ros::Publisher launch_publisher = n.advertise<std_msgs::String>("/launch_keyboard", 10);

    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        getch();
        if (keyboard_command == KEYCODE_SPACE)
        {
            std_msgs::String msg_launch;
            std::stringstream ss1;
            ss1 << "R";
            msg_launch.data = ss1.str();
            launch_publisher.publish(msg_launch);

            ros::spinOnce();

            loop_rate.sleep();
        }

        // if (keyboard_command == KEYCODE_D)
        // {
        //     std_msgs::String msg_land;
        //     std::stringstream ss2;
        //     ss2 << "Land";
        //     msg_land.data = ss2.str();
        //     launch_publisher.publish(msg_land);

        //     ros::spinOnce();

        //     loop_rate.sleep();
        // }
    }
    return 0;
}
