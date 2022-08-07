#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <vector>

class InterpolateImu{
public:
    
    InterpolateImu() : nh_("~") {
        InitPubSub();
    }

    void InitPubSub() {
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
        gyro_subscriber_ = nh_.subscribe("gyro_topic", 10, &InterpolateImu::GyroCallback, this);        
        accel_subscriber_ = nh_.subscribe("accel_topic", 10, &InterpolateImu::AccelCallback, this);        
    }
private:

    sensor_msgs::Imu RDF2FLU(const sensor_msgs::Imu& imu_msg) {
        sensor_msgs::Imu flu_imu = imu_msg;
        flu_imu.linear_acceleration.x = imu_msg.linear_acceleration.z;
        flu_imu.linear_acceleration.y = -imu_msg.linear_acceleration.x;
        flu_imu.linear_acceleration.z = -imu_msg.linear_acceleration.y;
        flu_imu.angular_velocity.x = imu_msg.angular_velocity.z;
        flu_imu.angular_velocity.y = -imu_msg.angular_velocity.x;
        flu_imu.angular_velocity.z = -imu_msg.angular_velocity.y;
        return flu_imu;
    }

    void GyroCallback(const sensor_msgs::Imu& gyro_msg) {
        if (!last_accel_valid_) {
            return;
        }

        gyro_buffer_.push_back(gyro_msg);
    } 
    
    void AccelCallback(const sensor_msgs::Imu& accel_msg) {
        for (auto const& gyro_msg : gyro_buffer_) {
            sensor_msgs::Imu imu_msg;

            imu_msg = gyro_msg;
            if(InterpolateAccelMessage(last_accel_,
                                       accel_msg,
                                       gyro_msg.header.stamp,
                                       &imu_msg.linear_acceleration)) {
                imu_publisher_.publish(RDF2FLU(imu_msg));
            } else {
                ROS_WARN("[InterpolateImu::AccelCallback] Can't interpolate accel timestamp");
            }
            
        }
        gyro_buffer_.clear();
        last_accel_ = accel_msg;
        last_accel_valid_ = true;
    } 

    bool InterpolateAccelMessage(const sensor_msgs::Imu& accel1_msg, 
                                 const sensor_msgs::Imu& accel2_msg,
                                 const ros::Time& interp_time,
                                 geometry_msgs::Vector3* accel) {
        double t1 = accel1_msg.header.stamp.toSec();
        double t2 = accel2_msg.header.stamp.toSec();
        double t = interp_time.toSec();

        geometry_msgs::Vector3 accel1 = accel1_msg.linear_acceleration;
        geometry_msgs::Vector3 accel2 = accel2_msg.linear_acceleration;

        double interp_factor = (t - t1) / (t2 - t1);
        const double slop = 0.5;
        if(interp_factor < -slop || interp_factor > (1 + slop)) {
            ROS_WARN("t1: %0.2f, t2: %0.2f, t: %0.2f", t1, t2, t);
            return false;
        }

        accel->x = accel1.x + interp_factor * (accel2.x - accel1.x);
        accel->y = accel1.y + interp_factor * (accel2.y - accel1.y);
        accel->z = accel1.z + interp_factor * (accel2.z - accel1.z);
        return true;
    }

    std::vector<sensor_msgs::Imu> gyro_buffer_;
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Subscriber gyro_subscriber_;
    ros::Subscriber accel_subscriber_;
    sensor_msgs::Imu last_accel_;
    bool last_accel_valid_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat_to_rpy");
    InterpolateImu interpolate_imu;
    ros::spin();
    return 0;
}
