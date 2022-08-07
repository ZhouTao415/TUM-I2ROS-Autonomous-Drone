// laserscan_to_pointcloud.cpp

#include <cmath>
#include <string>
#include <unistd.h>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <fla_msgs/ProcessStatus.h>

class LaserScanToPointCloud 
{
private:
  laser_geometry::LaserProjection laser_projection_;
  int channel_option_;

  sensor_msgs::LaserScan scan_;
  sensor_msgs::PointCloud2 cloud_;
  ros::Time last_scan_stamp_;

  // Parameters
  bool        use_negative_info_;
  int         fla_process_id_;

  // ROS interfaces
  ros::NodeHandle nh_;
  ros::Subscriber sub_scan_;
  ros::Publisher  pub_cloud2_;
  ros::Publisher  pub_status_;
  ros::Timer      heartbeat_timer_;

  void scan_handler(const sensor_msgs::LaserScan::ConstPtr& scan);
  void heartbeat_callback(const ros::TimerEvent&) const;
  std::size_t set_null_ranges_max(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

public:
  LaserScanToPointCloud();
};

LaserScanToPointCloud::LaserScanToPointCloud()
{
  nh_ = ros::NodeHandle("~");
  ROS_INFO("[LaserscanToPointcloud] Started.");

  // get required parameters
  bool params_received = true;
  params_received &= nh_.getParam("use_negative_info", use_negative_info_);
  params_received &= nh_.getParam("fla_process_id", fla_process_id_);

  if (!params_received)
  {
    ROS_ERROR("[LaserscanToPointcloud] Missing required parameters. Exiting.");
    ros::shutdown();
    return;
  }

  channel_option_ = laser_geometry::channel_option::Default;
  last_scan_stamp_ = ros::Time(0);

  // start interfaces
  pub_cloud2_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud2_out", 1);
  pub_status_ = nh_.advertise<fla_msgs::ProcessStatus>("/globalstatus", 1);
  sub_scan_   = nh_.subscribe("scan_in", 1, &LaserScanToPointCloud::scan_handler, this);
  heartbeat_timer_ = nh_.createTimer(ros::Duration(0.05), &LaserScanToPointCloud::heartbeat_callback, this);
  return;
}

void LaserScanToPointCloud::scan_handler(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  last_scan_stamp_ = scan->header.stamp;

  if (use_negative_info_)
  {
    std::size_t num_rays_maxed = set_null_ranges_max(*scan, scan_);
    laser_projection_.projectLaser(scan_, cloud_, -1.0, channel_option_);
  }
  else
  {
    laser_projection_.projectLaser(*scan, cloud_, -1.0, channel_option_);
  }
  
  pub_cloud2_.publish(cloud_);
  return;
}

void LaserScanToPointCloud::heartbeat_callback(const ros::TimerEvent&) const
{
  fla_msgs::ProcessStatus status;

  status.pid = getpid();

  ros::Time t_now = ros::Time::now();

  if ( last_scan_stamp_ == ros::Time(0) )
  {
    status.status = fla_msgs::ProcessStatus::INIT;
  }
  else if ( t_now - last_scan_stamp_ > ros::Duration(1.0) )
  {
    status.status = fla_msgs::ProcessStatus::ALARM;
  }
  else
  {
    status.status = fla_msgs::ProcessStatus::READY;
  }

  pub_status_.publish(status);

  return;
}

std::size_t LaserScanToPointCloud::set_null_ranges_max(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
  std::size_t num_rays_maxed = 0;

  // Copy over relevant data
  scan_out.header           = scan_in.header;
  scan_out.angle_min        = scan_in.angle_min;
  scan_out.angle_max        = scan_in.angle_max;
  scan_out.angle_increment  = scan_in.angle_increment;
  scan_out.time_increment   = scan_in.time_increment;
  scan_out.scan_time        = scan_in.scan_time;
  scan_out.range_min        = scan_in.range_min;
  scan_out.range_max        = scan_in.range_max;
  scan_out.intensities      = scan_in.intensities;

  scan_out.ranges.clear();
  scan_out.ranges.reserve(scan_in.ranges.size());

  // We need the maximum range value that is still LESS than max range
  float max_range = nextafterf(scan_in.range_max, 0.0);

  for (std::size_t i=0; i < scan_in.ranges.size(); ++i)
  {
    if (scan_out.ranges[i] < scan_in.range_min)
    {
      scan_out.ranges.push_back( max_range );
      ++num_rays_maxed;
    }
    else
    {
      scan_out.ranges.push_back( scan_in.ranges[i] );
    }
  }

  return num_rays_maxed;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_to_pointcloud");

  LaserScanToPointCloud converter;
  ros::spin();

  return 0;
}