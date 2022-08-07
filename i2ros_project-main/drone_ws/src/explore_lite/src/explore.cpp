/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore(int num_drones, char * names[])
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , last_markers_count_(0)
{
  ROS_INFO("Initializing explore.");
  num_drones_ = num_drones;
  for(int i=0;i<num_drones;i++){
    std::string name_drone = std::string(names[i]);
    name_drones_.push_back(name_drone);
    // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client(name_drone + "/move_base");
    move_base_clients_.push_back(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(name_drone + "/move_base"));
    geometry_msgs::Point initial_position = costmap_client_.getRobotPose(name_drone).position;
    possessed_goals_.push_back(initial_position);
    prev_goals_.push_back(initial_position);
    last_progresses_.push_back(ros::Time::now());
    prev_distances_.push_back(0);
    launch_clients_.push_back(relative_nh_.serviceClient<service_pkg::stop_service>(name_drone + "/state_machine/stop_service"));
  }
  
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("x_min", x_min_, std::numeric_limits<double>::lowest());
  private_nh_.param("x_max", x_max_, std::numeric_limits<double>::max());
  private_nh_.param("y_min", y_min_, std::numeric_limits<double>::lowest());
  private_nh_.param("y_max", y_max_, std::numeric_limits<double>::max());
  private_nh_.param("safety_radius", safety_radius_, 0.0);
  private_nh_.param("goal_always_on_frontier", goal_always_on_frontier_, false);
  private_nh_.param("autostart", autostart_, true);

  search_ = frontier_exploration::FrontierSearch(private_nh_, costmap_client_.getCostmap(),
                                                potential_scale_, gain_scale_,
                                                min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  for(int i=0;i<num_drones;i++){
    move_base_clients_[i]->waitForServer();
  }
  ROS_INFO("Connected to move_base server");


  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); }, false, false);

  if (autostart_){
    start();
  }

  switch_server_ = private_nh_.advertiseService("switch_explore",&Explore::switch_server, this);
  ROS_INFO("Waiting for launch client.");
  for(int i=0;i<num_drones;i++){
    launch_clients_[i].waitForExistence();
  }
  ROS_INFO("Get launch client.");
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    geometry_msgs::Point p = goal_always_on_frontier_ ? frontier.middle : frontier.centroid;
    if (goalOnBlacklist(p)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  ROS_INFO("making plan.");
  int num_alive = 0;
  for(int i=0; i<num_drones_; i++){
    if(makePlanSingle(i)){
      num_alive++;
    }
  }
  if(!num_alive) stop();
}

bool Explore::makePlanSingle(int drone_idx)
{
  ROS_INFO("making plan for drone %d", drone_idx);
  // find frontiers
  auto pose = costmap_client_.getRobotPose(name_drones_[drone_idx]);
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop(drone_idx);
    return false;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                        [this, drone_idx](const frontier_exploration::Frontier& f) {
                          geometry_msgs::Point p = goal_always_on_frontier_ ? f.middle : f.centroid;
                          int goal_idx = goalPossessed(p);
                          return goalOnBlacklist(p) || (goal_idx!=drone_idx && goal_idx!=-1) || goalOutOfRange(p);
                      });
  if (frontier == frontiers.end()) {
    stop(drone_idx);
    return false;
  }
  geometry_msgs::Point target_position = goal_always_on_frontier_ ? frontier->middle : frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = prev_goals_[drone_idx] == target_position;
  prev_goals_[drone_idx] = target_position;
  if (!same_goal || prev_distances_[drone_idx] > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progresses_[drone_idx] = ros::Time::now();
    prev_distances_[drone_idx] = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progresses_[drone_idx] > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    return makePlanSingle(drone_idx);
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return true;
  }

  // assign possessed goal
  ROS_INFO("assign possessed goal.");
  possessed_goals_[drone_idx] = target_position;

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_clients_[drone_idx]->sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
  return true;
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

bool Explore::goalOutOfRange(const geometry_msgs::Point& goal)
{
  if (goal.x > x_min_+goal_always_on_frontier_ && goal.x < x_max_-goal_always_on_frontier_ && goal.y > y_min_+goal_always_on_frontier_ && goal.y < y_max_-goal_always_on_frontier_){
    return false;
  }

  return true;
}

int Explore::goalPossessed(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the possessed list
  for(int i=0; i<num_drones_; i++){
    auto& frontier_goal = possessed_goals_[i];
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return i;

  }
  return -1;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

bool Explore::start()
{
  exploring_timer_.start();
  return true;
}

bool Explore::stop()
{
  for(int i=0; i<num_drones_; i++){
    stop(i);
  }
  service_pkg::stop_service stop;
  for(int i=0;i<num_drones_;i++){
    launch_clients_[i].call(stop);
  }
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
  return true;
}

bool Explore::stop(int drone_idx){
  move_base_clients_[drone_idx]->cancelAllGoals();
  return true;
}

bool Explore::switch_server(service_pkg::switch_explore::Request& req,
          service_pkg::switch_explore::Response& resp){

    bool success;
    if (req.switch_on)
    {
      success = start();
    }else{
      success = stop();
    }

    resp.success = success;
    return success;
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore(argc-1, argv+1);
  ros::spin();

  return 0;
}
