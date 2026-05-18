#include <cmath>
#include "planner_node.hpp"

//constructor declares and reads params
PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  this->declare_parameter<double>("goal_tolerance", 0.5);
  this->declare_parameter<int>("obstacle_threshold", 50); // cells >= this cost are treated as obstacles

  goal_tolerance_     = this->get_parameter("goal_tolerance").as_double();
  obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();

  //3 subscribers, 1 publisher, 1 timer
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  //fires every 500ms to check if the goal has been reached or a replan is needed
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// buffers the latest map and replans immediately if we're currently navigating
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath(); // new map data may reveal a better or now-valid path
  }
}

// new goal received: store it, transition state, and plan immediately
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

//buffers robot pose for goal-reached checks and A* start position
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

//fires every 500ms: check if goal reached, otherwise replan
void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      planPath();
    }
  }
}

//returns true when the robot is within goal_tolerance of the goal point
bool PlannerNode::goalReached() const {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx*dx + dy*dy) < goal_tolerance_;
}

//calls A* and publishes the resulting path
void PlannerNode::planPath() {
  if (!map_received_ || !goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or goal");
    return;
  }

  auto poses = planner_.planPath(
    current_map_,
    robot_pose_.position.x, robot_pose_.position.y,
    goal_.point.x, goal_.point.y,
    obstacle_threshold_);

  nav_msgs::msg::Path path;
  path.header.stamp    = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id;
  path.poses = poses;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
