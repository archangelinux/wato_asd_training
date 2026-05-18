#include <cmath>
#include "control_node.hpp"

// constructor declares and reads params, sets up ROS constructs
ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("linear_speed", 0.5);
  this->declare_parameter<double>("goal_tolerance", 0.5); // stop when within this distance of path end

  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  linear_speed_       = this->get_parameter("linear_speed").as_double();
  goal_tolerance_     = this->get_parameter("goal_tolerance").as_double();

  // 2 subscribers, 1 publisher, 1 timer
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // runs at 10Hz for smooth velocity output
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

// buffers the latest path from the planner
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  path_received_ = true;
}

// buffers robot position and extracts yaw from quaternion
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // extract yaw (rotation around vertical axis) from quaternion (x, y, z, w)
  auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// pure pursuit control loop: find lookahead point -> compute velocity -> publish
void ControlNode::controlLoop() {
  if (!path_received_ || current_path_.poses.empty()) return;

  // stop if the robot is within goal_tolerance of the last waypoint (end of path)
  auto& last = current_path_.poses.back();
  double dx = last.pose.position.x - robot_x_;
  double dy = last.pose.position.y - robot_y_;
  if (std::sqrt(dx*dx + dy*dy) < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // publish zero velocity to stop
    return;
  }

  auto lookahead = control_.findLookaheadPoint(current_path_, robot_x_, robot_y_, lookahead_distance_);
  if (!lookahead) return;

  auto cmd = control_.computeVelocity(*lookahead, robot_x_, robot_y_, robot_yaw_, linear_speed_);
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
