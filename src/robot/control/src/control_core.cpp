#include "control_core.hpp"
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

// finds the lookahead point: closest waypoint to robot first, then first point
// at or beyond lookahead_distance ahead of it (prevents doubling back)
std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path,
  double robot_x, double robot_y,
  double lookahead_distance)
{
  if (path.poses.empty()) return std::nullopt;

  // find the closest waypoint on the path to avoid re-tracking passed points
  int closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
    double dx = path.poses[i].pose.position.x - robot_x;
    double dy = path.poses[i].pose.position.y - robot_y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // search forward from the closest point for the first waypoint >= lookahead_distance away
  for (int i = closest_idx; i < static_cast<int>(path.poses.size()); ++i) {
    double dx = path.poses[i].pose.position.x - robot_x;
    double dy = path.poses[i].pose.position.y - robot_y;
    if (std::sqrt(dx*dx + dy*dy) >= lookahead_distance) {
      return path.poses[i];
    }
  }

  // if no point is far enough ahead, use the last waypoint (robot is near end of path)
  return path.poses.back();
}

// pure pursuit: transforms the lookahead point into the robot frame, then computes
// curvature kappa = 2 * y_local / L^2 and sets angular velocity = linear_speed * kappa
geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped& lookahead,
  double robot_x, double robot_y, double robot_yaw,
  double linear_speed)
{
  double dx = lookahead.pose.position.x - robot_x;
  double dy = lookahead.pose.position.y - robot_y;

  // rotate world-frame offset into robot frame
  double local_x =  dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
  double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

  // pure pursuit curvature: kappa = 2 * lateral_error / L^2
  double L_squared = local_x * local_x + local_y * local_y;
  double curvature = (L_squared > 1e-6) ? (2.0 * local_y / L_squared) : 0.0;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = linear_speed;
  cmd.angular.z = linear_speed * curvature; // omega = v * kappa
  return cmd;
}

}
