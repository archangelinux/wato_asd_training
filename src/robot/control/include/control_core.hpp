#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>
#include <cmath>

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    // finds the first waypoint on the path that is >= lookahead_distance from the robot
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path& path,
      double robot_x, double robot_y,
      double lookahead_distance);

    // computes linear and angular velocity using the pure pursuit formula
    geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::PoseStamped& lookahead,
      double robot_x, double robot_y, double robot_yaw,
      double linear_speed);

  private:
    rclcpp::Logger logger_;
};

}

#endif
