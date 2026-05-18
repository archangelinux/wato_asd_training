#include <cmath>
#include "map_memory_node.hpp"

// constructor declares and reads params
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  this->declare_parameter<double>("distance_threshold", 1.5);
  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 1000);
  this->declare_parameter<int>("height", 1000);
  this->declare_parameter<double>("origin_x", -50.0);
  this->declare_parameter<double>("origin_y", -50.0);

  distance_threshold_ = this->get_parameter("distance_threshold").as_double();

  // set up the grid
  map_memory_.configure({
    this->get_parameter("resolution").as_double(),
    this->get_parameter("width").as_int(),
    this->get_parameter("height").as_int(),
    this->get_parameter("origin_x").as_double(),
    this->get_parameter("origin_y").as_double()
  });

  //2 subscribers, one publisher and one timer
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // publish empty map immediately so the planner can start without waiting
  auto initial_map = map_memory_.getGlobalMap();
  initial_map.header.stamp = this->get_clock()->now();
  map_pub_->publish(initial_map);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

// buffers the latest costmap message
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

// buffers the latest odom message
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // extract yaw(rotation around vertical axis) from quaternion (x, y, z, w)
  auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  // sets the flag to update the map if the robot has moved enough (>=1.5 from last integration point)
  // the last_update_x_/y_ is for updateMap where the actual integration happens
  if (std::sqrt(dx * dx + dy * dy) >= distance_threshold_) {
    should_update_map_ = true;
  }
}

// fires every second, checks if the robot has moved enough + costmap exists, to update the map
void MapMemoryNode::updateMap() {
  // aggregate most recent costmap into global map
  if (costmap_received_ && should_update_map_) {
    map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    should_update_map_ = false;
  }

  // always publish current map (even if unchanged)
  auto map = map_memory_.getGlobalMap();
  map.header.stamp = this->get_clock()->now(); // planner gets fresh timestamp each second/tick even if map content is unchanged
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
