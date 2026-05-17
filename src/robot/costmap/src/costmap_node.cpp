#include <memory>
#include <cmath>

#include "costmap_node.hpp"

// constructor declares all params to be overriden from params.yaml
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<std::string>("lidar_topic", "/lidar");
  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 200);
  this->declare_parameter<int>("height", 200);
  this->declare_parameter<double>("origin_x", -10.0);
  this->declare_parameter<double>("origin_y", -10.0);
  this->declare_parameter<double>("inflation_radius", 1.0);
  this->declare_parameter<int>("max_cost", 100);

  std::string frame_id, lidar_topic;
  double resolution, origin_x, origin_y, inflation_radius;
  int width, height, max_cost;

  this->get_parameter("frame_id", frame_id);
  this->get_parameter("lidar_topic", lidar_topic);
  this->get_parameter("resolution", resolution);
  this->get_parameter("width", width);
  this->get_parameter("height", height);
  this->get_parameter("origin_x", origin_x);
  this->get_parameter("origin_y", origin_y);
  this->get_parameter("inflation_radius", inflation_radius);
  this->get_parameter("max_cost", max_cost);

  // send values to core logic 
  costmap_.configure(robot::CostmapCore::CostmapParams{frame_id, resolution, width, height, origin_x, origin_y, inflation_radius, max_cost});

  // create the /costmap publisher 
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  //creates the /lidar subscriber 
  auto qos = rclcpp::SensorDataQoS();
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic, qos, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

// // Warmup test publisher (unused):
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }

// fires every time a laser scan arrives
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  //reset -> compute -> inflate -> build grid -> publish
  costmap_.resetGrid();
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
    double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    int gx, gy;
    if (costmap_.polarToGrid(r, angle, gx, gy)) costmap_.markAsObstacle(gx, gy);
  }
  costmap_.inflateObstacles();
  nav_msgs::msg::OccupancyGrid grid_msg;
  costmap_.buildGrid(grid_msg);
  grid_msg.header.stamp = this->get_clock()->now();
  costmap_pub_->publish(grid_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}