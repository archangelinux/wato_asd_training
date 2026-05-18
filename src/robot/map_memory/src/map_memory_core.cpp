#include "map_memory_core.hpp"
#include <algorithm>

namespace robot
{
MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}

void MapMemoryCore::configure(const MapParams& params) {
  params_ = params;
  //fills the entire occupancy grid with -1 to represent unknown in ROS
  global_map_data_.assign(params_.width * params_.height, -1); // -1 = unknown
}

// transforms each costmap cell from robot-local frame into world frame then writes it into the global map using linear fusion
void MapMemoryCore::integrateCostmap(
  const nav_msgs::msg::OccupancyGrid& costmap,
  double robot_x, double robot_y, double robot_yaw)
{
  //precomputed
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);
  double cm_res = costmap.info.resolution;
  double cm_ox  = costmap.info.origin.position.x;
  double cm_oy  = costmap.info.origin.position.y;
  int cm_w   = static_cast<int>(costmap.info.width);
  int cm_h   = static_cast<int>(costmap.info.height);

  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      int8_t cost = costmap.data[cy * cm_w + cx];
      //linear fusion logic
      if (cost <= 0) continue; // unknown/free cell, keep existing global value

      // costmap cell center (hence the +0.5) in robot-local frame
      //converts grid index (cx, cy) back into a position in the robotis local frame
      double local_x = cm_ox + (cx + 0.5) * cm_res;
      double local_y = cm_oy + (cy + 0.5) * cm_res;

      // 2d rotation + translation into world frame
      double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      // world position --> global map grid index
      int mx = static_cast<int>(std::floor((world_x - params_.origin_x) / params_.resolution));
      int my = static_cast<int>(std::floor((world_y - params_.origin_y) / params_.resolution));

      if (inBounds(mx, my)) {
        global_map_data_[index(mx, my)] = cost;
      }
    }
  }
}

// packages the flat array into an occupancy grid message
nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "sim_world"; //matches gazebo world frame for foxglove to display in correct coord system
  map.info.resolution = params_.resolution;
  map.info.width      = params_.width;
  map.info.height     = params_.height;
  map.info.origin.position.x = params_.origin_x;
  map.info.origin.position.y = params_.origin_y;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data = global_map_data_;
  return map;
}

bool MapMemoryCore::inBounds(int x, int y) const {
  return x >= 0 && x < params_.width && y >= 0 && y < params_.height;
}

int MapMemoryCore::index(int x, int y) const {
  return y * params_.width + x;
}

}
