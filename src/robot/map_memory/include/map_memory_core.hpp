#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    struct MapParams {
      double resolution; // meters per cell
      int width;
      int height;
      double origin_x;
      double origin_y;
    };

     // public methods to setup, update and read , called by the node
    void configure(const MapParams& params);
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw);
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

  private:
    rclcpp::Logger logger_;
    MapParams params_;
    std::vector<int8_t> global_map_data_; //this is the persistent memory of the map that persists through scans

    inline bool inBounds(int x, int y) const; // checks whether a grid index actually falls within the global map before writing to it
    inline int index(int x, int y) const; // converts 2D grid coordinates to a 1D index for the vector
};

}

#endif
