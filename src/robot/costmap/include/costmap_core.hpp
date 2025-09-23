#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    struct CostmapParams {
      std::string frame_id;
      double resolution;
      int width;
      int height;
      double origin_x;
      double origin_y;
      double inflation_radius;
      int max_cost;
    };

    void configure(const CostmapParams& params);
    void resetGrid();
    bool polarToGrid(double range, double angle, int& grid_x, int& grid_y) const;
    void markAsObstacle(int grid_x, int grid_y);
    void inflateObstacles();
    void buildGrid(nav_msgs::msg::OccupancyGrid& output_grid) const;

  private:
    rclcpp::Logger logger_;
    CostmapParams params_;
    std::vector<uint8_t> grid_; // width*height, 0..100
    inline bool inBounds(int x, int y) const;
    inline int index(int x, int y) const;

};
}

#endif  