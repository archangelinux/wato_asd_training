#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

namespace robot
{

//2D grid index
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

// hash so CellIndex can be used in unordered_map/set
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score; // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// min-heap comparator by f_score
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score; // smallest f on top
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // runs A* on the occupancy grid and returns world-frame waypoints
    std::vector<geometry_msgs::msg::PoseStamped> planPath(
      const nav_msgs::msg::OccupancyGrid& map,
      double start_x, double start_y,
      double goal_x, double goal_y,
      int obstacle_threshold);

  private:
    rclcpp::Logger logger_;

    // converts world coords to grid cell index
    CellIndex worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid& map) const; 
    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) const;// converts grid cell index to world-frame PoseStamped
    bool inBounds(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) const; //checks if a cell is within map bounds
    double heuristic(const CellIndex& a, const CellIndex& b) const; //euclidean distance heuristic

};

}

#endif
