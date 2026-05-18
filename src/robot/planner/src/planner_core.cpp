#include "planner_core.hpp"
#include <algorithm>
#include <limits>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

//A* pathfinding from (start_x, start_y) to (goal_x, goal_y) on the occupancy grid
std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map,
  double start_x, double start_y,
  double goal_x, double goal_y,
  int obstacle_threshold)
{
  CellIndex start = worldToGrid(start_x, start_y, map);
  CellIndex goal  = worldToGrid(goal_x,  goal_y,  map);

  if (!inBounds(start, map) || !inBounds(goal, map)) {
    RCLCPP_WARN(logger_, "Start or goal is outside map bounds");
    return {};
  }

  if (start == goal) return {gridToWorld(start, map)};  //trivial case

  //open set: min-heap by f_score
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  //cost from start to each cell
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from; //for path reconstruction: which cell did we come from
  std::unordered_set<CellIndex, CellIndexHash> closed_set;   //already fully expanded cells


  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  //8-directional movement (cardinal + diagonal)
  const std::vector<std::pair<int,int>> directions = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
  };

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();
    //goal reached: reconstruct path from goal back to start
    if (current == goal) {
      std::vector<geometry_msgs::msg::PoseStamped> path;
      CellIndex c = goal;
      while (c != start) {
        path.push_back(gridToWorld(c, map));
        c = came_from[c];
      }
      path.push_back(gridToWorld(start, map));
      std::reverse(path.begin(), path.end());
      return path;
    }

    //skip stale entries from the lazy-deletion open set
    if (closed_set.count(current)) continue;
    closed_set.insert(current);

    for (auto [dx, dy] : directions) {
      CellIndex neighbor(current.x + dx, current.y + dy);

      if (!inBounds(neighbor, map)) continue;
      if (closed_set.count(neighbor)) continue;

      //treat unknown (-1) as passable, block cells above the obstacle threshold
      int8_t cell_cost = map.data[neighbor.y * map.info.width + neighbor.x];
      if (cell_cost >= obstacle_threshold) continue;

      //diagonal moves cost sqrt(2), straight moves cost 1
      double move_cost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
      double tentative_g = g_score[current] + move_cost;

      //only update if this is a cheaper path to the neighbor
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        open_set.push(AStarNode(neighbor, tentative_g + heuristic(neighbor, goal)));
      }
    }
  }

  RCLCPP_WARN(logger_, "A* found no path to goal");
  return {};
}

//world (x,y) --> grid cell (col, row)
CellIndex PlannerCore::worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid& map) const {
  int gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  int gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return CellIndex(gx, gy);
}

//grid cell center --> world-frame PoseStamped
geometry_msgs::msg::PoseStamped PlannerCore::gridToWorld(
  const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = map.header.frame_id;
  pose.pose.position.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  pose.pose.position.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool PlannerCore::inBounds(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) const {
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

//euclidean distance between two cells (used as the A* heuristic)
double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

}
