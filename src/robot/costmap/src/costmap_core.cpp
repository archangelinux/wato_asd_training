#include "costmap_core.hpp"
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

//store params and allocate grid size
void CostmapCore::configure(const CostmapParams& params) {
  params_ = params;
  grid_.resize(params_.width * params_.height, 0);
}

//set all cells to 0
void CostmapCore::resetGrid() {
  std::fill(grid_.begin(), grid_.end(), 0);
}

//convert polar coordinates to grid coordinates (x,y)
bool CostmapCore::polarToGrid(double range, double angle, int& grid_x, int& grid_y) const {
  // Compute world coordinates from polar (range, angle)
  const double world_x = range * std::cos(angle);
  const double world_y = range * std::sin(angle);

  // Convert world -> grid using origin (bottom-left) and resolution
  grid_x = static_cast<int>(std::floor((world_x - params_.origin_x) / params_.resolution));
  grid_y = static_cast<int>(std::floor((world_y - params_.origin_y) / params_.resolution));

  return inBounds(grid_x, grid_y);
}

//mark a cell as an obstacle, setting it to the max cost
void CostmapCore::markAsObstacle(int grid_x, int grid_y) {
  if (inBounds(grid_x, grid_y)) {
    grid_[index(grid_x, grid_y)] = params_.max_cost;
  }
}

//inflate obstacles by the inflation radius - with linear falloff (gradient that gradually decreases)
void CostmapCore::inflateObstacles() {
    //convert inflation radius from meters to grid cells
    int inflation_radius_cells = static_cast<int>(std::ceil(params_.inflation_radius / params_.resolution));
    //for each cell that is an obstacle
    for(int y = 0; y < params_.height; ++y) {
      for(int x = 0; x < params_.width; ++x) {
        if(grid_[index(x, y)] == params_.max_cost) { //if the cell is an obstacle (which we marked with max cost)
          //for each neighbor within inflation radius
          for(int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) { //for each neighbor within inflation radius
            for(int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) { //for each neighbor within inflation radius
              int new_x = x + dx; 
              int new_y = y + dy;
              if(inBounds(new_x, new_y)) { //if the neighbor is within bounds
                //calculate distance in meters
                double distance = std::sqrt(dx*dx + dy*dy) * params_.resolution;
                //only inflate if within inflation radius
                if(distance <= params_.inflation_radius) {
                  //calculate cost using linear falloff - creates a linear decrease from max_cost at distance 0 to 0 at distance inflation_radius
                  int new_cost = static_cast<int>(params_.max_cost * (1.0 - distance / params_.inflation_radius));
                  
                  //keep maximum cost in consideration of other obstacles that may be closer
                  int current_cost = grid_[index(new_x, new_y)];
                  grid_[index(new_x, new_y)] = std::max(current_cost, new_cost);
                }
              }
            }
          }
        }
      }
    }
  }

//build the grid and return it as an occupancy grid message
void CostmapCore::buildGrid(nav_msgs::msg::OccupancyGrid& output_grid) const {
  output_grid.header.frame_id = params_.frame_id;
  output_grid.header.stamp = rclcpp::Clock().now();
  output_grid.info.resolution = params_.resolution;
  output_grid.info.width = params_.width;
  output_grid.info.height = params_.height;
  output_grid.info.origin.position.x = params_.origin_x;
  output_grid.info.origin.position.y = params_.origin_y;
  output_grid.info.origin.position.z = 0.0;
  output_grid.info.origin.orientation.x = 0.0;
  output_grid.info.origin.orientation.y = 0.0;
  output_grid.info.origin.orientation.z = 0.0;
  output_grid.info.origin.orientation.w = 1.0;
  output_grid.data.assign(grid_.begin(), grid_.end());
}

bool CostmapCore::inBounds(int x, int y) const {
  return x >= 0 && x < params_.width && y >= 0 && y < params_.height;
}

int CostmapCore::index(int x, int y) const {
  return y * params_.width + x;
}

}