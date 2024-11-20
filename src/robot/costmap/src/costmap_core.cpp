#include <algorithm>
#include <queue>

#include "costmap_core.hpp"

namespace robot
{
  CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

  void CostmapCore::init_costmap(double resolution, int width, int height, 
      geometry_msgs::msg::Pose origin, double inflation_radius) {
    costmap_data_->info.resolution = resolution;
    costmap_data_->info.width = width;
    costmap_data_->info.height = height;
    costmap_data_->info.origin = origin;
    costmap_data_->data.assign(width * height, -1);

    inflation_radius_ = inflation_radius;
    inflation_cells_ = static_cast<int>(inflation_radius / resolution);

    RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
  }

  void CostmapCore::update_costmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const {
    // Reset the costmap to free space
    std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

    double angle = laserscan->angle_min;
    for (size_t i = 0; i < laserscan->ranges.size(); ++i, angle += laserscan->angle_increment) {
      double range = laserscan->ranges[i];

      // Check if the range is within the valid range
      if (range >= laserscan->range_min && range <= laserscan->range_max) {
        // Calculate obstacle position in the map frame
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        // Convert to grid coordinates
        int grid_x = static_cast<int>((x - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution);
        int grid_y = static_cast<int>((y - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution);

        if (grid_x >= 0 && grid_x < static_cast<int>(costmap_data_->info.width) &&
          grid_y >= 0 && grid_y < static_cast<int>(costmap_data_->info.height)) {
          // Mark the cell as occupied
          int index = grid_y * costmap_data_->info.width + grid_x;
          costmap_data_->data[index] = 100;  // 100 indicates an occupied cell

          // Inflate around the obstacle
          inflate_obstacle(grid_x, grid_y);
        }
      }
    }
  }

  void CostmapCore::inflate_obstacle(int origin_x, int origin_y) const {
    // Use a simple breadth-first search (BFS) to mark cells within the inflation radius
    std::queue<std::pair<int, int>> queue;
    queue.emplace(origin_x, origin_y);

    std::vector<std::vector<bool>> visited(costmap_data_->info.width, std::vector<bool>(costmap_data_->info.height, false));
    visited[origin_x][origin_y] = true;

    while (!queue.empty()) {
      auto [x, y] = queue.front();
      queue.pop();

      // Iterate over neighboring cells
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;  // Skip the center cell

          int nx = x + dx;
          int ny = y + dy;

          // Ensure the neighbor cell is within bounds
          if (nx >= 0 && nx < static_cast<int>(costmap_data_->info.width) &&
            ny >= 0 && ny < static_cast<int>(costmap_data_->info.height) &&
            !visited[nx][ny]) {
            // Calculate the distance to the original obstacle cell
            double distance = std::hypot(nx - origin_x, ny - origin_y) * costmap_data_->info.resolution;

            // If within inflation radius, mark as inflated and add to queue
            if (distance <= inflation_radius_) {
                int index = ny * costmap_data_->info.width + nx;
                if (costmap_data_->data[index] != 100) {
                  costmap_data_->data[index] = 10;  // 50 indicates an inflated cell
                }
                queue.emplace(nx, ny);
            }

            visited[nx][ny] = true;
          }
        }
      }
    }
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::get_costmap_data() const {
    return costmap_data_;
  }
} 
