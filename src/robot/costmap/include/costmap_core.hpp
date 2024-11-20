#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void init_costmap(
      double resolution, 
      int width, 
      int height, 
      geometry_msgs::msg::Pose origin, 
      double inflation_radius
      );
    void update_costmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;
    void inflate_obstacle(int origin_x, int origin_y) const;

    nav_msgs::msg::OccupancyGrid::SharedPtr get_costmap_data() const;

  private:
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    rclcpp::Logger logger_;

    double inflation_radius_;
    int inflation_cells_;

};

}  

#endif  
