#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Initializes the Costmap with the parameters that we get from the params.yaml
    void initCostmap(
      double resolution, 
      int width, 
      int height, 
      geometry_msgs::msg::Pose origin, 
      double inflation_radius
      );

    // Update the costmap based on the current laserscan reading
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;

    // Inflate the obstacle in the laserscan on the costmap because we want of range of values
    // where we can and cannot go
    void inflateObstacle(int origin_x, int origin_y) const;

    // Retrieves costmap data
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

  private:
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    rclcpp::Logger logger_;

    double inflation_radius_;
    int inflation_cells_;

};

}  

#endif  
