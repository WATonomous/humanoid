#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    // Costmap Node constructor
    CostmapNode();
    
    // Retrieves all the parameters and their values in params.yaml
    void processParameters();
    
    // Given a laserscan, it will send it into CostmapCore for processing and then
    // retrieve the costmap
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

  private:
    robot::CostmapCore costmap_;
};

#endif 
