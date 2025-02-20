#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "costmap_core.hpp"

/**
Builds cost map of robot space base on point cloud input
 */
class CostmapNode : public rclcpp::Node
{
public:
  /**
   * Costmap node constructor.
   */
  CostmapNode();

private:
  // Object containing costmap and methods.
  controller::CostmapCore costmap_;
};

#endif  // COSTMAP_NODE_HPP_
