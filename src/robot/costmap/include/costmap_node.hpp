#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    /**
    * Costmap node constructor.
    */
    CostmapNode();

  private:
    robot::CostmapCore costmap_;
};

#endif 
