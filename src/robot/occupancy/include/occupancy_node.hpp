#ifndef OCCUPANCY_NODE_HPP_
#define OCCUPANCY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "occupancy_core.hpp"

class OccupancyNode : public rclcpp::Node {
  public:
    /**
    * Occupancy node constructor.
    */
    OccupancyNode();

  private:
    robot::OccupancyCore occupancy_;
};

#endif 
