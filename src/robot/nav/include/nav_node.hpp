#ifndef NAV_NODE_HPP_
#define NAV_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_core.hpp"

class NavNode : public rclcpp::Node {
  public:
    NavNode();

  private:
    robot::NavCore nav_;
};

#endif
