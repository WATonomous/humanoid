#ifndef CAN_NODE_HPP
#define CAN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "can_core.hpp"

class CanNode : public rclcpp::Node {
public:
  CanNode();

private:
  autonomy::CanCore can_;
};

#endif // CAN_NODE_HPP
