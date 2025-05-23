#include "can_node.hpp"

CanNode::CanNode() : Node("can_node") {
  RCLCPP_INFO(this->get_logger(), "CAN Node has been initialized");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
