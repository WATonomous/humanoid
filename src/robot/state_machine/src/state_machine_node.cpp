#include <chrono>
#include <memory>

#include "state_machine_node.hpp"

StateMachineNode::StateMachineNode() : Node("state_machine"), state_machine_(robot::StateMachineCore()) {

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
