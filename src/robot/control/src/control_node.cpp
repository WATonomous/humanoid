#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  //Deliverable 5.1
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&ControlNode::timer_callback, this));  
}

void ControlNode::timer_callback()
{
  //Deliverable 5.1
  RCLCPP_INFO(this->get_logger(), "Timer Message");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
