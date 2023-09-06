#include <memory>

#include "nav_node.hpp"

NavNode::NavNode(): Node("transformer"), nav_(robot::NavCore())
{
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}
