#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
{

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
