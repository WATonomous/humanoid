#include <chrono>
#include <memory>

#include "occupancy_node.hpp"

OccupancyNode::OccupancyNode() : Node("occupancy"), occupancy_(robot::OccupancyCore()) {

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
