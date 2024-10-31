#include <chrono>
#include <memory>

#include "map_server_node.hpp"

MapServerNode::MapServerNode() : Node("map_server"), map_server_(robot::MapServerCore()) {

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapServerNode>());
  rclcpp::shutdown();
  return 0;
}
