#ifndef MAP_SERVER_NODE_HPP_
#define MAP_SERVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_server_core.hpp"

class MapServerNode : public rclcpp::Node {
  public:
    MapServerNode();

  private:
    robot::MapServerCore map_server_;
};

#endif 
