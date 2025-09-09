#ifndef FOO_NODE_HPP_
#define FOO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "foo_core.hpp"

class FooNode : public rclcpp::Node {
public:
  /*
   * Foo node constructor.
   */
  FooNode();

private:
};

#endif