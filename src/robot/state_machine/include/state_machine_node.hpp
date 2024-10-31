#ifndef STATE_MACHINE_NODE_HPP_
#define STATE_MACHINE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "state_machine_core.hpp"

class StateMachineNode : public rclcpp::Node {
  public:
    StateMachineNode();

  private:
    robot::StateMachineCore state_machine_;
};

#endif 
