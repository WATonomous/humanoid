#ifndef CAN_NODE_HPP
#define CAN_NODE_HPP

#include "can_core.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class CanNode : public rclcpp::Node {
public:
  CanNode();

private:
  autonomy::CanCore can_;
  YAML::Node hardware_config;

  // Subscribers and publishers
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr>
      _subscribers; // Map of topic name to its subscriber

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>
      _publishers; // Map of topic name to its publisher

  
  rclcpp::TimerBase::SharedPtr
      receive_timer_; // Timer to periodically check for CAN messages

  // Methods
  void createSubscribersPublishers();

  void receiveCanMessages(); // Method to be called by the timer

  // Helper methods
  uint32_t generateCanId(const std::string &topic_name);
  std::vector<autonomy::CanMessage>
  createCanMessages(const std::string &topic_name,
                    std::shared_ptr<rclcpp::SerializedMessage> ros_msg);
};

#endif // CAN_NODE_HPP
