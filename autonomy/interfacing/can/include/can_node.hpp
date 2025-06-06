#ifndef CAN_NODE_HPP
#define CAN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_core.hpp"
#include <vector>
#include <string>
#include <memory>

struct TopicConfig {
  // name of the topic, and the message type 
  std::string name;
  std::string type;
};

class CanNode : public rclcpp::Node {
public:
  CanNode();

private:
  autonomy::CanCore can_;
  std::vector<TopicConfig> topic_configs_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscribers_;
  rclcpp::TimerBase::SharedPtr receive_timer_; // Timer to periodically check for CAN messages
  
  // Methods
  void loadTopicConfigurations();
  void createSubscribers();
  std::string discoverTopicType(const std::string& topic_name);
  void topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name, const std::string& topic_type);
  void receiveCanMessages(); // Method to be called by the timer

  // Helper methods
  uint32_t generateCanId(const std::string& topic_name);
  std::vector<autonomy::CanMessage> createCanMessages(const std::string& topic_name, std::shared_ptr<rclcpp::SerializedMessage> ros_msg);
};

#endif // CAN_NODE_HPP
