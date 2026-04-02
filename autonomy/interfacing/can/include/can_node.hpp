#ifndef CAN_NODE_HPP
#define CAN_NODE_HPP

#include "can_core.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>
#include <vector>

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


  // Subscribers and publishers
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr>
      _subscribers; // Map of topic name to its subscriber

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>
      _publishers; // Map of topic name to its publisher

  
  rclcpp::TimerBase::SharedPtr
      receive_timer_; // Timer to periodically check for CAN messages

  // Methods
  void createSubscribersPublishers();

  void topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg,
                     const std::string &topic_name,
                     const std::string &topic_type);
  void receiveCanMessages(); // Method to be called by the timer

  // Helper methods
  uint32_t generateCanId(const std::string &topic_name);
  std::vector<autonomy::CanMessage>
  createCanMessages(const std::string &topic_name,
                    std::shared_ptr<rclcpp::SerializedMessage> ros_msg);
};

#endif // CAN_NODE_HPP
