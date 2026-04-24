#pragma once

#include "can_core.hpp"

// Libraries
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <dbcppp/Network.h>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

// Messages
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

#include "common_msgs/msg/arm_pose.hpp"
#include "common_msgs/msg/gripper_pose.hpp"
#include "common_msgs/msg/hand_pose.hpp"
#include "common_msgs/msg/joint_state.hpp"
#include "common_msgs/msg/motor_cmd.hpp"
#include "common_msgs/msg/motor_feedback.hpp"
#include "std_msgs/msg/string.hpp"

class CanNode : public rclcpp::Node {
public:
  CanNode();

private:
  CanCore can_core;
  YAML::Node hardware_config;
  // Can messages
  // Map of CAN message ID to its DBC definition for decoding
  std::unordered_map<std::string, const dbcppp::IMessage *> can_messages;
  std::unordered_map<int, const dbcppp::IMessage *>
      can_id_map; // for programming convience
  std::unique_ptr<dbcppp::INetwork>
      dbc_net; // decoding CAN messages using DBC file

  const dbcppp::ISignal *
  findSignalByName(const dbcppp::IMessage *msg,
                   const std::string &signal_name); // O(signal_num)

  static constexpr size_t max_payload_per_frame = 8; // CAN frame max bytes
  static constexpr size_t data_chunk_size = 8;

  void publishCanMessage(CanMessage &can_msg);
  void encodeSignal(const dbcppp::ISignal *signal, int64_t phys_value,
                    CanMessage &can_msg);
  void encodeSignal(const dbcppp::ISignal *signal, double phys_value,
                    CanMessage &can_msg);
  int32_t getMessageId(const dbcppp::IMessage *msg, int device_id) const;
  void recieveCanMessages();

  // Subscribers and publishers
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>
      _subscribers;

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>
      _publishers; // Map of topic name to its publisher

  // Callbacks
  void motorCMDCallback(const common_msgs::msg::MotorCmd::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr
      receive_timer_; // Timer to periodically check for CAN messages

  // Methods
  void createSubscribersPublishers();

  // Helper methods
  uint32_t generateCanId(const std::string &topic_name);
  std::vector<CanMessage>
  createCanMessages(const std::string &topic_name,
                    std::shared_ptr<rclcpp::SerializedMessage> ros_msg);
};
