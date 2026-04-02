#ifndef CAN_NODE_HPP
#define CAN_NODE_HPP

#include "can_core.hpp"

// Libraries
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Messages
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "common_msgs/msg/arm_pose.hpp"
#include "common_msgs/msg/hand_pose.hpp"
#include "common_msgs/msg/gripper_pose.hpp"
#include "common_msgs/msg/joint_state.hpp"
#include "common_msgs/msg/encoder.hpp"
#include "common_msgs/msg/motor_cmd.hpp"

class CanNode : public rclcpp::Node {
public:
  CanNode();

private:
  autonomy::CanCore can_;
  YAML::Node hardware_config;
  static constexpr size_t max_payload_per_frame = 8;  // CAN frame max bytes
  static constexpr size_t data_chunk_size = 8;

  // Subscribers and publishers
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> _subscribers;

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>
      _publishers; // Map of topic name to its publisher

  // Callbacks
  void armCMDCallback(const common_msgs::msg::ArmPose::SharedPtr msg);
  void handCMDCallback(const common_msgs::msg::HandPose::SharedPtr msg);
  void gripperCMDCallback(const common_msgs::msg::GripperPose::SharedPtr msg);
  void motorCMDCallback(const common_msgs::msg::MotorCmd::SharedPtr msg);
  
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
