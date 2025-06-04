#include "can_node.hpp"
#include <functional>
#include <rclcpp/serialization.hpp>
#include <thread>
#include <chrono>
#include <cstring> // Required for std::memcpy

CanNode::CanNode() : Node("can_node"), can_(this->get_logger()) {
  RCLCPP_INFO(this->get_logger(), "CAN Node has been initialized");
  
  // Load parameters from config/params.yaml
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("device_path", "/dev/canable");
  this->declare_parameter("bustype", "slcan");
  this->declare_parameter("bitrate", 500000);
  
  // Get parameter values
  std::string can_interface = this->get_parameter("can_interface").as_string();
  std::string device_path = this->get_parameter("device_path").as_string();
  std::string bustype = this->get_parameter("bustype").as_string();
  int bitrate = this->get_parameter("bitrate").as_int();
  
  RCLCPP_INFO(this->get_logger(), "Loaded parameters: interface=%s, bustype=%s, bitrate=%d", 
              can_interface.c_str(), bustype.c_str(), bitrate);
  
  // Configure CanCore
  autonomy::CanConfig config;
  config.interface_name = can_interface;
  config.device_path = device_path;
  config.bustype = bustype;
  config.bitrate = bitrate;
  config.receive_timeout_ms = 10000;
  
  // Initialize the CAN interface
  if (can_.initialize(config)) {
    RCLCPP_INFO(this->get_logger(), "CAN Core interface initialized successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN Core");
  }
  
  // Load topic configurations and create subscribers
  loadTopicConfigurations();
  createSubscribers();
}

void CanNode::loadTopicConfigurations() {
  try {
    // Declare and get the topics parameter as a string array
    this->declare_parameter("topics", std::vector<std::string>{"/test_controller"});
    auto topic_names = this->get_parameter("topics").as_string_array();
    
    for (const auto& topic_name : topic_names) {
      // Discover the message type for this topic
      std::string topic_type = discoverTopicType(topic_name);
      
      if (!topic_type.empty()) {
        TopicConfig config;
        config.name = topic_name;
        config.type = topic_type;
        topic_configs_.push_back(config);
        
        RCLCPP_INFO(this->get_logger(), "Loaded topic config: %s (%s)", 
                   topic_name.c_str(), topic_type.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not discover message type for topic: %s", 
                   topic_name.c_str());
      }
    }
    
    if (topic_configs_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid topics found in configuration - CAN node will not subscribe to any topics");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading topic configurations: %s", e.what());
  }
}

std::string CanNode::discoverTopicType(const std::string& topic_name) {
  // Get topic information from ROS graph
  auto topic_names_and_types = this->get_topic_names_and_types();
  
  for (const auto& topic_info : topic_names_and_types) {
    if (topic_info.first == topic_name) {
      if (!topic_info.second.empty()) {
        // Return the first message type (there's usually only one)
        return topic_info.second[0];
      }
    }
  }
  
  // If topic is not found, wait a bit and try again (topic might not be published yet)
  RCLCPP_INFO(this->get_logger(), "Topic '%s' not found, waiting for it to become available...", topic_name.c_str());
  
  // Wait up to 10 seconds for the topic to appear (increased from 5)
  for (int i = 0; i < 100; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    topic_names_and_types = this->get_topic_names_and_types();
    
    for (const auto& topic_info : topic_names_and_types) {
      if (topic_info.first == topic_name) {
        if (!topic_info.second.empty()) {
          RCLCPP_INFO(this->get_logger(), "Found topic '%s' with type '%s'", 
                     topic_name.c_str(), topic_info.second[0].c_str());
          return topic_info.second[0];
        }
      }
    }
  }
  
  RCLCPP_ERROR(this->get_logger(), "Timeout waiting for topic '%s' to become available", topic_name.c_str());
  return "";
}

void CanNode::createSubscribers() {
  for (const auto& topic_config : topic_configs_) {
    // Create generic subscriber that can handle any message type
    auto subscriber = this->create_generic_subscription(
      topic_config.name,
      topic_config.type,
      10,
      [this, topic_name = topic_config.name, topic_type = topic_config.type]
      (std::shared_ptr<rclcpp::SerializedMessage> msg) {
        this->topicCallback(msg, topic_name, topic_type);
      }
    );
    
    subscribers_.push_back(subscriber);
    RCLCPP_INFO(this->get_logger(), "Created generic subscriber for topic: %s (type: %s)", 
               topic_config.name.c_str(), topic_config.type.c_str());
  }
}

void CanNode::topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name, [[maybe_unused]] const std::string& topic_type) {
  // Create CAN message from ROS message
  autonomy::CanMessage can_message = createCanMessage(topic_name, msg);
  
  // Send the CAN message
  if (can_.sendMessage(can_message)) {
    return;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message for topic '%s'", topic_name.c_str());
  }
}

uint32_t CanNode::generateCanId(const std::string& topic_name) {
  // Generate a CAN ID from topic name using hash
  // Use the first 29 bits for extended CAN ID (CAN 2.0B)
  std::hash<std::string> hasher;
  uint32_t hash = static_cast<uint32_t>(hasher(topic_name));
  
  // Mask to 29 bits for extended CAN ID (0x1FFFFFFF)
  // For CAN-FD, we can use the full extended ID range
  return hash & 0x1FFFFFFF;
}

autonomy::CanMessage CanNode::createCanMessage(const std::string& topic_name, std::shared_ptr<rclcpp::SerializedMessage> ros_msg) {
  autonomy::CanMessage can_msg;
  
  // Generate CAN ID from topic name
  can_msg.id = generateCanId(topic_name);
  
  can_msg.is_fd_frame = true;
  can_msg.is_extended_id = true;  
  can_msg.is_remote_frame = false;
  can_msg.is_brs = true;  // Enable bit rate switching for faster data transmission
  can_msg.is_esi = false; // Error state indicator (false = active error state)
  
  // Get current timestamp
  auto now = std::chrono::high_resolution_clock::now();
  can_msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
    now.time_since_epoch()).count();
  
  // Package the ROS message data into CAN frame. CAN-FD supports up to 64 bytes of data
  const size_t max_can_fd_data_size = 64;
  size_t ros_msg_size = ros_msg->size();
  
  // Data Packaging
  if (ros_msg_size <= max_can_fd_data_size) { // if message fits in single CAN-FD frame
    can_msg.data.resize(ros_msg_size);
    std::memcpy(can_msg.data.data(), ros_msg->get_rcl_serialized_message().buffer, ros_msg_size);
    
    RCLCPP_DEBUG(this->get_logger(), "Packaged %zu bytes from topic '%s' into single CAN-FD frame with ID 0x%X", 
                ros_msg_size, topic_name.c_str(), can_msg.id);
  } else {
    // Message is too large for single CAN-FD frame
    // For now, we'll truncate to fit in one frame and log a warning
    // TODO: Implement multi-frame transmission protocol
    RCLCPP_WARN(this->get_logger(), "Message from topic '%s' (%zu bytes) exceeds CAN-FD frame limit (%zu bytes). Truncating.", 
               topic_name.c_str(), ros_msg_size, max_can_fd_data_size);
    
    can_msg.data.resize(max_can_fd_data_size);
    std::memcpy(can_msg.data.data(), ros_msg->get_rcl_serialized_message().buffer, max_can_fd_data_size);
  }
  
  return can_msg;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
