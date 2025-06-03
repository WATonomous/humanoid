#include "can_node.hpp"
#include <functional>
#include <rclcpp/serialization.hpp>
#include <thread>
#include <chrono>

CanNode::CanNode() : Node("can_node"), can_(autonomy::CanCore(this->get_logger())) {
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
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN Core: %s", can_.getLastError().c_str());
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

void CanNode::topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name, const std::string& topic_type) {
  RCLCPP_INFO(this->get_logger(), "Received message on topic '%s' (type: %s), serialized size: %zu bytes", 
             topic_name.c_str(), topic_type.c_str(), msg->size());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
