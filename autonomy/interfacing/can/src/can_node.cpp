#include "can_node.hpp"

CanNode::CanNode() : Node("can_node"), can_(autonomy::CanCore(this->get_logger())) {
  RCLCPP_INFO(this->get_logger(), "CAN Node has been initialized");
  
  // Load parameters from config/params.yaml
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("device_path", "/dev/ttyACM0");
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
  config.receive_timeout_ms = 100;
  
  // Initialize the CAN interface
  if (can_.initialize(config)) {
    RCLCPP_INFO(this->get_logger(), "CAN Core interface initialized successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN Core: %s", can_.getLastError().c_str());
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
