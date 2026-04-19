#include "can_node.hpp"
#include <chrono>
#include <cstring> // Required for std::memcpy
#include <functional>
#include <rclcpp/serialization.hpp>
#include <thread>

CanNode::CanNode() : Node("can_node"), can_core(this->get_logger()) {
  RCLCPP_INFO(this->get_logger(), "CAN Node has been initialized");

  hardware_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("can") 
                                   + "/config/hardware_mapping.yaml");

  // Load DBC file
  {
      std::ifstream idbc(ament_index_cpp::get_package_share_directory("can") + "/config/humanoid.dbc");
      dbc_net = dbcppp::INetwork::LoadDBCFromIs(idbc);
      if (!dbc_net) {
          RCLCPP_ERROR(this->get_logger(), "Failed to load DBC file");
      } else {
          RCLCPP_INFO(this->get_logger(), "DBC file loaded successfully");
          // Build the message ID to IMessage* map for quick lookup during decoding
          for (const auto& msg : dbc_net->Messages()) {
              RCLCPP_INFO(this->get_logger(), "Loaded DBC message: %s (ID 0x%X)", msg.Name().c_str(), msg.Id());
              can_messages.insert(std::make_pair(msg.Name(), &msg));
          }
      }
  }

  // Load parameters from config/params.yaml
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("device_path", "/dev/canable");
  this->declare_parameter("bustype", "slcan");
  this->declare_parameter("bitrate", 500000);
  this->declare_parameter("receive_poll_interval_ms", 10);
  this->declare_parameter("receive_timeout_ms", 10000);

  // Get parameter values
  std::string can_interface = this->get_parameter("can_interface").as_string();
  std::string device_path = this->get_parameter("device_path").as_string();
  std::string bustype = this->get_parameter("bustype").as_string();
  int bitrate = this->get_parameter("bitrate").as_int();

  int receive_poll_interval_ms =
      this->get_parameter("receive_poll_interval_ms").as_int();

  RCLCPP_INFO(this->get_logger(),
              "Loaded parameters: interface=%s, bustype=%s, bitrate=%d, "
              "poll_interval_ms=%d",
              can_interface.c_str(), bustype.c_str(), bitrate,
              receive_poll_interval_ms);

  // Configure CanCore
  CanConfig config;
  config.interface_name = can_interface;
  config.device_path = device_path;
  config.bustype = bustype;
  config.bitrate = bitrate;
  config.receive_timeout_ms = 10000;

  // Initialize the CAN interface
  if (can_core.initialize(config)) {
    RCLCPP_INFO(this->get_logger(),
                "CAN Core interface initialized successfully");

    // Setup a timer to periodically call receiveCanMessages
    receive_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(receive_poll_interval_ms),
        [this]() { recieveCanMessages(); }
    );

  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN Core");
  }

  // Load topic configurations and create subscribers
  createSubscribersPublishers();
}

void CanNode::createSubscribersPublishers() {
  _subscribers.clear();
  _publishers.clear();

  // Create subscribers
  _subscribers["/armCMD"] = this->create_subscription<common_msgs::msg::ArmPose>(
      "/armCMD", rclcpp::QoS(10),
      std::bind(&CanNode::armCMDCallback, this, std::placeholders::_1));

  _subscribers["/handCMD"] = this->create_subscription<common_msgs::msg::HandPose>(
      "/handCMD", rclcpp::QoS(10),
      std::bind(&CanNode::handCMDCallback, this, std::placeholders::_1));

  _subscribers["/gripperCMD"] = this->create_subscription<common_msgs::msg::GripperPose>(
      "/gripperCMD", rclcpp::QoS(10),
      std::bind(&CanNode::gripperCMDCallback, this, std::placeholders::_1));

  _subscribers["/motorCMD"] = this->create_subscription<common_msgs::msg::MotorCmd>(
      "/motorCMD", rclcpp::QoS(10),
      std::bind(&CanNode::motorCMDCallback, this, std::placeholders::_1));

  // Create publishers
  _publishers["/encoder"] = this->create_publisher<common_msgs::msg::Encoder>("/encoder", 10);
  RCLCPP_INFO(this->get_logger(), "Subscribers and publishers created successfully");
}

// Subscriber callbacks
void CanNode::armCMDCallback(const common_msgs::msg::ArmPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received ArmPose command");
}

void CanNode::handCMDCallback(const common_msgs::msg::HandPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received HandPose command");
}

void CanNode::gripperCMDCallback(const common_msgs::msg::GripperPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received GripperPose command: position=%.2f",
              msg->position);
}

void CanNode::motorCMDCallback(const common_msgs::msg::MotorCmd::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received MotorCMD motor=%d", static_cast<int>(msg->motor_id));

    const dbcppp::IMessage* dbc_msg = can_messages["PositionLoopCmd"];
    CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());

    encodeSignal(dbc_msg->Signals_Get(0), msg->state.position[0], can_msg);

    publishCanMessage(can_msg);
}

void CanNode::encodeSignal(const dbcppp::ISignal& signal, int64_t phys_value, CanMessage& can_msg) {
    auto raw = signal.PhysToRaw(phys_value);
    signal.Encode(raw, can_msg.data.data());
}

void CanNode::encodeSignal(const dbcppp::ISignal& signal, double phys_value, CanMessage& can_msg) {
    auto raw = signal.PhysToRaw(phys_value);
    signal.Encode(raw, can_msg.data.data());
}

int32_t CanNode::getMessageId(const dbcppp::IMessage* msg, int device_id) const {
    // CAN ID format: base_id | device_id (lower 2 bits)
    return msg->Id() | (device_id & 0x03);
}

/// TODO: Implement the receiveCanMessages method to read CAN messages and publish to ROS topics
void CanNode::recieveCanMessages() {
  std::vector<CanMessage> messages;
  CanMessage msg;
  while (can_core.receiveMessage(msg)) {
    messages.push_back(msg);
  }

  for (const auto& message : messages) {
    RCLCPP_INFO(this->get_logger(), "Received CAN message: ID=0x%X, DLC=%d",
                message.id, message.dlc);
    // Here you would decode the CAN message using the DBC and publish to ROS topics
  }
}

void CanNode::publishCanMessage(CanMessage& can_msg) {

  // Send the CAN message
  if (can_core.sendMessage(can_msg)) {
    RCLCPP_INFO(this->get_logger(), "Sentfor CAN message: ID=0x%X", can_msg.id);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message: ID=0x%X", can_msg.id);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
