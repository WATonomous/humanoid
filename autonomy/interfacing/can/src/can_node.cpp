#include "can_node.hpp"
#include <chrono>
#include <cstring> // Required for std::memcpy
#include <functional>
#include <rclcpp/serialization.hpp>
#include <thread>

CanNode::CanNode() : Node("can_node"), can_(this->get_logger()) {
  RCLCPP_INFO(this->get_logger(), "CAN Node has been initialized");

  hardware_config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("can") 
                                   + "/config/hardware_params.yaml");

  // Load parameters from config/params.yaml
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("device_path", "/dev/canable");
  this->declare_parameter("bustype", "slcan");
  this->declare_parameter("bitrate", 500000);
  this->declare_parameter("receive_poll_interval_ms", 10);

  // Get parameter values
  std::string can_interface = this->get_parameter("can_interface").as_string();
  std::string device_path = this->get_parameter("device_path").as_string();
  std::string bustype = this->get_parameter("bustype").as_string();
  int bitrate = this->get_parameter("bitrate").as_int();

  long receive_poll_interval_ms =
      this->get_parameter("receive_poll_interval_ms").as_int();

  RCLCPP_INFO(this->get_logger(),
              "Loaded parameters: interface=%s, bustype=%s, bitrate=%d, "
              "poll_interval_ms=%ld",
              can_interface.c_str(), bustype.c_str(), bitrate,
              receive_poll_interval_ms);

  // Configure CanCore
  autonomy::CanConfig config;
  config.interface_name = can_interface;
  config.device_path = device_path;
  config.bustype = bustype;
  config.bitrate = bitrate;
  config.receive_timeout_ms = 10000; // This specific timeout in CanConfig might
                                     // be for other uses or can be reviewed.

  // Initialize the CAN interface
  if (can_.initialize(config)) {
    RCLCPP_INFO(this->get_logger(),
                "CAN Core interface initialized successfully");

    // Setup a timer to periodically call receiveCanMessages
    receive_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(receive_poll_interval_ms),
        std::bind(&CanNode::receiveCanMessages, this));
    RCLCPP_INFO(this->get_logger(),
                "CAN message receive timer started with %ld ms interval.",
                receive_poll_interval_ms);

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
}

// Subscriber callbacks
void CanNode::armCMDCallback(const common_msgs::msg::ArmPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received ArmPose command");
}

void CanNode::handCMDCallback(common_msgs::msg::HandPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received HandPose command");
}

void CanNode::gripperCMDCallback(const common_msgs::msg::GripperPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received GripperPose command: position=%.2f",
              msg->position);
}

void CanNode::motorCMDCallback(const common_msgs::msg::MotorCmd::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received MotorCMD motor=%d", msg->motor_id);
}

// Can message handling
void CanNode::receiveCanMessages() {
  autonomy::CanMessage received_msg;
  // Attempt to receive a message. CanCore::receiveMessage is non-blocking.
  if (can_.receiveMessage(received_msg)) {
    // Print the received message details to the console
    std::stringstream ss;
    ss << "Received CAN Message: ID=0x" << std::hex << received_msg.id
       << std::dec << ", DLC=" << static_cast<int>(received_msg.dlc)
       << ", Extended=" << received_msg.is_extended_id
       << ", RTR=" << received_msg.is_remote_frame << ", Data=[ ";
    for (size_t i = 0; i < received_msg.data.size(); ++i) {
      ss << "0x" << std::hex << static_cast<int>(received_msg.data[i])
         << (i < received_msg.data.size() - 1 ? " " : "");
    }
    ss << std::dec
       << " ]"; // Switch back to decimal for any further logging if needed
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
  // If receiveMessage returns false, it means no message was available at that
  // moment (non-blocking read) or an error occurred (which CanCore should log).
  // No action needed here for no-message case.
}

uint32_t CanNode::generateCanId(const std::string &topic_name) {
  // Generate a CAN ID from topic name using hash
  // Use the first 29 bits for extended CAN ID (CAN 2.0B)
  std::hash<std::string> hasher;
  uint32_t hash = static_cast<uint32_t>(hasher(topic_name));

  // Mask to 29 bits for extended CAN ID (0x1FFFFFFF)
  // For CAN-FD, we can use the full extended ID range
  return hash & 0x1FFFFFFF;
}

std::vector<autonomy::CanMessage>
CanNode::createCanMessages(const std::string &topic_name,
                           std::shared_ptr<rclcpp::SerializedMessage> ros_msg) {
  std::vector<autonomy::CanMessage> messages_to_send;

  uint32_t can_id = generateCanId(topic_name);

  bool is_extended = true; // Use extended CAN ID (29 bits)
  bool is_rtr = false;     // Remote Transmission Request

  auto now_chrono = std::chrono::high_resolution_clock::now();
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                           now_chrono.time_since_epoch())
                           .count();

  const uint8_t *ros_msg_buffer = ros_msg->get_rcl_serialized_message().buffer;
  size_t ros_msg_size = ros_msg->size();

  RCLCPP_INFO(this->get_logger(),
              "Message from topic '%s' (%zu bytes) is too large for a single "
              "CAN frame (max %zu bytes). Fragmenting.",
              topic_name.c_str(), ros_msg_size, max_payload_per_frame);

  size_t bytes_sent = 0;
  uint8_t sequence_number = 0;

  while (bytes_sent < ros_msg_size) {
    autonomy::CanMessage can_fragment;
    can_fragment.id = can_id; // All fragments share the same ID for now
    can_fragment.is_extended_id = is_extended;
    can_fragment.is_remote_frame = is_rtr;
    can_fragment.timestamp_us =
        timestamp; // Could also update timestamp per fragment

    size_t current_fragment_payload_size =
        std::min(data_chunk_size, ros_msg_size - bytes_sent);

    can_fragment.data.resize(
        1 + current_fragment_payload_size); // 1 byte for sequence number
    can_fragment.data[0] = sequence_number;
    std::memcpy(can_fragment.data.data() + 1, ros_msg_buffer + bytes_sent,
                current_fragment_payload_size);

    messages_to_send.push_back(can_fragment);

    // This is just logging that fragment creation was successful
    // RCLCPP_INFO(this->get_logger(), "Created fragment %u for topic '%s' (ID
    // 0x%X), size %zu (payload %zu)", sequence_number, topic_name.c_str(),
    // can_fragment.id, can_fragment.data.size(),
    // current_fragment_payload_size);

    bytes_sent += current_fragment_payload_size;
    sequence_number++;

    if (sequence_number == 0 &&
        bytes_sent < ros_msg_size) { // Rollover, too many fragments
      RCLCPP_ERROR(this->get_logger(),
                    "Too many fragments for message from topic '%s'. Max 256 "
                    "fragments supported with 1-byte sequence.",
                    topic_name.c_str());
      messages_to_send.clear(); // Indicate error by returning no messages
      return messages_to_send;
    }
  }
  // RCLCPP_INFO(this->get_logger(), "Fragmented message from topic '%s' into
  // %zu frames.", topic_name.c_str(), messages_to_send.size());

  return messages_to_send;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
