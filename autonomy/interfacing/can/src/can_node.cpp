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
  this->declare_parameter("receive_poll_interval_ms", 10); // Parameter for polling interval
  
  // Get parameter values
  std::string can_interface = this->get_parameter("can_interface").as_string();
  std::string device_path = this->get_parameter("device_path").as_string();
  std::string bustype = this->get_parameter("bustype").as_string();
  int bitrate = this->get_parameter("bitrate").as_int();
  long receive_poll_interval_ms = this->get_parameter("receive_poll_interval_ms").as_int();
  
  RCLCPP_INFO(this->get_logger(), "Loaded parameters: interface=%s, bustype=%s, bitrate=%d, poll_interval_ms=%ld", 
              can_interface.c_str(), bustype.c_str(), bitrate, receive_poll_interval_ms);
  
  // Configure CanCore
  autonomy::CanConfig config;
  config.interface_name = can_interface;
  config.device_path = device_path;
  config.bustype = bustype;
  config.bitrate = bitrate;
  config.receive_timeout_ms = 10000; // This specific timeout in CanConfig might be for other uses or can be reviewed.
  
  // Initialize the CAN interface
  if (can_.initialize(config)) {
    RCLCPP_INFO(this->get_logger(), "CAN Core interface initialized successfully");

    // Setup a timer to periodically call receiveCanMessages
    receive_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(receive_poll_interval_ms),
      std::bind(&CanNode::receiveCanMessages, this));
    RCLCPP_INFO(this->get_logger(), "CAN message receive timer started with %ld ms interval.", receive_poll_interval_ms);

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
      std::string topic_type = discoverTopicType(topic_name); // Discover the message type for a given topic
      
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
  auto topic_names_and_types = this->get_topic_names_and_types(); // get topic information from ROS graph
  
  for (const auto& topic_info : topic_names_and_types) {
    if (topic_info.first == topic_name) {
      if (!topic_info.second.empty()) {
        return topic_info.second[0];
      }
    }
  }
  
  // If topic is not found, wait a bit and try again (topic might not be published yet)
  RCLCPP_INFO(this->get_logger(), "Topic '%s' not found, waiting for it to become available...", topic_name.c_str());
  
  // Wait up to 10 seconds for the topic to appear
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
    if (topic_config.name == "/test_controller") {
      subscribers_[topic_config.name] = this->create_generic_subscription(
        topic_config.name,
        topic_config.type,
        10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handleControllerTopic(msg, "/test_controller");
        }
      );
    } else if (topic_config.name == "/cmd_arm_joint") {
      subscribers_[topic_config.name] = this->create_generic_subscription(
        topic_config.name,
        topic_config.type,
        10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handleJointTopic(msg, "/cmd_arm_joint");
        }
      );
    } else if (topic_config.name == "/cmd_hand_joint") {
      subscribers_[topic_config.name] = this->create_generic_subscription(
        topic_config.name,
        topic_config.type,
        10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handleJointTopic(msg, "/cmd_hand_joint");
        }
      );
    } else if (topic_config.name == "/cmd_arm_ee") {
      subscribers_[topic_config.name] = this->create_generic_subscription(
        topic_config.name,
        topic_config.type,
        10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handleEndEffectorTopic(msg, "/cmd_arm_ee");
        }
      );
    } else if (topic_config.name == "/cmd_hand_ee") {
      subscribers_[topic_config.name] = this->create_generic_subscription(
        topic_config.name,
        topic_config.type,
        10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handleEndEffectorTopic(msg, "/cmd_hand_ee");
        }
      );
    } else {
      subscribers_[topic_config.name] = this->create_generic_subscription(
        topic_config.name,
        topic_config.type,
        10,
        [this, topic_name = topic_config.name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handleGenericTopic(msg, topic_name);
        }
      );
    }
    
    RCLCPP_INFO(this->get_logger(), "Created generic subscriber for topic: %s (type: %s)", 
               topic_config.name.c_str(), topic_config.type.c_str());
  }
}

// dummy handler for testing
void CanNode::handleControllerTopic(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name) {
  // Custom CAN ID range for controller messages
  uint32_t can_id = 0x100;  // Controller messages start at 0x100
y  std::vector<autonomy::CanMessage> can_messages = createCanMessages(topic_name, msg, base_can_id);
  
  // Send with high priority
  for (const auto& can_message : can_messages) {
    if (!can_.sendMessage(can_message)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send controller CAN message (ID 0x%X)", can_message.id);
    }
  }
}

void CanNode::handleJointTopic(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name) {
  // Custom CAN ID range for joint messages
  uint32_t base_can_id = 0x200;  // Joint messages start at 0x200
  std::vector<autonomy::CanMessage> can_messages = createCanMessages(topic_name, msg, base_can_id);
  
  // Send joint messages
  for (const auto& can_message : can_messages) {
    if (!can_.sendMessage(can_message)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send joint CAN message (ID 0x%X)", can_message.id);
    }
  }
}

void CanNode::handleEndEffectorTopic(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name) {
  // Custom CAN ID range for end effector messages
  uint32_t base_can_id = 0x300;  // End effector messages start at 0x300
  std::vector<autonomy::CanMessage> can_messages = createCanMessages(topic_name, msg, base_can_id);
  
  // Send end effector messages
  for (const auto& can_message : can_messages) {
    if (!can_.sendMessage(can_message)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send end effector CAN message (ID 0x%X)", can_message.id);
    }
  }


void CanNode::handleGenericTopic(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name) {
  // Original generic handling
  std::vector<autonomy::CanMessage> can_messages = createCanMessages(topic_name, msg);
  sendCanMessages(can_messages, topic_name);
}

// Helper method to reduce code duplication
void CanNode::sendCanMessages(const std::vector<autonomy::CanMessage>& can_messages, const std::string& topic_name) {
  int successful_sends = 0;
  for (const auto& can_message : can_messages) {
    if (can_.sendMessage(can_message)) {
      successful_sends++;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message for topic '%s' (ID 0x%X)", topic_name.c_str(), can_message.id);
    }
  }
  
  if (can_messages.size() > 1) {
    RCLCPP_INFO(this->get_logger(), "Successfully sent %d/%zu CAN frames for topic '%s'", 
                successful_sends, can_messages.size(), topic_name.c_str());
  }
}

void CanNode::receiveCanMessages() {
  autonomy::CanMessage received_msg;
  // Attempt to receive a message. CanCore::receiveMessage is non-blocking.
  if (can_.receiveMessage(received_msg)) {
    // Print the received message details to the console
    std::stringstream ss;
    ss << "Received CAN Message: ID=0x" << std::hex << received_msg.id
       << std::dec << ", DLC=" << static_cast<int>(received_msg.dlc)
       << ", Extended=" << received_msg.is_extended_id
       << ", RTR=" << received_msg.is_remote_frame
       << ", Data=[ ";
    for (size_t i = 0; i < received_msg.data.size(); ++i) {
      ss << "0x" << std::hex << static_cast<int>(received_msg.data[i]) << (i < received_msg.data.size() - 1 ? " " : "");
    }
    ss << std::dec << " ]"; // Switch back to decimal for any further logging if needed
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
  // If receiveMessage returns false, it means no message was available at that moment (non-blocking read)
  // or an error occurred (which CanCore should log). No action needed here for no-message case.
}


// Not sure if this is needed, but it's here for now
uint32_t CanNode::generateCanId(const std::string& topic_name) {
  // Generate a CAN ID from topic name using hash
  // Use the first 29 bits for extended CAN ID (CAN 2.0B)
  std::hash<std::string> hasher;
  uint32_t hash = static_cast<uint32_t>(hasher(topic_name));
  
  // Mask to 29 bits for extended CAN ID (0x1FFFFFFF)
  // For CAN-FD, we can use the full extended ID range
  return hash & 0x1FFFFFFF;
}

std::vector<autonomy::CanMessage> CanNode::createCanMessages(const std::string& topic_name, std::shared_ptr<rclcpp::SerializedMessage> ros_msg, uint32_t can_id) {
  std::vector<autonomy::CanMessage> messages_to_send;
    
  bool is_extended = true; // Use extended CAN ID (29 bits)
  bool is_rtr = false; // Remote Transmission Request

  auto now_chrono = std::chrono::high_resolution_clock::now();
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
    now_chrono.time_since_epoch()).count();

  const uint8_t* ros_msg_buffer = ros_msg->get_rcl_serialized_message().buffer;
  size_t ros_msg_size = ros_msg->size();

  // Determine max data payload per frame
  // For Classic CAN, it's 8 bytes maximum
  // But reserve 1 byte for sequence number if fragmentation is needed.
  const size_t max_data_bytes_per_classic_frame = 8;
  
  size_t max_payload_per_frame = max_data_bytes_per_classic_frame;
  size_t data_chunk_size = max_payload_per_frame;
  bool needs_fragmentation = ros_msg_size > max_payload_per_frame;

  if (needs_fragmentation) {
    // Reserve 1 byte for sequence number if fragmenting
    data_chunk_size = max_payload_per_frame - 1; 
    if (data_chunk_size == 0 && max_payload_per_frame > 0) { // Should not happen with CAN_MAX_DLEN >= 1
        RCLCPP_ERROR(this->get_logger(), "Calculated data_chunk_size is 0, this should not happen. Max payload: %zu", max_payload_per_frame);
        return messages_to_send; // Return empty, indicates error
    }
  }

  if (!needs_fragmentation) {
    autonomy::CanMessage can_msg;
    can_msg.id = can_id;
    can_msg.is_extended_id = is_extended;
    can_msg.is_remote_frame = is_rtr;
    can_msg.timestamp_us = timestamp;
    
    can_msg.data.resize(ros_msg_size);
    std::memcpy(can_msg.data.data(), ros_msg_buffer, ros_msg_size);
    
    RCLCPP_DEBUG(this->get_logger(), "Packaged %zu bytes from topic '%s' into single CAN frame with ID 0x%X (Classic CAN)", 
                ros_msg_size, topic_name.c_str(), can_msg.id);
    messages_to_send.push_back(can_msg);
  } else {
    RCLCPP_INFO(this->get_logger(), "Message from topic '%s' (%zu bytes) is too large for a single CAN frame (max %zu bytes). Fragmenting.",
               topic_name.c_str(), ros_msg_size, max_payload_per_frame);

    size_t bytes_sent = 0;
    uint8_t sequence_number = 0;

    while (bytes_sent < ros_msg_size) {
      autonomy::CanMessage can_fragment;
      can_fragment.id = can_id; // All fragments share the same ID for now
      can_fragment.is_extended_id = is_extended;
      can_fragment.is_remote_frame = is_rtr;
      can_fragment.timestamp_us = timestamp; // Could also update timestamp per fragment

      size_t current_fragment_payload_size = std::min(data_chunk_size, ros_msg_size - bytes_sent);
      
      can_fragment.data.resize(1 + current_fragment_payload_size); // 1 byte for sequence number
      can_fragment.data[0] = sequence_number;
      std::memcpy(can_fragment.data.data() + 1, ros_msg_buffer + bytes_sent, current_fragment_payload_size);
      
      messages_to_send.push_back(can_fragment);
      
      // This is just logging that fragment creation was successful
      // RCLCPP_INFO(this->get_logger(), "Created fragment %u for topic '%s' (ID 0x%X), size %zu (payload %zu)",
                  // sequence_number, topic_name.c_str(), can_fragment.id, can_fragment.data.size(), current_fragment_payload_size);

      bytes_sent += current_fragment_payload_size;
      sequence_number++;

      if (sequence_number == 0 && bytes_sent < ros_msg_size) { // Rollover, too many fragments
          RCLCPP_ERROR(this->get_logger(), "Too many fragments for message from topic '%s'. Max 256 fragments supported with 1-byte sequence.", topic_name.c_str());
          messages_to_send.clear(); // Indicate error by returning no messages
          return messages_to_send;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "Fragmented message from topic '%s' into %zu frames.", topic_name.c_str(), messages_to_send.size());
  }
  
  return messages_to_send;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
