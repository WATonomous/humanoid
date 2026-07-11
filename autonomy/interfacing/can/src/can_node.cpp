#include "can_node.hpp"
#include <chrono>
#include <cstring> // Required for std::memcpy
#include <functional>
#include <rclcpp/serialization.hpp>
#include <thread>

CanNode::CanNode() : Node("can_node"), can_core(this->get_logger()) {
  RCLCPP_INFO(this->get_logger(), "CAN Node has been initialized");

  // Load DBC file
  {
    std::ifstream idbc(ament_index_cpp::get_package_share_directory("can") +
                       "/config/humanoid.dbc");
    dbc_net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    if (!dbc_net) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load DBC file");
    } else {
      RCLCPP_INFO(this->get_logger(), "DBC file loaded successfully");
      // Build the message ID to IMessage* map for quick lookup during decoding
      for (const auto& msg : dbc_net->Messages()) {
        RCLCPP_INFO(this->get_logger(), "Loaded DBC message: %s (ID 0x%lX)", msg.Name().c_str(),
                    msg.Id());
        can_messages.insert(std::make_pair(msg.Name(), &msg));
        // Subtraction due to extended can frame
        can_id_map.insert(std::make_pair(msg.Id() - 0x80000000, &msg));
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

  int receive_poll_interval_ms = this->get_parameter("receive_poll_interval_ms").as_int();

  RCLCPP_INFO(this->get_logger(),
              "Loaded parameters: interface=%s, bustype=%s, bitrate=%d, "
              "poll_interval_ms=%d",
              can_interface.c_str(), bustype.c_str(), bitrate, receive_poll_interval_ms);

  // Configure CanCore
  CanConfig config;
  config.interface_name = can_interface;
  config.device_path = device_path;
  config.bustype = bustype;
  config.bitrate = bitrate;
  config.receive_timeout_ms = 10000;

  // Initialize the CAN interface
  if (can_core.initialize(config)) {
    RCLCPP_INFO(this->get_logger(), "CAN Core interface initialized successfully");

    // Setup a timer to periodically call receiveCanMessages
    receive_timer_ = this->create_wall_timer(std::chrono::milliseconds(receive_poll_interval_ms),
                                             [this]() { receiveCanMessages(); });

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
  _subscribers["/interfacing/motorCMD"] = this->create_subscription<common_msgs::msg::MotorCmd>(
      "/interfacing/motorCMD", rclcpp::QoS(10),
      std::bind(&CanNode::motorCMDCallback, this, std::placeholders::_1));

  // Create publishers
  _publishers["/interfacing/motorFeedback"] =
      this->create_publisher<common_msgs::msg::MotorFeedback>("/interfacing/motorFeedback", 10);
  RCLCPP_INFO(this->get_logger(), "Subscribers and publishers created successfully");
}

// Subscriber callbacks
const dbcppp::ISignal* CanNode::findSignalByName(const dbcppp::IMessage* msg,
                                                 const std::string& signal_name) {
  for (const auto& signal : msg->Signals()) {
    if (signal.Name() == signal_name) {
      return const_cast<dbcppp::ISignal*>(&signal);
    }
  }
  RCLCPP_ERROR(rclcpp::get_logger("CanNode"), "Signal '%s' not found in message '%s'",
               signal_name.c_str(), msg->Name().c_str());
  return nullptr;
}

double CanNode::decodeSignalPhysical(const dbcppp::ISignal* signal, const uint8_t* data) {
  if (!signal || !data) {
    return 0.0;
  }

  // Decode() returns a uint64 bit pattern. For signed signals dbcppp may leave the
  // value sign-extended in that uint64; assigning it (or RawToPhys without a proper
  // signed cast) yields ~2^64. Mask + sign-extend, then apply DBC scale ourselves.
  uint64_t raw = signal->Decode(data);
  const uint64_t bit_size = signal->BitSize();
  if (bit_size == 0 || bit_size > 64) {
    return 0.0;
  }

  if (bit_size < 64) {
    const uint64_t mask = (1ULL << bit_size) - 1ULL;
    raw &= mask;
    if (signal->ValueType() == dbcppp::ISignal::EValueType::Signed) {
      const uint64_t sign_bit = 1ULL << (bit_size - 1ULL);
      if (raw & sign_bit) {
        raw |= ~mask;
      }
    }
  }

  const double numeric = (signal->ValueType() == dbcppp::ISignal::EValueType::Signed)
                             ? static_cast<double>(static_cast<int64_t>(raw))
                             : static_cast<double>(raw);
  return numeric * signal->Factor() + signal->Offset();
}

void CanNode::motorCMDCallback(const common_msgs::msg::MotorCmd::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received MotorCMD motor=%d control_type=%d",
              static_cast<int>(msg->motor_id), static_cast<int>(msg->control_type));

  const dbcppp::IMessage* dbc_msg = nullptr;

  try {
    switch (msg->control_type) {

    case common_msgs::msg::MotorCmd::DUTY_CYCLE: {
      dbc_msg = can_messages["DutyCycleCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      encodeSignal(findSignalByName(dbc_msg, "DutyCycle"), static_cast<double>(msg->duty_cycle),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::CURRENT_LOOP: {
      dbc_msg = can_messages["CurrentLoopCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      encodeSignal(findSignalByName(dbc_msg, "IqCurrent"), static_cast<double>(msg->current),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::CURRENT_BRAKE: {
      dbc_msg = can_messages["CurrentBrakeCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      encodeSignal(findSignalByName(dbc_msg, "BrakeCurrent"), static_cast<double>(msg->current),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::VELOCITY_LOOP: {
      dbc_msg = can_messages["VelocityLoopCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      encodeSignal(findSignalByName(dbc_msg, "VelocityERPM"), static_cast<double>(msg->velocity),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::POSITION_LOOP: {
      dbc_msg = can_messages["PositionLoopCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      encodeSignal(findSignalByName(dbc_msg, "PositionDeg"), static_cast<double>(msg->position),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::SET_ORIGIN: {
      dbc_msg = can_messages["SetOriginCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      // 0 = Temporary Origin, 1 = Permanent Origin (from DBC VAL_)
      double origin_mode = msg->temporary ? 0.0 : 1.0;
      encodeSignal(findSignalByName(dbc_msg, "OriginMode"), origin_mode, can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::POSITION_VELOCITY: {
      dbc_msg = can_messages["PositionVelocityCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      encodeSignal(findSignalByName(dbc_msg, "PosVelPosition"), static_cast<double>(msg->position),
                   can_msg);
      encodeSignal(findSignalByName(dbc_msg, "PosVelSpeed"), static_cast<double>(msg->velocity),
                   can_msg);
      encodeSignal(findSignalByName(dbc_msg, "PosVelAccel"), static_cast<double>(msg->acceleration),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::MIT_CONTROL: {
      dbc_msg = can_messages["MITControlCmd"];
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      // Can additionally add this to the config file reducing network overhead
      encodeSignal(findSignalByName(dbc_msg, "MIT_KP"), static_cast<double>(msg->kp), can_msg);
      encodeSignal(findSignalByName(dbc_msg, "MIT_KD"), static_cast<double>(msg->kd), can_msg);
      encodeSignal(findSignalByName(dbc_msg, "MIT_Position"), static_cast<double>(msg->position),
                   can_msg);
      encodeSignal(findSignalByName(dbc_msg, "MIT_Velocity"), static_cast<double>(msg->velocity),
                   can_msg);
      encodeSignal(findSignalByName(dbc_msg, "MIT_Torque"), static_cast<double>(msg->torque),
                   can_msg);
      publishCanMessage(can_msg);
      break;
    }

    case common_msgs::msg::MotorCmd::DISABLE: {
      dbc_msg = can_messages["MotorDisableCmd"];
      // MotorDisableCmd has no signals, just send the CAN ID
      CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
      publishCanMessage(can_msg);
      break;
    }

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown control_type=%d, ignoring",
                  static_cast<int>(msg->control_type));
      return;
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to encode CAN message: %s", e.what());
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "CAN message sent for control_type=%d motor=%d",
               static_cast<int>(msg->control_type), static_cast<int>(msg->motor_id));
}

void CanNode::encodeSignal(const dbcppp::ISignal* signal, int64_t phys_value, CanMessage& can_msg) {
  auto raw = signal->PhysToRaw(phys_value);
  signal->Encode(raw, can_msg.data.data());
}

void CanNode::encodeSignal(const dbcppp::ISignal* signal, double phys_value, CanMessage& can_msg) {
  auto raw = signal->PhysToRaw(phys_value);
  signal->Encode(raw, can_msg.data.data());
}

int32_t CanNode::getMessageId(const dbcppp::IMessage* msg, int device_id) const {
  // CAN ID format: base_id | device_id (lower 2 bits)
  return (msg->Id() & 0xFFFFFF00) | (device_id & 0xFF);
}

void CanNode::receiveCanMessages() {
  std::vector<CanMessage> messages;
  CanMessage msg;
  while (can_core.receiveMessage(msg)) {
    messages.push_back(msg);
  }

  for (const auto& message : messages) {
    // all messages are extended frame CAN ids
    int device_id = message.id & 0xFF;
    int base_id = message.id & 0xFFFFFF00;

    if (can_id_map.find(base_id) != can_id_map.end()) {
      // Handling each feedback message on can bus
      std::string msg_name = can_id_map[base_id]->Name();
      if (msg_name == "ServoStatusFeedback") {
        const dbcppp::IMessage* dbc_msg = can_id_map[base_id];

        // Publish to ROS topic
        auto feedback_msg = common_msgs::msg::MotorFeedback();
        feedback_msg.motor_id = device_id;
        feedback_msg.position = static_cast<float>(
            decodeSignalPhysical(findSignalByName(dbc_msg, "FbkPosition"), message.data.data()));
        feedback_msg.velocity = static_cast<float>(
            decodeSignalPhysical(findSignalByName(dbc_msg, "FbkSpeed"), message.data.data()));
        feedback_msg.current = static_cast<float>(
            decodeSignalPhysical(findSignalByName(dbc_msg, "FbkCurrent"), message.data.data()));
        feedback_msg.temperature = static_cast<int8_t>(
            decodeSignalPhysical(findSignalByName(dbc_msg, "FbkTemperature"), message.data.data()));
        feedback_msg.error_code = static_cast<int8_t>(
            decodeSignalPhysical(findSignalByName(dbc_msg, "FbkErrorCode"), message.data.data()));
        RCLCPP_DEBUG(this->get_logger(),
                     "Received feedback for motor %d: pos=%.2f vel=%.2f "
                     "current=%.2f temp=%d error=%d",
                     feedback_msg.motor_id, feedback_msg.position, feedback_msg.velocity,
                     feedback_msg.current, feedback_msg.temperature, feedback_msg.error_code);
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<common_msgs::msg::MotorFeedback>>(
            _publishers["/interfacing/motorFeedback"]);

        if (pub) {
          pub->publish(feedback_msg);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Publisher for /interfacing/motorFeedback not found");
        }
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Received CAN message with unknown ID: 0x%X", base_id);
    }
  }
}

void CanNode::publishCanMessage(CanMessage& can_msg) {
  // Send the CAN message
  if (can_core.sendMessage(can_msg)) {
    RCLCPP_INFO(this->get_logger(), "Sent for CAN message: ID=0x%X", can_msg.id);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message: ID=0x%X", can_msg.id);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}