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
              RCLCPP_INFO(this->get_logger(), "Loaded DBC message: %s (ID 0x%lX)", msg.Name().c_str(), msg.Id());
              can_messages.insert(std::make_pair(msg.Name(), &msg));
              can_id_map.insert(std::make_pair(msg.Id(), &msg));
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
  // _subscribers["/armCMD"] = this->create_subscription<common_msgs::msg::ArmPose>(
  //     "/armCMD", rclcpp::QoS(10),
  //     std::bind(&CanNode::armCMDCallback, this, std::placeholders::_1));

  // _subscribers["/handCMD"] = this->create_subscription<common_msgs::msg::HandPose>(
  //     "/handCMD", rclcpp::QoS(10),
  //     std::bind(&CanNode::handCMDCallback, this, std::placeholders::_1));

  // _subscribers["/gripperCMD"] = this->create_subscription<common_msgs::msg::GripperPose>(
  //     "/gripperCMD", rclcpp::QoS(10),
  //     std::bind(&CanNode::gripperCMDCallback, this, std::placeholders::_1));

  _subscribers["/interfacing/motorCMD"] = this->create_subscription<common_msgs::msg::MotorCmd>(
      "/interfacing/motorCMD", rclcpp::QoS(10),
      std::bind(&CanNode::motorCMDCallback, this, std::placeholders::_1));

  // Create publishers
  _publishers["/interfacing/motorFeedback"] = this->create_publisher<common_msgs::msg::Encoder>("/interfacing/motorFeedback", 10);
  RCLCPP_INFO(this->get_logger(), "Subscribers and publishers created successfully");
}

// Subscriber callbacks
// void CanNode::armCMDCallback(const common_msgs::msg::ArmPose::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "Received ArmPose command");
// }

// void CanNode::handCMDCallback(const common_msgs::msg::HandPose::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "Received HandPose command");
// }

// void CanNode::gripperCMDCallback(const common_msgs::msg::GripperPose::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "Received GripperPose command: position=%.2f",
//               msg->position);
// }

const dbcppp::ISignal* CanNode::findSignalByName(const dbcppp::IMessage* msg, const std::string& signal_name) {
    for (const auto& signal : msg->Signals()) {
        if (signal.Name() == signal_name) {
            return const_cast<dbcppp::ISignal*>(&signal);
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("CanNode"), "Signal '%s' not found in message '%s'", signal_name.c_str(), msg->Name().c_str());
    return nullptr;
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
            encodeSignal(findSignalByName(dbc_msg, "DutyCycle"), static_cast<double>(msg->duty_cycle), can_msg);
            publishCanMessage(can_msg);
            break;
        }

        case common_msgs::msg::MotorCmd::CURRENT_LOOP: {
            dbc_msg = can_messages["CurrentLoopCmd"];
            CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
            encodeSignal(findSignalByName(dbc_msg, "IqCurrent"), static_cast<double>(msg->current), can_msg);
            publishCanMessage(can_msg);
            break;
        }

        case common_msgs::msg::MotorCmd::CURRENT_BRAKE: {
            dbc_msg = can_messages["CurrentBrakeCmd"];
            CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
            encodeSignal(findSignalByName(dbc_msg, "BrakeCurrent"), static_cast<double>(msg->current), can_msg);
            publishCanMessage(can_msg);
            break;
        }

        case common_msgs::msg::MotorCmd::VELOCITY_LOOP: {
            dbc_msg = can_messages["VelocityLoopCmd"];
            CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
            encodeSignal(findSignalByName(dbc_msg, "VelocityERPM"), static_cast<double>(msg->velocity), can_msg);
            publishCanMessage(can_msg);
            break;
        }

        case common_msgs::msg::MotorCmd::POSITION_LOOP: {
            dbc_msg = can_messages["PositionLoopCmd"];
            CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
            encodeSignal(findSignalByName(dbc_msg, "PositionDeg"), static_cast<double>(msg->position), can_msg);
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
            encodeSignal(findSignalByName(dbc_msg, "PosVelPosition"),     static_cast<double>(msg->position),     can_msg);
            encodeSignal(findSignalByName(dbc_msg, "PosVelSpeed"),        static_cast<double>(msg->velocity),     can_msg);
            encodeSignal(findSignalByName(dbc_msg, "PosVelAccel"),        static_cast<double>(msg->acceleration), can_msg);
            publishCanMessage(can_msg);
            break;
        }

        case common_msgs::msg::MotorCmd::MIT_CONTROL: {
            dbc_msg = can_messages["MITControlCmd"];
            CanMessage can_msg(getMessageId(dbc_msg, msg->motor_id), dbc_msg->MessageSize());
            // NOTE: MIT mode requires KP/KD gains — these are not in the message yet
            // TODO: add kp, kd fields to MotorCmd.msg
            encodeSignal(findSignalByName(dbc_msg, "MIT_Position"), static_cast<double>(msg->position), can_msg);
            encodeSignal(findSignalByName(dbc_msg, "MIT_Velocity"), static_cast<double>(msg->velocity), can_msg);
            encodeSignal(findSignalByName(dbc_msg, "MIT_Torque"),   static_cast<double>(msg->torque),   can_msg);
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

/// TODO: Implement the receiveCanMessages method to read CAN messages and publish to ROS topics
void CanNode::recieveCanMessages() {
  std::vector<CanMessage> messages;
  CanMessage msg;
  while (can_core.receiveMessage(msg)) {
    messages.push_back(msg);
  }

  for (const auto& message : messages) {
    // all messages are extended frame CAN ids
    unsigned device_id = message.id & 0xFF; 
    unsigned int base_id = message.id & 0xFFFFFF00;

    // if (can_id_map.find(base_id) != can_id_map.end()) {
    //     // Handling each feedback message on can bus
    //     // switch (can_id_map[base_id]->Name()) {
    //     //   case "ServoStatusFeedback": {

    //     //   }
    //     // }
    // } else {
    //   RCLCPP_WARN(this->get_logger(), "Received CAN message with unknown ID: 0x%X", base_id);
    // }
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}