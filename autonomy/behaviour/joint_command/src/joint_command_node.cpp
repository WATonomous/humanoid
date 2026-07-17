#include "joint_command_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

JointCommandNode::JointCommandNode() : Node("joint_command_node") {
  this->declare_parameter("arm_side", "left");
  this->declare_parameter("control_rate_hz", 50.0);
  this->declare_parameter("input_topic", "/behaviour/arm_pose");
  this->declare_parameter("motor_cmd_topic", "/interfacing/motorCMD");
  this->declare_parameter("control_type", common_msgs::msg::MotorCmd::POSITION_LOOP);

  const std::string arm_side = this->get_parameter("arm_side").as_string();
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  const std::string input_topic = this->get_parameter("input_topic").as_string();
  const std::string motor_cmd_topic = this->get_parameter("motor_cmd_topic").as_string();
  control_type_ = static_cast<int8_t>(this->get_parameter("control_type").as_int());

  const YAML::Node hardware_config =
      YAML::LoadFile(ament_index_cpp::get_package_share_directory("joint_command") +
                     "/config/hardware_mapping.yaml");

  if (!core_.loadFromYaml(hardware_config, arm_side)) {
    throw std::runtime_error("Failed to load 6-joint mapping for arm side '" + arm_side + "'");
  }

  const std::string safety_config_path =
      ament_index_cpp::get_package_share_directory("joint_command") + "/config/safety_limits.yaml";
  const YAML::Node safety_config = YAML::LoadFile(safety_config_path);
  const YAML::Node safety_root = safety_config["safety"];
  if (!core_.loadSafetyFromYaml(safety_root, control_rate_hz_)) {
    throw std::runtime_error("Failed to load safety config from '" + safety_config_path + "'");
  }

  motor_cmd_pub_ =
      this->create_publisher<common_msgs::msg::MotorCmd>(motor_cmd_topic, rclcpp::QoS(10));

  arm_pose_sub_ = this->create_subscription<common_msgs::msg::ArmPose>(
      input_topic, rclcpp::QoS(10),
      std::bind(&JointCommandNode::armPoseCallback, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                              std::bind(&JointCommandNode::controlTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Joint command node ready: arm=%s, joints=%zu, rate=%.1f Hz",
              arm_side.c_str(), core_.jointCount(), control_rate_hz_);
}

void JointCommandNode::armPoseCallback(const common_msgs::msg::ArmPose::SharedPtr msg) {
  try {
    latest_cmds_ = core_.armPoseToMotorCmds(*msg, control_type_);
    have_latest_cmds_ = true;
    publishMotorCommands(latest_cmds_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process ArmPose: %s", e.what());
  }
}

void JointCommandNode::controlTimerCallback() {
  if (!have_latest_cmds_) {
    return;
  }
  publishMotorCommands(latest_cmds_);
}

void JointCommandNode::publishMotorCommands(const std::vector<common_msgs::msg::MotorCmd>& cmds) {
  for (const auto& cmd : cmds) {
    motor_cmd_pub_->publish(cmd);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandNode>());
  rclcpp::shutdown();
  return 0;
}