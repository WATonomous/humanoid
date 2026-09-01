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
  this->declare_parameter("feedback_topic", "/interfacing/motorFeedback");
  this->declare_parameter("command_timeout_sec", 10.0);

  const std::string arm_side = this->get_parameter("arm_side").as_string();
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  const std::string input_topic = this->get_parameter("input_topic").as_string();
  const std::string motor_cmd_topic = this->get_parameter("motor_cmd_topic").as_string();
  control_type_ = static_cast<int8_t>(this->get_parameter("control_type").as_int());
  const std::string feedback_topic = this->get_parameter("feedback_topic").as_string();
  command_timeout_sec_ = this->get_parameter("command_timeout_sec").as_double();

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

  feedback_sub_ = this->create_subscription<common_msgs::msg::MotorFeedback>(
      feedback_topic, rclcpp::QoS(20),
      std::bind(&JointCommandNode::motorFeedbackCallback, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                              std::bind(&JointCommandNode::controlTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Joint command node ready: arm=%s, joints=%zu, rate=%.1f Hz",
              arm_side.c_str(), core_.jointCount(), control_rate_hz_);
}

void JointCommandNode::armPoseCallback(const common_msgs::msg::ArmPose::SharedPtr msg) {
  // Refuse to act until the rate-limiter has been seeded from the arm's real pose. We seed
  // lazily on the first ArmPose (feedback has been streaming by the time an operator starts
  // commanding), using the freshest feedback right before motion begins.
  if (!seeded_from_feedback_) {
    if (latest_feedback_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Ignoring ArmPose: no motor feedback received yet, cannot seed the "
                           "rate-limiter from the real pose. No command sent.");
      return;
    }
    const size_t matched = core_.seedPrevTargetsFromFeedback(latest_feedback_);
    seeded_from_feedback_ = true;
    const auto& seeded = core_.prevTargets();
    std::string s;
    for (size_t i = 0; i < seeded.size(); ++i) {
      s += (i ? ", " : "") + std::to_string(seeded[i]);
    }
    RCLCPP_INFO(this->get_logger(),
                "Rate-limiter seeded from feedback: %zu/%zu joints from real position "
                "(others default 0 = unwired). prev_targets(cmd-frame deg)=[%s]. Now "
                "accepting ArmPose; motion ramps from here.",
                matched, core_.jointCount(), s.c_str());
  }

  // Only recompute and cache the rate-limited targets here. Actual publishing happens
  // exclusively on control_timer_'s own steady 50Hz clock (below) -- previously this also
  // called publishMotorCommands() directly, so with ArmPose already arriving at ~50Hz from
  // task_space_real.py, motor commands were sent from two independent, phase-unrelated 50Hz
  // sources at once (up to ~100Hz combined, jittery), not the intended steady 50Hz.
  try {
    latest_cmds_ = core_.armPoseToMotorCmds(*msg, control_type_);
    have_latest_cmds_ = true;
    last_armpose_time_ = this->get_clock()->now();
    have_armpose_time_ = true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process ArmPose: %s", e.what());
  }
}

void JointCommandNode::motorFeedbackCallback(const common_msgs::msg::MotorFeedback::SharedPtr msg) {
  // Record only; seeding happens on the first ArmPose using this latest snapshot.
  latest_feedback_[static_cast<int>(msg->motor_id)] = static_cast<double>(msg->position);
}

void JointCommandNode::controlTimerCallback() {
  if (!have_latest_cmds_) {
    return;
  }
  if (have_armpose_time_ &&
      (this->get_clock()->now() - last_armpose_time_).seconds() > command_timeout_sec_) {
    // No fresh ArmPose within the timeout window: stop republishing the stale cached
    // target and force a fresh seed-from-feedback + ramp on the next real command,
    // instead of instantly resuming motion toward a target that could be arbitrarily old.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "ArmPose stream stale (no message in %.1fs): halting motor "
                         "commands until a fresh ArmPose re-seeds the rate-limiter.",
                         command_timeout_sec_);
    have_latest_cmds_ = false;
    seeded_from_feedback_ = false;
    return;
  }
  publishMotorCommands(latest_cmds_);
}

void JointCommandNode::publishMotorCommands(const std::vector<common_msgs::msg::MotorCmd>& cmds) {
  // One-shot proof of how many MotorCmds this node actually emits per cycle (and for which
  // motor ids), logged from INSIDE the node so it's independent of any subscriber-side
  // (ros2 echo/hz) message-drop artifact.
  if (!logged_publish_count_) {
    std::string ids;
    for (const auto& cmd : cmds) {
      ids += (ids.empty() ? "" : ", ") + std::to_string(static_cast<int>(cmd.motor_id));
    }
    RCLCPP_INFO(this->get_logger(), "Publishing %zu MotorCmd per cycle; motor_ids=[%s]",
                cmds.size(), ids.c_str());
    logged_publish_count_ = true;
  }

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
