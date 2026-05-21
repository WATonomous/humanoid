#include "joint_command_core.hpp"

#include <algorithm>
#include <stdexcept>

JointConfig JointCommandCore::loadJointConfig(const YAML::Node &joint_node) {
  JointConfig joint;
  joint.motor_id = static_cast<int8_t>(joint_node["can_id"].as<int>());
  joint.lower_limit = joint_node["lower_limit"].as<double>();
  joint.upper_limit = joint_node["upper_limit"].as<double>();
  joint.direction = joint_node["direction"].as<int>();
  joint.zero_offset = joint_node["zero_offset"].as<double>();
  joint.limit_range = joint_node["limit_range"].as<bool>();
  return joint;
}

bool JointCommandCore::loadFromYaml(const YAML::Node &config,
                                    const std::string &arm_side) {
  joints_.clear();

  if (!config[arm_side]) {
    return false;
  }

  const YAML::Node arm = config[arm_side];
  const std::vector<std::pair<std::string, std::string>> joint_paths = {
      {"shoulder", "pitch"}, {"shoulder", "roll"}, {"shoulder", "yaw"},
      {"elbow", "pitch"},    {"elbow", "roll"},    {"wrist", "pitch"},
  };

  for (const auto &[group, joint_name] : joint_paths) {
    const YAML::Node joint_node = arm[group][joint_name];
    if (!joint_node) {
      return false;
    }
    joints_.push_back(loadJointConfig(joint_node));
  }

  return joints_.size() == 6;
}

double JointCommandCore::clampAngle(double angle, const JointConfig &joint) {
  if (!joint.limit_range) {
    return angle;
  }
  return std::clamp(angle, joint.lower_limit, joint.upper_limit);
}

double JointCommandCore::applyCalibration(double angle,
                                          const JointConfig &joint) {
  return static_cast<double>(joint.direction) * (angle - joint.zero_offset);
}

std::vector<common_msgs::msg::MotorCmd>
JointCommandCore::armPoseToMotorCmds(const common_msgs::msg::ArmPose &pose,
                                     int8_t control_type) const {
  if (joints_.size() != 6) {
    throw std::runtime_error("JointCommandCore is not configured for 6 joints");
  }

  if (pose.shoulder.position.size() < 3 || pose.elbow.position.size() < 2 ||
      pose.wrist.position.size() < 1) {
    throw std::runtime_error(
        "ArmPose must contain 3 shoulder, 2 elbow, and 1 wrist positions");
  }

  const std::vector<double> source_angles = {
      pose.shoulder.position[0], pose.shoulder.position[1],
      pose.shoulder.position[2], pose.elbow.position[0],
      pose.elbow.position[1],    pose.wrist.position[0],
  };

  std::vector<common_msgs::msg::MotorCmd> commands;
  commands.reserve(joints_.size());

  for (size_t i = 0; i < joints_.size(); ++i) {
    common_msgs::msg::MotorCmd cmd;
    cmd.motor_id = joints_[i].motor_id;
    cmd.control_type = control_type;
    cmd.position = static_cast<float>(
        applyCalibration(clampAngle(source_angles[i], joints_[i]), joints_[i]));
    commands.push_back(cmd);
  }

  return commands;
}
