#include "joint_command_core.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

JointConfig JointCommandCore::loadJointConfig(const YAML::Node& joint_node) {
  JointConfig joint;
  joint.motor_id = static_cast<int8_t>(joint_node["can_id"].as<int>());
  joint.lower_limit = joint_node["lower_limit"].as<double>();
  joint.upper_limit = joint_node["upper_limit"].as<double>();
  joint.direction = joint_node["direction"].as<int>();
  joint.zero_offset = joint_node["zero_offset"].as<double>();
  joint.limit_range = joint_node["limit_range"].as<bool>();
  return joint;
}

bool JointCommandCore::loadFromYaml(const YAML::Node& config, const std::string& arm_side) {
  joints_.clear();

  if (!config[arm_side]) {
    return false;
  }

  const YAML::Node arm = config[arm_side];
  const std::vector<std::pair<std::string, std::string>> joint_paths = {
      {"shoulder", "pitch"}, {"shoulder", "roll"}, {"shoulder", "yaw"},
      {"elbow", "pitch"},    {"elbow", "roll"},    {"wrist", "pitch"},
  };

  for (const auto& [group, joint_name] : joint_paths) {
    const YAML::Node joint_node = arm[group][joint_name];
    if (!joint_node) {
      return false;
    }
    joints_.push_back(loadJointConfig(joint_node));
  }

  const bool ok = joints_.size() == 6;
  if (!ok) {
    return false;
  }

  safety_.assign(joints_.size(), JointSafetyConfig{});
  // Seed at 0 (the assumed safe starting pose an operator positions the arm at before
  // startup) and mark ready immediately, so the very first ArmPose message received is
  // ALSO velocity/delta rate-limited relative to that pose, not just position-clamped.
  // Previously this started false, letting the first command bypass all rate limiting
  // and jump straight to its target -- visible as a sudden snap before smooth tracking.
  prev_targets_.assign(joints_.size(), 0.0);
  have_prev_targets_ = true;
  return true;
}

size_t JointCommandCore::seedPrevTargetsFromFeedback(const std::map<int, double>& motor_positions) {
  std::vector<double> seeded(joints_.size(), 0.0);
  size_t matched = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto it = motor_positions.find(static_cast<int>(joints_[i].motor_id));
    if (it == motor_positions.end()) {
      continue;  // motor not reporting (e.g. unwired wrist) -> leave at 0
    }
    // Inverse of applyCalibration (motor = direction * (cmd - zero_offset)):
    //   cmd = zero_offset + motor / direction
    const double dir =
        (joints_[i].direction == 0) ? 1.0 : static_cast<double>(joints_[i].direction);
    seeded[i] = joints_[i].zero_offset + it->second / dir;
    ++matched;
  }
  prev_targets_ = std::move(seeded);
  have_prev_targets_ = true;
  return matched;
}

double JointCommandCore::clampAngle(double angle, const JointConfig& joint) {
  if (!joint.limit_range) {
    return angle;
  }
  return std::clamp(angle, joint.lower_limit, joint.upper_limit);
}

double JointCommandCore::applyCalibration(double angle, const JointConfig& joint) {
  return static_cast<double>(joint.direction) * (angle - joint.zero_offset);
}

JointSafetyConfig JointCommandCore::loadJointSafetyConfig(const YAML::Node& joint_node,
                                                          const JointSafetyConfig& base) {
  JointSafetyConfig cfg = base;
  if (!joint_node) {
    return cfg;
  }

  if (joint_node["enable_position_clamp"]) {
    cfg.enable_position_clamp = joint_node["enable_position_clamp"].as<bool>();
  }
  if (joint_node["enable_velocity_limit"]) {
    cfg.enable_velocity_limit = joint_node["enable_velocity_limit"].as<bool>();
  }
  if (joint_node["enable_delta_limit"]) {
    cfg.enable_delta_limit = joint_node["enable_delta_limit"].as<bool>();
  }
  if (joint_node["enable_low_pass"]) {
    cfg.enable_low_pass = joint_node["enable_low_pass"].as<bool>();
  }
  if (joint_node["velocity_max"]) {
    cfg.velocity_max = joint_node["velocity_max"].as<double>();
  }
  if (joint_node["delta_max"]) {
    cfg.delta_max = joint_node["delta_max"].as<double>();
  }
  if (joint_node["low_pass_alpha"]) {
    cfg.low_pass_alpha = joint_node["low_pass_alpha"].as<double>();
  }
  cfg.low_pass_alpha = std::clamp(cfg.low_pass_alpha, 0.0, 1.0);
  return cfg;
}

bool JointCommandCore::loadSafetyFromYaml(const YAML::Node& safety_cfg, double control_rate_hz) {
  if (joints_.empty()) {
    return false;
  }

  control_rate_hz_ = control_rate_hz;
  JointSafetyConfig defaults;
  if (safety_cfg["global"]) {
    defaults = loadJointSafetyConfig(safety_cfg["global"], defaults);
  }

  const std::vector<std::pair<std::string, std::string>> joint_paths = {
      {"shoulder", "pitch"}, {"shoulder", "roll"}, {"shoulder", "yaw"},
      {"elbow", "pitch"},    {"elbow", "roll"},    {"wrist", "pitch"},
  };

  safety_.assign(joints_.size(), defaults);
  for (size_t i = 0; i < joint_paths.size(); ++i) {
    const auto& [group, joint_name] = joint_paths[i];
    const YAML::Node joint_node = safety_cfg["joints"][group][joint_name];
    safety_[i] = loadJointSafetyConfig(joint_node, defaults);
  }
  return true;
}

double JointCommandCore::clampStep(double target, double previous, double delta_max) {
  return previous + std::clamp(target - previous, -delta_max, delta_max);
}

double JointCommandCore::applyLowPass(double target, double previous, double alpha) {
  return alpha * previous + (1.0 - alpha) * target;
}

std::vector<common_msgs::msg::MotorCmd>
JointCommandCore::armPoseToMotorCmds(const common_msgs::msg::ArmPose& pose, int8_t control_type) {
  if (joints_.size() != 6) {
    throw std::runtime_error("JointCommandCore is not configured for 6 joints");
  }

  if (pose.shoulder.position.size() < 3 || pose.elbow.position.size() < 2 ||
      pose.wrist.position.size() < 1) {
    throw std::runtime_error("ArmPose must contain 3 shoulder, 2 elbow, and 1 wrist positions");
  }

  const std::vector<double> source_angles = {
      pose.shoulder.position[0], pose.shoulder.position[1], pose.shoulder.position[2],
      pose.elbow.position[0],    pose.elbow.position[1],    pose.wrist.position[0],
  };

  std::vector<common_msgs::msg::MotorCmd> commands;
  commands.reserve(joints_.size());
  std::vector<double> next_targets(joints_.size(), 0.0);

  if (safety_.size() != joints_.size()) {
    safety_.assign(joints_.size(), JointSafetyConfig{});
  }
  if (prev_targets_.size() != joints_.size()) {
    prev_targets_.assign(joints_.size(), 0.0);
    have_prev_targets_ = true;
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointSafetyConfig& safety = safety_[i];

    double target = source_angles[i];
    if (safety.enable_position_clamp) {
      target = clampAngle(target, joints_[i]);
    }

    if (have_prev_targets_) {
      if (safety.enable_velocity_limit && control_rate_hz_ > 0.0) {
        const double velocity_step = std::abs(safety.velocity_max) / control_rate_hz_;
        target = clampStep(target, prev_targets_[i], velocity_step);
      }
      if (safety.enable_delta_limit) {
        target = clampStep(target, prev_targets_[i], std::abs(safety.delta_max));
      }
      if (safety.enable_low_pass) {
        target = applyLowPass(target, prev_targets_[i], safety.low_pass_alpha);
      }
    }

    if (safety.enable_position_clamp) {
      target = clampAngle(target, joints_[i]);
    }
    next_targets[i] = target;

    common_msgs::msg::MotorCmd cmd;
    cmd.motor_id = joints_[i].motor_id;
    cmd.control_type = control_type;
    cmd.position = static_cast<float>(applyCalibration(target, joints_[i]));
    commands.push_back(cmd);
  }

  prev_targets_ = std::move(next_targets);
  have_prev_targets_ = true;
  return commands;
}
