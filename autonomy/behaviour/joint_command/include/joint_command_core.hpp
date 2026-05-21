#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "common_msgs/msg/arm_pose.hpp"
#include "common_msgs/msg/motor_cmd.hpp"
#include "yaml-cpp/yaml.h"

struct JointConfig {
  int8_t motor_id{0};
  double lower_limit{-180.0};
  double upper_limit{180.0};
  int direction{1};
  double zero_offset{0.0};
  bool limit_range{false};
};

class JointCommandCore {
public:
  bool loadFromYaml(const YAML::Node &config, const std::string &arm_side);

  std::vector<common_msgs::msg::MotorCmd>
  armPoseToMotorCmds(const common_msgs::msg::ArmPose &pose,
                     int8_t control_type) const;

  size_t jointCount() const { return joints_.size(); }

private:
  static JointConfig loadJointConfig(const YAML::Node &joint_node);
  static double clampAngle(double angle, const JointConfig &joint);
  static double applyCalibration(double angle, const JointConfig &joint);

  std::vector<JointConfig> joints_;
};
