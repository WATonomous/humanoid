#pragma once

#include <cstdint>
#include <map>
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

struct JointSafetyConfig {
  bool enable_position_clamp{true};
  bool enable_velocity_limit{true};
  bool enable_delta_limit{true};
  bool enable_low_pass{true};
  double velocity_max{30.0};   // degrees / second
  double delta_max{2.0};       // degrees / control step
  double low_pass_alpha{0.85}; // q_out = alpha * q_prev + (1-alpha) * q_cmd

  // MIT_CONTROL (compliant holding) gains. Only used when armPoseToMotorCmds is called with
  // control_type=MIT_CONTROL; ignored for POSITION_LOOP etc. Default 0/0 is deliberately a
  // safe no-op (zero stiffness/damping = motor free) -- a joint must be explicitly configured
  // with nonzero mit_kp/mit_kd to actually hold under MIT. Units match the CubeMars AK-series
  // manual's MIT protocol range for this motor (see can/config/mit_profiles.yaml): kp in
  // [0,500], kd in [0,5] -- can_node clamps to the exact per-motor range before sending.
  double mit_kp{0.0};
  double mit_kd{0.0};
};

class JointCommandCore {
public:
  bool loadFromYaml(const YAML::Node& config, const std::string& arm_side);
  bool loadSafetyFromYaml(const YAML::Node& safety_cfg, double control_rate_hz);

  std::vector<common_msgs::msg::MotorCmd> armPoseToMotorCmds(const common_msgs::msg::ArmPose& pose,
                                                             int8_t control_type);

  // Seed the rate-limiter's "previous target" from measured motor angles so the first
  // streamed ArmPose is velocity/delta-limited relative to the arm's ACTUAL pose, not an
  // assumed 0. Without this, an arm not physically at 0 gets a large first command (the
  // limiter ramps from 0), i.e. a slam. motor_positions: motor_id -> measured angle (deg).
  // Joints whose motor is ABSENT from the map (e.g. an unwired wrist) are seeded to 0 --
  // safe, because there is no physical motor there to slam. Returns the number of joints
  // seeded from real feedback (the rest defaulted to 0).
  size_t seedPrevTargetsFromFeedback(const std::map<int, double>& motor_positions);

  const std::vector<double>& prevTargets() const {
    return prev_targets_;
  }

  size_t jointCount() const {
    return joints_.size();
  }

private:
  static JointConfig loadJointConfig(const YAML::Node& joint_node);
  static JointSafetyConfig loadJointSafetyConfig(const YAML::Node& joint_node,
                                                 const JointSafetyConfig& base);
  static double clampAngle(double angle, const JointConfig& joint);
  static double applyCalibration(double angle, const JointConfig& joint);
  static double clampStep(double target, double previous, double delta_max);
  static double applyLowPass(double target, double previous, double alpha);

  std::vector<JointConfig> joints_;
  std::vector<JointSafetyConfig> safety_;
  std::vector<double> prev_targets_;
  bool have_prev_targets_{false};
  double control_rate_hz_{50.0};
};
