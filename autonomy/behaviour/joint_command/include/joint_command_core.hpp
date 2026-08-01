#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "common_msgs/msg/arm_pose.hpp"
#include "common_msgs/msg/motor_cmd.hpp"
#include "yaml-cpp/yaml.h"

// Static per-joint hardware facts (motor ID, direction, zero offset, limits).
struct JointConfig {
  int8_t motor_id{0};
  double lower_limit{-180.0};
  double upper_limit{180.0};
  int direction{1};
  double zero_offset{0.0};
  bool limit_range{false};
};

// Tunable per-joint safety/moderation settings, reloadable from YAML.
struct JointSafetyConfig {
  bool enable_position_clamp{true};
  bool enable_velocity_limit{true};
  bool enable_delta_limit{true};
  bool enable_low_pass{true};
  double velocity_max{30.0};   // degrees / second
  double delta_max{2.0};       // degrees / control step
  double low_pass_alpha{0.85}; // q_out = alpha * q_prev + (1-alpha) * q_cmd

  // Reactive trapezoidal ramp: accel_max -> cruise at velocity_max -> decelerate onto target,
  // re-planned every tick since the target can keep moving (IK/teleop). Replaces the plain
  // velocity clamp + low-pass when enabled (both under-deliver speed -- see JointCommand.md).
  // delta_max still applies independently. Off by default: not yet bench-tested on hardware.

  bool enable_trapezoidal_limit{false};
  double accel_max{60.0}; // degrees / second^2

  // MIT_CONTROL PD gains -- ignored for POSITION_LOOP etc. Default 0/0 = joint free until set
  // per-joint. Ranges kp:[0,500], kd:[0,5] (AK-series, see can/config/mit_profiles.yaml);
  // can_node clamps to exact per-motor range before sending.
  double mit_kp{0.0};
  double mit_kd{0.0};
};

// Public API: load hardware config, load safety config, run one moderation tick, seed from
// feedback.
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

// Private helpers (stateless math) + the mutable per-joint state vectors.
private:
  static JointConfig loadJointConfig(const YAML::Node& joint_node);
  static JointSafetyConfig loadJointSafetyConfig(const YAML::Node& joint_node,
                                                 const JointSafetyConfig& base);
  static double clampAngle(double angle, const JointConfig& joint);
  static double applyCalibration(double angle, const JointConfig& joint);
  static double clampStep(double target, double previous, double delta_max);
  static double applyLowPass(double target, double previous, double alpha);
  static double stepTrapezoidal(double target, double prev_pos, double& prev_vel,
                                 double velocity_max, double accel_max, double dt);

  std::vector<JointConfig> joints_;
  std::vector<JointSafetyConfig> safety_;
  std::vector<double> prev_targets_;
  std::vector<double> prev_velocities_; // degrees / second, one per joint, for the trapezoidal ramp
  bool have_prev_targets_{false};
  double control_rate_hz_{50.0};
};
