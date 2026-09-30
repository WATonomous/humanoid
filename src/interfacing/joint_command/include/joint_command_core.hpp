#pragma once

#include <cstdint>
#include <map>
#include <optional>
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

// Which CubeMars MIT dialect a joint's drive speaks. They disagree about what the status byte
// means (see can/include/mit_protocol.hpp), so the watchdog has to know.
enum class MitDriveFamily { Gl2, Ak };

// What a MIT joint does when the node stops trusting its command stream (fault, stale ArmPose,
// seeding, shutdown). Limp is fine for the unloaded wrist; for a gravity-loaded AK joint it
// drops the arm, so those default to Damp: kp = 0, kd = mit_fault_kd, and the arm sinks slowly.
enum class MitFaultAction { Limp, Damp };

struct JointSafetyConfig {
  bool enable_position_clamp{true};
  bool enable_velocity_limit{true};
  bool enable_delta_limit{true};
  bool enable_low_pass{true};
  double velocity_max{30.0};   // degrees / second (a true per-second bound: the limiter runs
                               // on the control tick, not per incoming message)
  double delta_max{2.0};       // degrees / control step -- backstop under velocity_max
  double low_pass_alpha{0.85}; // q_out = alpha * q_prev + (1-alpha) * q_cmd

  // Which MotorCmd::control_type this joint is driven with. -1 = the node's default
  // (joint_command.yaml). The wrist/gripper GL40s need MIT_CONTROL (0); the AK joints run
  // POSITION_LOOP (4), which has no gains.
  int control_type{-1};

  // MIT_CONTROL (compliant holding) gains, in physical units (N.m/rad, N.m.s/rad).
  // can_node scales them to the drive's 12-bit codes via can/config/mit_profiles.yaml.
  // Default 0/0 is deliberately a safe no-op: zero stiffness/damping = motor free. A joint
  // must be explicitly configured to hold under MIT.
  double mit_kp{0.0};
  double mit_kd{0.0};

  // MIT watchdog. A PD drive has no internal limit checking, so these are the only thing
  // between a stuck joint and a motor pulling at full current forever.
  double mit_max_torque{0.3};       // N.m -- fault above this
  double mit_max_track_err{12.0};   // deg -- fault if the joint lags its setpoint by more
  double mit_feedback_timeout{0.2}; // s without feedback before faulting

  MitDriveFamily mit_family{MitDriveFamily::Gl2};
  // Default follows the family (ak -> Damp, gl2 -> Limp) unless the YAML sets it explicitly.
  MitFaultAction mit_fault_action{MitFaultAction::Limp};
  bool mit_fault_action_explicit{false};
  double mit_fault_kd{0.0}; // N.m.s/rad, used by Damp; must be > 0 when Damp is in effect

  // Gravity feed-forward (MIT joints only), sent as MotorCmd.torque. scale 0 = off; the model
  // still runs so its prediction can be compared with measured torque before enabling it.
  double gravity_ff_scale{0.0};
  double gravity_ff_max_torque{0.0}; // N.m, |feed-forward| is clamped to this
  // ArmPose cmd frame -> URDF frame for the gravity model: q_urdf = urdf_direction * q_cmd +
  // urdf_offset_deg. Needed on EVERY joint (a shoulder's load depends on the elbow angle).
  int urdf_direction{1};
  double urdf_offset_deg{0.0};
  // Cmd-frame angle the gravity model uses while this joint is UNPOWERED (no feedback at the
  // seed). Unset = an unpowered joint zeroes all feed-forward. Only valid while the limp joint
  // is strapped at that angle -- left free it follows gravity, not a constant.
  std::optional<double> gravity_assume_deg;
};

// One motor's latest feedback, as the MIT watchdog needs it. Kept ROS-free so the checks are
// unit-testable: the node converts MotorFeedback + clock into this.
struct MotorFeedbackSample {
  double position_deg{0.0}; // MOTOR frame, as published on /interfacing/motorFeedback
  double torque_nm{0.0};
  int status{0}; // GL II: 0 Disable / 1 Enable / else fault. AK: 0 = no fault, else fault.
  double age_s{0.0};
};

// Outcome of seeding from real feedback.
struct SeedReport {
  size_t matched{0};                // joints seeded from real feedback
  std::vector<size_t> unmatched;    // joint indices with no feedback (e.g. unwired)
  std::vector<size_t> out_of_range; // joints physically outside their configured limits
  std::string describe() const;
};

class JointCommandCore {
public:
  bool loadFromYaml(const YAML::Node& config, const std::string& arm_side);
  bool loadSafetyFromYaml(const YAML::Node& safety_cfg, double control_rate_hz);

  // Why loadSafetyFromYaml refused (gain rule violated, etc). Empty when it succeeded.
  const std::string& lastError() const {
    return last_error_;
  }

  // Advance the moderation pipeline ONE control tick toward `pose` and return the commands to
  // publish. Call this at control_rate_hz (NOT once per incoming ArmPose): velocity_max is
  // enforced per tick, so calling it faster makes the arm move faster.
  std::vector<common_msgs::msg::MotorCmd> armPoseToMotorCmds(const common_msgs::msg::ArmPose& pose,
                                                             int8_t default_control_type);

  // Each MIT joint's fault-action frame: kp = 0 always, so the position field cannot pull the
  // joint anywhere. Limp joints get kd = 0 (zero torque); Damp joints get kd = mit_fault_kd.
  // Used to poke drives into answering before seeding (they only speak when spoken to), and
  // to hold MIT joints safe after a fault or a stale stream. damped_only skips Limp joints.
  std::vector<common_msgs::msg::MotorCmd> mitSafeCommands(bool damped_only = false) const;

  // MIT_ENTER / MIT_EXIT for every MIT joint (a GL II ignores commands until it is entered).
  // limp_only restricts it to Limp joints -- a Damp joint must keep receiving damping frames,
  // not be exited, or a gravity-loaded arm drops.
  std::vector<common_msgs::msg::MotorCmd> mitModeCommands(int8_t control_type,
                                                          bool limp_only = false) const;
  bool hasDampedMitJoints() const;

  // Seed the rate-limiter's "previous target" from measured motor angles so the first
  // streamed ArmPose is velocity/delta-limited relative to the arm's ACTUAL pose, not an
  // assumed 0. Without this, an arm not physically at 0 gets a large first command (the
  // limiter ramps from 0), i.e. a slam. motor_positions: motor_id -> measured angle (deg,
  // motor frame). Joints whose motor is ABSENT are reported as unmatched and excluded from
  // commands (MIT ones held at kp = 0) until the next seed -- never ramped from an assumed 0.
  SeedReport seedPrevTargetsFromFeedback(const std::map<int, double>& motor_positions);

  // Joints whose seeded position is outside their configured limits: the calibration and the
  // hardware disagree, so commanding them would walk them to the limit on their own. They are
  // excluded from further commands (MIT ones are held limp) until the mapping is fixed.
  void blockJoints(const std::vector<size_t>& indices);
  bool isBlocked(size_t joint) const;
  bool isUnpowered(size_t joint) const;

  // MIT watchdog. Returns a human-readable fault, or nullopt when every MIT joint is healthy.
  std::optional<std::string>
  checkMitFaults(const std::map<int, MotorFeedbackSample>& feedback) const;

  bool isMitJoint(size_t joint) const;
  std::vector<int> mitMotorIds() const;
  int8_t motorId(size_t joint) const;
  std::string jointName(size_t joint) const;

  const std::vector<double>& prevTargets() const {
    return prev_targets_;
  }

  // What was last commanded to each motor, in the MOTOR frame (degrees) -- the frame the
  // feedback comes back in, so tracking error is a straight subtraction.
  const std::vector<double>& lastMotorCmdDeg() const {
    return last_motor_cmd_deg_;
  }

  // Gravity model's holding torque for the last commanded pose, MOTOR frame (N.m), before
  // scale/clamp/ramp -- compare with measured feedback torque. Zero while any joint is blocked.
  const std::vector<double>& lastGravityTorqueMotor() const {
    return last_gravity_torque_motor_;
  }

  size_t jointCount() const {
    return joints_.size();
  }

  const JointSafetyConfig& safety(size_t joint) const {
    return safety_.at(joint);
  }

  const JointConfig& joint(size_t joint) const {
    return joints_.at(joint);
  }

  // kp as the drive will actually apply it after 12-bit quantisation (nearest code), so the
  // gain rule is checked against what the motor gets, not what the YAML asked for.
  static double quantiseKp(double kp, double kp_max = 500.0);
  static double quantiseKd(double kd, double kd_max = 5.0);

private:
  static JointConfig loadJointConfig(const YAML::Node& joint_node);
  static JointSafetyConfig loadJointSafetyConfig(const YAML::Node& joint_node,
                                                 const JointSafetyConfig& base);
  static double clampAngle(double angle, const JointConfig& joint);
  static double applyCalibration(double angle, const JointConfig& joint);
  static double clampStep(double target, double previous, double delta_max);
  static double applyLowPass(double target, double previous, double alpha);
  bool validateMitGains();
  common_msgs::msg::MotorCmd mitSafeCommand(size_t joint) const;
  std::vector<double> gravityTorqueMotor(const std::vector<double>& cmd_targets_deg) const;

  std::vector<JointConfig> joints_;
  std::vector<JointSafetyConfig> safety_;
  std::vector<double> prev_targets_;
  std::vector<double> last_motor_cmd_deg_;
  std::vector<bool> blocked_;
  std::vector<bool> unpowered_; // no feedback at the last seed; recomputed every seed
  std::vector<double> last_gravity_torque_motor_;
  // 0 -> 1 over kGravityFfRampSec after every seed, so feed-forward never steps onto a joint.
  double gravity_ff_ramp_{0.0};
  std::string arm_side_;
  bool have_prev_targets_{false};
  double control_rate_hz_{50.0};
  std::string last_error_;
};
