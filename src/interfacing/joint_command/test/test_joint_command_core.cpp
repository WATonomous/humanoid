// Does the moderation pipeline actually enforce what safety_limits.yaml claims?
//
// These run against the SHIPPED config files (CONFIG_DIR), not fixtures, so a future edit that
// turns the position clamp off or raises a velocity past the testing ceiling fails here.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

#include "gravity_model.hpp"
#include "joint_command_core.hpp"

namespace {

constexpr double kRateHz = 50.0;
constexpr int kMit = common_msgs::msg::MotorCmd::MIT_CONTROL;
constexpr int kPositionLoop = common_msgs::msg::MotorCmd::POSITION_LOOP;

std::string configPath(const std::string& name) {
  return std::string(CONFIG_DIR) + "/" + name;
}

common_msgs::msg::ArmPose makePose(double shoulder_pitch, double shoulder_roll, double shoulder_yaw,
                                   double elbow_pitch, double elbow_roll, double wrist_pitch) {
  common_msgs::msg::ArmPose pose;
  pose.shoulder.position = {shoulder_pitch, shoulder_roll, shoulder_yaw};
  pose.elbow.position = {elbow_pitch, elbow_roll};
  pose.wrist.position = {wrist_pitch};
  return pose;
}

// Every joint commanded to the same angle -- enough for limit/velocity behaviour.
common_msgs::msg::ArmPose uniformPose(double deg) {
  return makePose(deg, deg, deg, deg, deg, deg);
}

// A core loaded from the real config files.
class ShippedConfig : public ::testing::Test {
protected:
  void SetUp() override {
    ASSERT_TRUE(core.loadFromYaml(YAML::LoadFile(configPath("hardware_mapping.yaml")), "left"));
    ASSERT_TRUE(core.loadSafetyFromYaml(YAML::LoadFile(configPath("safety_limits.yaml"))["safety"],
                                        kRateHz))
        << core.lastError();
  }

  // Motor-frame feedback that corresponds to a given COMMAND-frame angle, i.e. the inverse
  // of the calibration: motor = direction * (cmd - zero_offset). Feeding motor-frame 0.0
  // would NOT mean "command zero" -- on this arm zero_offset is ~185 deg, so it would place
  // every joint far outside its own limits.
  std::map<int, double> feedbackForCommandFrame(double cmd_deg) {
    std::map<int, double> feedback;
    for (size_t i = 0; i < core.jointCount(); ++i) {
      const double dir = core.joint(i).direction == 0 ? 1.0 : core.joint(i).direction;
      feedback[static_cast<int>(core.motorId(i))] = dir * (cmd_deg - core.joint(i).zero_offset);
    }
    return feedback;
  }

  // Seed every joint at command-frame 0, which is inside every joint's limits.
  void seedAtCommandZero() {
    const SeedReport report = core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));
    ASSERT_TRUE(report.out_of_range.empty())
        << "command-frame 0 should be inside every configured limit";
  }

  JointCommandCore core;
};

} // namespace

// ---------------------------------------------------------------------------
// Position limits
// ---------------------------------------------------------------------------

TEST_F(ShippedConfig, PositionClampIsEnabledInTheShippedConfig) {
  for (size_t i = 0; i < core.jointCount(); ++i) {
    EXPECT_TRUE(core.safety(i).enable_position_clamp)
        << "joint " << core.jointName(i)
        << " ships with the position clamp DISABLED -- limits would not be enforced";
  }
}

TEST_F(ShippedConfig, CommandsNeverLeaveTheConfiguredJointLimits) {
  seedAtCommandZero();
  // Ask for something absurd in both directions and run long enough to get there.
  for (const double target : {+720.0, -720.0}) {
    for (int tick = 0; tick < 2000; ++tick) {
      const auto cmds = core.armPoseToMotorCmds(uniformPose(target), kPositionLoop);
      for (size_t i = 0; i < cmds.size(); ++i) {
        if (!core.joint(i).limit_range) {
          continue;
        }
        const double cmd_frame = core.prevTargets()[i];
        EXPECT_GE(cmd_frame, core.joint(i).lower_limit - 1e-6)
            << core.jointName(i) << " went below its lower limit on tick " << tick;
        EXPECT_LE(cmd_frame, core.joint(i).upper_limit + 1e-6)
            << core.jointName(i) << " went above its upper limit on tick " << tick;
      }
    }
  }
}

TEST_F(ShippedConfig, ClampHoldsAfterSmoothing) {
  // The low-pass runs between the two clamps; the second clamp is what guarantees the final
  // value is inside the limits regardless of what smoothing did.
  seedAtCommandZero();
  for (int tick = 0; tick < 500; ++tick) {
    core.armPoseToMotorCmds(uniformPose(1e6), kPositionLoop);
  }
  for (size_t i = 0; i < core.jointCount(); ++i) {
    if (core.joint(i).limit_range) {
      EXPECT_LE(core.prevTargets()[i], core.joint(i).upper_limit + 1e-6) << core.jointName(i);
    }
  }
}

// ---------------------------------------------------------------------------
// Velocity limits -- the number this whole exercise is meant to pin down
// ---------------------------------------------------------------------------

TEST_F(ShippedConfig, VelocityMaxIsADegreesPerSecondBound) {
  seedAtCommandZero();
  const size_t joint = 0; // shoulder.pitch
  const double velocity_max = core.safety(joint).velocity_max;
  const double per_tick = velocity_max / kRateHz;

  // Target far outside the limits so the velocity limiter -- not the target -- is what binds.
  double previous = core.prevTargets()[joint];
  double travelled = 0.0;
  const int ticks = static_cast<int>(kRateHz); // exactly one second
  for (int tick = 0; tick < ticks; ++tick) {
    core.armPoseToMotorCmds(uniformPose(1e6), kPositionLoop);
    const double now = core.prevTargets()[joint];
    const double step = std::abs(now - previous);
    EXPECT_LE(step, per_tick + 1e-9) << "tick " << tick << " moved " << step
                                     << " deg, more than velocity_max/rate (" << per_tick << ")";
    travelled += step;
    previous = now;
  }
  // It should also actually ACHIEVE the configured speed (the bug being guarded against made
  // the arm crawl at 15% of it), unless a limit stopped it first.
  const double limit_room = core.joint(joint).upper_limit - 0.0;
  const double expected = std::min(velocity_max, limit_room);
  EXPECT_NEAR(travelled, expected, std::max(0.5, per_tick * 2))
      << "one second of streaming moved " << travelled << " deg; velocity_max is " << velocity_max
      << " deg/s";
}

TEST_F(ShippedConfig, SpeedDoesNotDependOnHowOftenArmPoseArrives) {
  // The pipeline advances per TICK. Two cores stepped the same number of ticks must agree,
  // no matter that one "received" many more poses -- that is the property that broke when
  // moderation ran in the ArmPose callback.
  JointCommandCore other;
  ASSERT_TRUE(other.loadFromYaml(YAML::LoadFile(configPath("hardware_mapping.yaml")), "left"));
  ASSERT_TRUE(other.loadSafetyFromYaml(YAML::LoadFile(configPath("safety_limits.yaml"))["safety"],
                                       kRateHz));
  seedAtCommandZero();
  other.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));

  for (int tick = 0; tick < 100; ++tick) {
    core.armPoseToMotorCmds(uniformPose(30.0), kPositionLoop);
    other.armPoseToMotorCmds(uniformPose(30.0), kPositionLoop);
  }
  for (size_t i = 0; i < core.jointCount(); ++i) {
    EXPECT_DOUBLE_EQ(core.prevTargets()[i], other.prevTargets()[i]) << core.jointName(i);
  }
}

TEST_F(ShippedConfig, DeltaLimitBindsWhenItIsTighterThanTheVelocityLimit) {
  YAML::Node cfg = YAML::LoadFile(configPath("safety_limits.yaml"))["safety"];
  cfg["global"]["velocity_max"] = 1000.0; // effectively disable the velocity limiter
  cfg["global"]["delta_max"] = 0.05;
  cfg["global"]["enable_low_pass"] = false;
  for (const auto& group : {"shoulder", "elbow", "wrist"}) {
    for (auto joint : cfg["joints"][group]) {
      joint.second["velocity_max"] = 1000.0;
      joint.second["delta_max"] = 0.05;
    }
  }
  ASSERT_TRUE(core.loadSafetyFromYaml(cfg, kRateHz)) << core.lastError();
  seedAtCommandZero();

  double previous = core.prevTargets()[0];
  for (int tick = 0; tick < 50; ++tick) {
    core.armPoseToMotorCmds(uniformPose(1e6), kPositionLoop);
    const double now = core.prevTargets()[0];
    EXPECT_LE(std::abs(now - previous), 0.05 + 1e-9) << "delta_max not enforced on tick " << tick;
    previous = now;
  }
}

TEST_F(ShippedConfig, ShippedVelocitiesStayUnderTheTestingCeiling) {
  // 2 rad/s = 114.6 deg/s is the per-joint testing limit in the real-hardware-safety skill.
  constexpr double kCeilingDps = 114.59;
  for (size_t i = 0; i < core.jointCount(); ++i) {
    EXPECT_LE(core.safety(i).velocity_max, kCeilingDps)
        << core.jointName(i) << " ships with velocity_max above the 2 rad/s testing ceiling";
    EXPECT_GT(core.safety(i).velocity_max, 0.0) << core.jointName(i);
  }
}

// ---------------------------------------------------------------------------
// Seeding from real feedback
// ---------------------------------------------------------------------------

TEST_F(ShippedConfig, FirstCommandRampsFromTheMeasuredPoseNotFromZero) {
  // The arm is parked at command-frame +2 deg, nowhere near 0 in the motor frame.
  const double parked = 2.0;
  const SeedReport report = core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(parked));
  EXPECT_EQ(report.matched, core.jointCount());
  EXPECT_TRUE(report.out_of_range.empty());

  for (size_t i = 0; i < core.jointCount(); ++i) {
    // The inverse mapping must recover the command-frame pose, including direction = -1.
    EXPECT_NEAR(core.prevTargets()[i], parked, 1e-9)
        << core.jointName(i) << " (direction " << core.joint(i).direction << ", zero_offset "
        << core.joint(i).zero_offset << ")";
  }

  // The first tick must move no further than one velocity step from that seeded pose.
  const std::vector<double> before = core.prevTargets();
  core.armPoseToMotorCmds(uniformPose(0.0), kPositionLoop);
  for (size_t i = 0; i < core.jointCount(); ++i) {
    const double step = std::abs(core.prevTargets()[i] - before[i]);
    EXPECT_LE(step, core.safety(i).velocity_max / kRateHz + 1e-9)
        << core.jointName(i) << " jumped " << step << " deg on its first command";
  }
}

TEST_F(ShippedConfig, MotorsWithoutFeedbackAreReportedNotSilentlyZeroed) {
  std::map<int, double> feedback;
  const double dir = core.joint(0).direction == 0 ? 1.0 : core.joint(0).direction;
  feedback[static_cast<int>(core.motorId(0))] =
      dir * (0.0 - core.joint(0).zero_offset); // only one joint reporting
  const SeedReport report = core.seedPrevTargetsFromFeedback(feedback);
  EXPECT_EQ(report.matched, 1u);
  EXPECT_EQ(report.unmatched.size(), core.jointCount() - 1);
  EXPECT_NE(report.describe().find("without feedback"), std::string::npos);
}

TEST_F(ShippedConfig, UnpoweredJointsAreExcludedUntilTheNextSeed) {
  // One motor powered: the silent ones must not be ramped from an assumed 0 -- a drive powered
  // on mid-session would jump to that stream.
  std::map<int, double> feedback;
  const double dir = core.joint(0).direction == 0 ? 1.0 : core.joint(0).direction;
  feedback[static_cast<int>(core.motorId(0))] = dir * (0.0 - core.joint(0).zero_offset);
  core.seedPrevTargetsFromFeedback(feedback);

  EXPECT_FALSE(core.isBlocked(0));
  for (size_t i = 1; i < core.jointCount(); ++i) {
    EXPECT_TRUE(core.isBlocked(i)) << core.jointName(i);
  }
  const auto cmds = core.armPoseToMotorCmds(uniformPose(10.0), kPositionLoop);
  bool powered_commanded = false;
  for (const auto& cmd : cmds) {
    if (cmd.motor_id == core.motorId(0)) {
      powered_commanded = true;
      continue;
    }
    // Only a silent MIT joint gets anything, and only zero stiffness.
    EXPECT_EQ(cmd.control_type, kMit) << "motor " << static_cast<int>(cmd.motor_id);
    EXPECT_FLOAT_EQ(cmd.kp, 0.0f);
  }
  EXPECT_TRUE(powered_commanded);

  // Re-seeding with every motor reporting lifts the exclusion: it is not sticky.
  core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));
  for (size_t i = 0; i < core.jointCount(); ++i) {
    EXPECT_FALSE(core.isBlocked(i)) << core.jointName(i);
  }
}

TEST_F(ShippedConfig, JointFoundOutsideItsOwnLimitsIsFlaggedAndExcluded) {
  // The wrist's mapping limits are placeholders; a GL40 sitting at 140 deg is exactly the
  // "calibration disagrees with hardware" case that must NOT be silently clamped.
  std::map<int, double> feedback = feedbackForCommandFrame(0.0);
  const size_t wrist = core.jointCount() - 1;
  feedback[static_cast<int>(core.motorId(wrist))] = 140.0; // where the real GL40 actually sits

  const SeedReport report = core.seedPrevTargetsFromFeedback(feedback);
  ASSERT_EQ(report.out_of_range.size(), 1u);
  EXPECT_EQ(report.out_of_range[0], wrist);

  core.blockJoints(report.out_of_range);
  EXPECT_TRUE(core.isBlocked(wrist));

  // A blocked MIT joint is still commanded -- but limp (zero gains), never toward a target.
  const auto cmds = core.armPoseToMotorCmds(uniformPose(0.0), kPositionLoop);
  for (const auto& cmd : cmds) {
    if (cmd.motor_id == core.motorId(wrist)) {
      EXPECT_EQ(cmd.control_type, kMit);
      EXPECT_FLOAT_EQ(cmd.kp, 0.0f);
      EXPECT_FLOAT_EQ(cmd.kd, 0.0f);
    }
  }
}

// ---------------------------------------------------------------------------
// MIT: per-joint control type, gains, and the watchdog
// ---------------------------------------------------------------------------

TEST_F(ShippedConfig, WristShipsAsMitAndTheArmJointsDoNot) {
  const size_t wrist = core.jointCount() - 1;
  EXPECT_TRUE(core.isMitJoint(wrist)) << "the GL40 wrist must run MIT_CONTROL";
  for (size_t i = 0; i < wrist; ++i) {
    EXPECT_FALSE(core.isMitJoint(i)) << core.jointName(i) << " should not be a MIT joint";
  }
  EXPECT_EQ(core.mitMotorIds().size(), 1u);
}

TEST_F(ShippedConfig, MitCommandsCarryRadiansAndTheConfiguredGains) {
  seedAtCommandZero();
  const size_t wrist = core.jointCount() - 1;
  const auto cmds = core.armPoseToMotorCmds(uniformPose(10.0), kPositionLoop);

  bool seen = false;
  for (const auto& cmd : cmds) {
    if (cmd.motor_id != core.motorId(wrist)) {
      EXPECT_EQ(cmd.control_type, kPositionLoop) << "non-MIT joints keep the node default";
      continue;
    }
    seen = true;
    EXPECT_EQ(cmd.control_type, kMit);
    EXPECT_FLOAT_EQ(cmd.kp, static_cast<float>(core.safety(wrist).mit_kp));
    EXPECT_FLOAT_EQ(cmd.kd, static_cast<float>(core.safety(wrist).mit_kd));
    // Position must be radians for MIT (degrees for POSITION_LOOP).
    const double expected_deg = core.lastMotorCmdDeg()[wrist];
    EXPECT_NEAR(cmd.position, expected_deg * M_PI / 180.0, 1e-5);
  }
  EXPECT_TRUE(seen) << "no command emitted for the wrist";
}

TEST_F(ShippedConfig, ShippedMitGainsSatisfyTheStallTorqueRule) {
  for (size_t i = 0; i < core.jointCount(); ++i) {
    if (!core.isMitJoint(i)) {
      continue;
    }
    const auto& s = core.safety(i);
    EXPECT_GT(s.mit_kp, 0.0) << core.jointName(i);
    EXPECT_GT(s.mit_kd, 0.0) << core.jointName(i) << ": kd = 0 makes a GL II oscillate";
    const double worst =
        JointCommandCore::quantiseKp(s.mit_kp) * s.mit_max_track_err * M_PI / 180.0;
    EXPECT_LE(worst, s.mit_max_torque)
        << core.jointName(i) << ": a stalled joint would pull " << worst << " N.m";
  }
}

TEST_F(ShippedConfig, UnsafeMitGainsAreRefusedAtLoadTime) {
  YAML::Node cfg = YAML::LoadFile(configPath("safety_limits.yaml"))["safety"];
  // 1.46 N.m/rad * 12 deg = 0.306 N.m, over the 0.3 ceiling -> must be refused.
  cfg["joints"]["wrist"]["pitch"]["mit_kp"] = 1.46;
  EXPECT_FALSE(core.loadSafetyFromYaml(cfg, kRateHz));
  EXPECT_NE(core.lastError().find("exceeds mit_max_torque"), std::string::npos) << core.lastError();

  // 1.34 (raw 11 = 1.3428) * 12 deg = 0.281 N.m -> allowed.
  cfg["joints"]["wrist"]["pitch"]["mit_kp"] = 1.34;
  EXPECT_TRUE(core.loadSafetyFromYaml(cfg, kRateHz)) << core.lastError();

  // kd = 0 is refused outright, whatever kp is.
  cfg["joints"]["wrist"]["pitch"]["mit_kd"] = 0.0;
  EXPECT_FALSE(core.loadSafetyFromYaml(cfg, kRateHz));
  EXPECT_NE(core.lastError().find("mit_kd"), std::string::npos) << core.lastError();
}

TEST_F(ShippedConfig, GainQuantisationMatchesWhatTheDriveApplies) {
  // 12-bit codes over 0..500 (kp) and 0..5 (kd): the drive can only apply multiples.
  EXPECT_NEAR(JointCommandCore::quantiseKp(1.22), 10 * (500.0 / 4096), 1e-9);
  EXPECT_NEAR(JointCommandCore::quantiseKp(0.61), 5 * (500.0 / 4096), 1e-9);
  EXPECT_NEAR(JointCommandCore::quantiseKd(0.0098), 8 * (5.0 / 4096), 1e-9);
}

TEST_F(ShippedConfig, MitWatchdogCatchesEveryFaultCondition) {
  seedAtCommandZero();
  const size_t wrist = core.jointCount() - 1;
  const int id = static_cast<int>(core.motorId(wrist));
  core.armPoseToMotorCmds(uniformPose(0.0), kPositionLoop);
  const double commanded = core.lastMotorCmdDeg()[wrist];

  MotorFeedbackSample healthy;
  healthy.position_deg = commanded;
  healthy.torque_nm = 0.05;
  healthy.status = 1; // Enable
  healthy.age_s = 0.01;
  EXPECT_FALSE(core.checkMitFaults({{id, healthy}}).has_value());

  auto expectFault = [&](MotorFeedbackSample sample, const std::string& needle) {
    const auto fault = core.checkMitFaults({{id, sample}});
    ASSERT_TRUE(fault.has_value()) << "expected a fault mentioning " << needle;
    EXPECT_NE(fault->find(needle), std::string::npos) << *fault;
  };

  MotorFeedbackSample over_torque = healthy;
  over_torque.torque_nm = 0.45; // above mit_max_torque 0.3
  expectFault(over_torque, "torque");

  MotorFeedbackSample lagging = healthy;
  lagging.position_deg = commanded + 30.0; // above mit_max_track_err 12
  expectFault(lagging, "tracking error");

  MotorFeedbackSample stale = healthy;
  stale.age_s = 0.5; // above mit_feedback_timeout 0.2
  expectFault(stale, "old");

  MotorFeedbackSample faulted = healthy;
  faulted.status = 0xA; // over-current
  expectFault(faulted, "fault status");

  // No feedback at all for a MIT joint is itself a fault.
  const auto missing = core.checkMitFaults({});
  ASSERT_TRUE(missing.has_value());
  EXPECT_NE(missing->find("no MIT feedback"), std::string::npos) << *missing;
}

TEST_F(ShippedConfig, SafeMitCommandsHaveZeroStiffnessSoTheyCannotMoveAnything) {
  const auto safe = core.mitSafeCommands();
  ASSERT_FALSE(safe.empty());
  for (const auto& cmd : safe) {
    EXPECT_EQ(cmd.control_type, kMit);
    EXPECT_FLOAT_EQ(cmd.kp, 0.0f);
    EXPECT_FLOAT_EQ(cmd.kd, 0.0f) << "the shipped GL40 wrist is limp, not damped";
    EXPECT_FLOAT_EQ(cmd.torque, 0.0f);
  }
  const auto enter = core.mitModeCommands(common_msgs::msg::MotorCmd::MIT_ENTER);
  ASSERT_EQ(enter.size(), safe.size());
  EXPECT_EQ(enter[0].control_type, common_msgs::msg::MotorCmd::MIT_ENTER);
}

// ---------------------------------------------------------------------------
// MIT on an AK (gravity-loaded) joint. Elbow roll is switched to MIT in a copy of the shipped
// config -- the shipped file keeps every AK joint on POSITION_LOOP until bench bring-up.
// ---------------------------------------------------------------------------

namespace {

constexpr size_t kElbowRoll = 4;

YAML::Node configWithAkElbowRollOnMit(bool with_fault_kd = true) {
  YAML::Node cfg = YAML::LoadFile(configPath("safety_limits.yaml"))["safety"];
  YAML::Node j = cfg["joints"]["elbow"]["roll"];
  j["control_type"] = 0;
  j["mit_family"] = "ak";
  j["mit_kp"] = 8.0; // quantised 8.06 * 12 deg = 1.69 N.m <= 5
  j["mit_kd"] = 0.4;
  j["mit_max_torque"] = 5.0; // AK80-9 testing ceiling
  if (with_fault_kd) {
    j["mit_fault_kd"] = 0.5;
  }
  return cfg;
}

} // namespace

TEST_F(ShippedConfig, AkMitJointDefaultsToDampAndRequiresAFaultKd) {
  EXPECT_FALSE(core.loadSafetyFromYaml(configWithAkElbowRollOnMit(false), kRateHz));
  EXPECT_NE(core.lastError().find("mit_fault_kd"), std::string::npos) << core.lastError();

  ASSERT_TRUE(core.loadSafetyFromYaml(configWithAkElbowRollOnMit(), kRateHz)) << core.lastError();
  EXPECT_EQ(core.safety(kElbowRoll).mit_fault_action, MitFaultAction::Damp);
  EXPECT_EQ(core.safety(core.jointCount() - 1).mit_fault_action, MitFaultAction::Limp)
      << "the GL40 wrist keeps its limp default";
  EXPECT_TRUE(core.hasDampedMitJoints());
}

TEST_F(ShippedConfig, DampedJointIsHeldWithDampingNotDroppedOrExited) {
  ASSERT_TRUE(core.loadSafetyFromYaml(configWithAkElbowRollOnMit(), kRateHz)) << core.lastError();
  const int elbow = static_cast<int>(core.motorId(kElbowRoll));
  const int wrist = static_cast<int>(core.motorId(core.jointCount() - 1));

  bool saw_elbow = false;
  for (const auto& cmd : core.mitSafeCommands()) {
    EXPECT_FLOAT_EQ(cmd.kp, 0.0f) << "a safe command must never have stiffness";
    if (cmd.motor_id == elbow) {
      saw_elbow = true;
      EXPECT_FLOAT_EQ(cmd.kd, 0.5f);
    } else {
      EXPECT_FLOAT_EQ(cmd.kd, 0.0f);
    }
  }
  EXPECT_TRUE(saw_elbow);

  const auto damped = core.mitSafeCommands(/*damped_only=*/true);
  ASSERT_EQ(damped.size(), 1u);
  EXPECT_EQ(damped[0].motor_id, elbow);

  // The fault path exits only Limp joints: exiting the elbow would cut its damping.
  const auto exits = core.mitModeCommands(common_msgs::msg::MotorCmd::MIT_EXIT, true);
  ASSERT_EQ(exits.size(), 1u);
  EXPECT_EQ(exits[0].motor_id, wrist);
}

TEST_F(ShippedConfig, AkStatusOneIsOverTemperatureNotEnable) {
  ASSERT_TRUE(core.loadSafetyFromYaml(configWithAkElbowRollOnMit(), kRateHz)) << core.lastError();
  seedAtCommandZero();
  core.armPoseToMotorCmds(uniformPose(0.0), kPositionLoop);
  const int elbow = static_cast<int>(core.motorId(kElbowRoll));
  const int wrist = static_cast<int>(core.motorId(core.jointCount() - 1));

  MotorFeedbackSample elbow_fb;
  elbow_fb.position_deg = core.lastMotorCmdDeg()[kElbowRoll];
  elbow_fb.torque_nm = 1.0;
  elbow_fb.status = 0; // AK: no fault
  elbow_fb.age_s = 0.01;
  MotorFeedbackSample wrist_fb;
  wrist_fb.position_deg = core.lastMotorCmdDeg()[core.jointCount() - 1];
  wrist_fb.status = 1; // GL II: Enable
  wrist_fb.age_s = 0.01;
  EXPECT_FALSE(core.checkMitFaults({{elbow, elbow_fb}, {wrist, wrist_fb}}).has_value());

  elbow_fb.status = 1; // AK: motor over-temperature
  const auto fault = core.checkMitFaults({{elbow, elbow_fb}, {wrist, wrist_fb}});
  ASSERT_TRUE(fault.has_value());
  EXPECT_NE(fault->find("elbow.roll"), std::string::npos) << *fault;
  EXPECT_NE(fault->find("AK error code"), std::string::npos) << *fault;
}

TEST_F(ShippedConfig, BlockedAkJointGetsDampingInsteadOfGoingLimp) {
  ASSERT_TRUE(core.loadSafetyFromYaml(configWithAkElbowRollOnMit(), kRateHz)) << core.lastError();
  seedAtCommandZero();
  core.blockJoints({kElbowRoll});
  const int elbow = static_cast<int>(core.motorId(kElbowRoll));
  bool seen = false;
  for (const auto& cmd : core.armPoseToMotorCmds(uniformPose(5.0), kPositionLoop)) {
    if (cmd.motor_id == elbow) {
      seen = true;
      EXPECT_EQ(cmd.control_type, kMit);
      EXPECT_FLOAT_EQ(cmd.kp, 0.0f);
      EXPECT_FLOAT_EQ(cmd.kd, 0.5f);
    }
  }
  EXPECT_TRUE(seen);
}

TEST_F(ShippedConfig, UnknownMitFamilyOrFaultActionIsRefused) {
  YAML::Node cfg = configWithAkElbowRollOnMit();
  cfg["joints"]["elbow"]["roll"]["mit_family"] = "ak80";
  EXPECT_FALSE(core.loadSafetyFromYaml(cfg, kRateHz));
  EXPECT_NE(core.lastError().find("mit_family"), std::string::npos) << core.lastError();

  cfg = configWithAkElbowRollOnMit();
  cfg["joints"]["elbow"]["roll"]["mit_fault_action"] = "brake";
  EXPECT_FALSE(core.loadSafetyFromYaml(cfg, kRateHz));
  EXPECT_NE(core.lastError().find("mit_fault_action"), std::string::npos) << core.lastError();
}

TEST_F(ShippedConfig, RejectsAMalformedArmPose) {
  seedAtCommandZero();
  common_msgs::msg::ArmPose pose;
  pose.shoulder.position = {1.0}; // too few
  EXPECT_THROW(core.armPoseToMotorCmds(pose, kPositionLoop), std::runtime_error);
}

// ---------------------------------------------------------------------------
// Gravity feed-forward
// ---------------------------------------------------------------------------

namespace {

constexpr size_t kShoulderPitch = 0;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

std::array<double, 6> degToRad(const std::array<double, 6>& deg) {
  std::array<double, 6> rad{};
  for (size_t i = 0; i < deg.size(); ++i) {
    rad[i] = deg[i] * kDeg;
  }
  return rad;
}

// Shoulder pitch switched to MIT with gravity feed-forward at full scale.
YAML::Node configWithShoulderPitchFf(double scale = 1.0, double max_torque = 5.5) {
  YAML::Node cfg = YAML::LoadFile(configPath("safety_limits.yaml"))["safety"];
  YAML::Node j = cfg["joints"]["shoulder"]["pitch"];
  j["control_type"] = 0;
  j["gravity_ff_scale"] = scale;
  j["gravity_ff_max_torque"] = max_torque;
  return cfg;
}

} // namespace

// Reference values from an independent numpy evaluation of the same URDF chain.
TEST(GravityModel, MatchesTheUrdfReference) {
  const auto hang = leftArmGravityHoldTorque(degToRad({0, 0, 0, 0, 0, 0}));
  for (const double t : hang) {
    EXPECT_LT(std::abs(t), 0.2) << "arm hanging straight down carries almost no joint load";
  }

  const auto out = leftArmGravityHoldTorque(degToRad({90, 0, 0, 0, 0, 0}));
  EXPECT_NEAR(out[0], 4.794, 0.01) << "2.78 kg arm straight out forward";
  EXPECT_NEAR(out[3], -1.208, 0.01);

  const auto back = leftArmGravityHoldTorque(degToRad({-30, 0, 0, 0, 0, 0}));
  EXPECT_NEAR(back[0], -2.539, 0.01) << "swung back: the holding torque flips sign";

  const std::array<double, 6> expected = {0.592, 1.614, 0.151, 0.254, 0.016, -0.022};
  const auto mixed = leftArmGravityHoldTorque(degToRad({20, 30, -40, 50, 10, -20}));
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(mixed[i], expected[i], 0.01) << "joint " << i;
  }
}

TEST_F(ShippedConfig, ShippedConfigSendsNoGravityFeedForward) {
  seedAtCommandZero();
  for (int tick = 0; tick < 60; ++tick) {
    for (const auto& cmd : core.armPoseToMotorCmds(makePose(30, 0, -30, 30, 0, 0), kPositionLoop)) {
      EXPECT_FLOAT_EQ(cmd.torque, 0.0f) << "motor " << static_cast<int>(cmd.motor_id);
    }
  }
}

TEST_F(ShippedConfig, FeedForwardRampsInAfterSeedingAndMatchesTheModel) {
  ASSERT_TRUE(core.loadSafetyFromYaml(configWithShoulderPitchFf(), kRateHz)) << core.lastError();
  const auto pose = makePose(60, 0, 0, 0, 0, 0);
  std::map<int, double> fb = feedbackForCommandFrame(0.0);
  fb[core.motorId(kShoulderPitch)] =
      core.joint(kShoulderPitch).direction * (60.0 - core.joint(kShoulderPitch).zero_offset);
  ASSERT_TRUE(core.seedPrevTargetsFromFeedback(fb).out_of_range.empty());

  const double full = core.joint(kShoulderPitch).direction * 4.0696; // numpy reference
  const int id = core.motorId(kShoulderPitch);
  auto torqueOf = [&](const std::vector<common_msgs::msg::MotorCmd>& cmds) {
    for (const auto& cmd : cmds) {
      if (cmd.motor_id == id) {
        return static_cast<double>(cmd.torque);
      }
    }
    ADD_FAILURE() << "no command for shoulder pitch";
    return 0.0;
  };

  EXPECT_NEAR(torqueOf(core.armPoseToMotorCmds(pose, kPositionLoop)), full / kRateHz, 1e-3)
      << "first tick after a seed carries 1/50 of the feed-forward, not all of it";
  double t = 0.0;
  for (int tick = 0; tick < 60; ++tick) {
    t = torqueOf(core.armPoseToMotorCmds(pose, kPositionLoop));
  }
  EXPECT_NEAR(t, full, 0.01);
  EXPECT_NEAR(core.lastGravityTorqueMotor()[kShoulderPitch], full, 0.01);

  core.seedPrevTargetsFromFeedback(fb);
  EXPECT_LT(std::abs(torqueOf(core.armPoseToMotorCmds(pose, kPositionLoop))), 0.1)
      << "a re-seed restarts the ramp";
}

TEST_F(ShippedConfig, FeedForwardIsClampedAndFollowsTheUrdfMapping) {
  YAML::Node cfg = configWithShoulderPitchFf(1.0, 1.0);
  const auto pose = makePose(0, 0, 0, 0, 0, 0);
  const int dir = core.joint(kShoulderPitch).direction;
  const int id = core.motorId(kShoulderPitch);
  auto settle = [&]() {
    float torque = 0.0f;
    for (int tick = 0; tick < 60; ++tick) {
      for (const auto& cmd : core.armPoseToMotorCmds(pose, kPositionLoop)) {
        if (cmd.motor_id == id) {
          torque = cmd.torque;
        }
      }
    }
    return torque;
  };

  // cmd 0 -> URDF 90 (arm straight out): 4.79 N.m wanted, clamped to 1.0.
  cfg["joints"]["shoulder"]["pitch"]["urdf_offset_deg"] = 90.0;
  ASSERT_TRUE(core.loadSafetyFromYaml(cfg, kRateHz)) << core.lastError();
  core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));
  EXPECT_FLOAT_EQ(settle(), static_cast<float>(dir * 1.0));

  // Flipping urdf_direction flips the torque the motor is sent.
  cfg["joints"]["shoulder"]["pitch"]["urdf_direction"] = -1;
  ASSERT_TRUE(core.loadSafetyFromYaml(cfg, kRateHz)) << core.lastError();
  core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));
  EXPECT_FLOAT_EQ(settle(), static_cast<float>(-dir * 1.0));
}

TEST_F(ShippedConfig, BlockedJointZeroesAllFeedForward) {
  YAML::Node cfg = configWithShoulderPitchFf();
  cfg["joints"]["shoulder"]["pitch"]["urdf_offset_deg"] = 90.0;
  ASSERT_TRUE(core.loadSafetyFromYaml(cfg, kRateHz)) << core.lastError();
  core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));
  core.blockJoints({kElbowRoll});
  for (int tick = 0; tick < 60; ++tick) {
    for (const auto& cmd : core.armPoseToMotorCmds(uniformPose(0.0), kPositionLoop)) {
      EXPECT_FLOAT_EQ(cmd.torque, 0.0f) << "an unknown joint angle makes every load unknown";
    }
  }
}

TEST_F(ShippedConfig, UnsafeGravityFeedForwardConfigIsRefused) {
  EXPECT_FALSE(core.loadSafetyFromYaml(configWithShoulderPitchFf(1.0, 7.0), kRateHz))
      << "3.14 N.m of PD + 7 N.m of feed-forward exceeds the 10 N.m ceiling";
  EXPECT_NE(core.lastError().find("gravity_ff_max_torque"), std::string::npos) << core.lastError();

  EXPECT_FALSE(core.loadSafetyFromYaml(configWithShoulderPitchFf(1.0, 0.0), kRateHz));

  YAML::Node cfg = YAML::LoadFile(configPath("safety_limits.yaml"))["safety"];
  cfg["joints"]["elbow"]["pitch"]["control_type"] = kPositionLoop;
  cfg["joints"]["elbow"]["pitch"]["gravity_ff_scale"] = 1.0;
  EXPECT_FALSE(core.loadSafetyFromYaml(cfg, kRateHz));
  EXPECT_NE(core.lastError().find("POSITION_LOOP"), std::string::npos) << core.lastError();

  cfg = configWithShoulderPitchFf();
  cfg["joints"]["elbow"]["roll"]["urdf_direction"] = 0;
  EXPECT_FALSE(core.loadSafetyFromYaml(cfg, kRateHz));

  YAML::Node mapping = YAML::LoadFile(configPath("hardware_mapping.yaml"));
  mapping["right"] = mapping["left"];
  ASSERT_TRUE(core.loadFromYaml(mapping, "right"));
  EXPECT_FALSE(core.loadSafetyFromYaml(configWithShoulderPitchFf(), kRateHz))
      << "the model is the left arm only";
}

TEST_F(ShippedConfig, UnpoweredJointUsesItsGravityAssumptionOnlyWhenSet) {
  constexpr size_t kElbowPitch = 3;
  const auto pose = makePose(60, 0, 0, 0, 0, 0);
  const int pitch_id = core.motorId(kShoulderPitch);
  auto seedWithElbowPitchSilent = [&]() {
    std::map<int, double> fb = feedbackForCommandFrame(0.0);
    fb[pitch_id] =
        core.joint(kShoulderPitch).direction * (60.0 - core.joint(kShoulderPitch).zero_offset);
    fb.erase(core.motorId(kElbowPitch));
    const SeedReport report = core.seedPrevTargetsFromFeedback(fb);
    ASSERT_EQ(report.unmatched.size(), 1u);
  };
  auto settledTorque = [&]() {
    double t = 0.0;
    for (int tick = 0; tick < 60; ++tick) {
      for (const auto& cmd : core.armPoseToMotorCmds(pose, kPositionLoop)) {
        if (cmd.motor_id == pitch_id) {
          t = cmd.torque;
        }
      }
    }
    return t;
  };

  ASSERT_TRUE(core.loadSafetyFromYaml(configWithShoulderPitchFf(), kRateHz)) << core.lastError();
  seedWithElbowPitchSilent();
  EXPECT_FLOAT_EQ(settledTorque(), 0.0f) << "unpowered with no assumption: angle unknown";

  YAML::Node cfg = configWithShoulderPitchFf();
  cfg["joints"]["elbow"]["pitch"]["gravity_assume_deg"] = -90.0;
  ASSERT_TRUE(core.loadSafetyFromYaml(cfg, kRateHz)) << core.lastError();
  seedWithElbowPitchSilent();
  EXPECT_NEAR(settledTorque(), core.joint(kShoulderPitch).direction * 3.6219, 0.01)
      << "numpy reference: pitch 60, elbow assumed -90";

  // Out of range is a calibration mismatch, not an unpowered joint: never assumed.
  core.seedPrevTargetsFromFeedback(feedbackForCommandFrame(0.0));
  core.blockJoints({kElbowPitch});
  EXPECT_FLOAT_EQ(settledTorque(), 0.0f);
}
