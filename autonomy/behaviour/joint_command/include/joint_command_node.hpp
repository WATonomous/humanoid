#pragma once

#include <memory>
#include <vector>

#include "common_msgs/msg/arm_pose.hpp"
#include "common_msgs/msg/motor_cmd.hpp"
#include "joint_command_core.hpp"
#include "rclcpp/rclcpp.hpp"

class JointCommandNode : public rclcpp::Node {
public:
  JointCommandNode();

private:
  void armPoseCallback(const common_msgs::msg::ArmPose::SharedPtr msg);
  void controlTimerCallback();
  void publishMotorCommands(const std::vector<common_msgs::msg::MotorCmd> &cmds);

  JointCommandCore core_;
  rclcpp::Subscription<common_msgs::msg::ArmPose>::SharedPtr arm_pose_sub_;
  rclcpp::Publisher<common_msgs::msg::MotorCmd>::SharedPtr motor_cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  int8_t control_type_{common_msgs::msg::MotorCmd::POSITION_LOOP};
  double control_rate_hz_{50.0};
  std::vector<common_msgs::msg::MotorCmd> latest_cmds_;
  bool have_latest_cmds_{false};
};
