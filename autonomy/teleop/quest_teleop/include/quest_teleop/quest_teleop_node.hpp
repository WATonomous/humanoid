#pragma once

#include <memory>
#include <string>

#include "quest_teleop/msg/quest_hand_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quest_teleop/wss_server.hpp" 

class QuestTeleopNode : public rclcpp::Node {
public:
  QuestTeleopNode();
  ~QuestTeleopNode();

private:
  void handle_quest_message(const std::string& json_text);

  rclcpp::Publisher<quest_teleop::msg::QuestHandPose>::SharedPtr publisher_;
  std::unique_ptr<WssServer> wss_server_;
};