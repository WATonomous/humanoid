#pragma once

#include <memory>
#include <string>

#include "common_msgs/msg/quest_hand_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quest_teleop/wss_server.hpp" 

class QuestTeleopNode : public rclcpp::Node {
public:
  QuestTeleopNode();
  ~QuestTeleopNode();

private:
  void handle_quest_message(const std::string& json_text);

  rclcpp::Publisher<common_msgs::msg::QuestHandPose>::SharedPtr publisher_;
  std::unique_ptr<WssServer> wss_server_;
};