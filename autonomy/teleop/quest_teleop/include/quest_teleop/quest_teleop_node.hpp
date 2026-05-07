#pragma once

#include <memory>
#include <string>

<<<<<<< HEAD
#include "common_msgs/msg/quest_hand_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quest_teleop/wss_server.hpp"
=======
#include "quest_teleop/msg/quest_hand_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quest_teleop/wss_server.hpp" 
>>>>>>> cbcbea1d (new changes)

class QuestTeleopNode : public rclcpp::Node {
public:
  QuestTeleopNode();
  ~QuestTeleopNode();

private:
  void handle_quest_message(const std::string& json_text);

<<<<<<< HEAD
  rclcpp::Publisher<common_msgs::msg::QuestHandPose>::SharedPtr publisher_;
=======
  rclcpp::Publisher<quest_teleop::msg::QuestHandPose>::SharedPtr publisher_;
>>>>>>> cbcbea1d (new changes)
  std::unique_ptr<WssServer> wss_server_;
};