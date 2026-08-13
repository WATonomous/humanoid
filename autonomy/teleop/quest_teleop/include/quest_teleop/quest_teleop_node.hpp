#pragma once

#include <memory>
#include <string>

#include "common_msgs/msg/quest_hand_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quest_teleop/ws_server.hpp"
#include "quest_teleop/wss_server.hpp"

class QuestTeleopNode : public rclcpp::Node {
public:
  QuestTeleopNode();
  ~QuestTeleopNode();

private:
  void handle_quest_message(const std::string& json_text);

  rclcpp::Publisher<common_msgs::msg::QuestHandPose>::SharedPtr publisher_;
  std::unique_ptr<WssServer> wss_server_;
  // Plain-WS mirror on port 9091, for quest_openxr_app (native app, no TLS needed/implemented
  // client-side -- see ws_server.hpp). Same publisher, same handle_quest_message callback --
  // it's just a second front door into the same pipeline.
  std::unique_ptr<WsServer> ws_server_;
};