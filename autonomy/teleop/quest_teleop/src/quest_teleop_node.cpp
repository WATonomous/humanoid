#include "quest_teleop/quest_teleop_node.hpp"

#include <memory>
#include <string>

#include "quest_teleop/quest_message_parser.hpp"

QuestTeleopNode::QuestTeleopNode() : Node("quest_teleop_node") {
  publisher_ = create_publisher<quest_teleop::msg::QuestHandPose>("/quest_teleop", 1);

  wss_server_ = std::make_unique<WssServer>(
      9090, "/certs", [this](const std::string& json_text) { handle_quest_message(json_text); });

  wss_server_->start();

  RCLCPP_INFO(get_logger(), "quest_teleop_node started");
}

QuestTeleopNode::~QuestTeleopNode() {
  if (wss_server_) {
    wss_server_->stop();
  }
}

void QuestTeleopNode::handle_quest_message(const std::string& json_text) {
  try {
    quest_teleop::msg::QuestHandPose msg = QuestMessageParser::parse(json_text);

    publisher_->publish(msg);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Failed to parse Quest message: %s", e.what());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuestTeleopNode>());
  rclcpp::shutdown();
  return 0;
}