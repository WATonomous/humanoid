#include "quest_teleop/quest_teleop_node.hpp"

#include <memory>
#include <string>
#include <unistd.h>

#include "quest_teleop/quest_message_parser.hpp"

QuestTeleopNode::QuestTeleopNode() : Node("quest_teleop_node") {
  publisher_ = create_publisher<common_msgs::msg::QuestHandPose>("/quest_teleop", 1);

  const char* env_cert_dir = std::getenv("TELEOP_CERT_DIR");
  std::string cert_dir = env_cert_dir ? env_cert_dir : "/certs";
  // If /certs does not exist, check default workspace path
  if (!env_cert_dir && access("/certs/cert.pem", F_OK) != 0) {
    if (access("/workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/certs/cert.pem", F_OK) == 0) {
      cert_dir = "/workspace/isaaclab/FallRepo/humanoid/src/teleop/quest_teleop/certs";
    }
  }

  wss_server_ = std::make_unique<WssServer>(
      9090, cert_dir, [this](const std::string& json_text) { handle_quest_message(json_text); });

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
    common_msgs::msg::QuestHandPose msg = QuestMessageParser::parse(json_text);

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