#include "rosbridge_publisher_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

ROSbridgePublisherNode::ROSbridgePublisherNode()
    : Node("rosbridge_publisher"),
      rosbridge_publisher_(ROSbridgePublisherCore()) {

  hand_pose_pub_ =
      this->create_publisher<sample_msgs::msg::HandPose>("teleop", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ROSbridgePublisherNode::timer_callback, this));
}

void ROSbridgePublisherNode::timer_callback() {
  auto message = sample_msgs::msg::HandPose();
  message.data = std::vector<float>(17);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
              rosbridge_publisher_.hand_pose_to_string(message).c_str());
  hand_pose_pub_->publish(message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSbridgePublisherNode>());
  rclcpp::shutdown();
  return 0;
}