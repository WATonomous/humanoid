#include "rosbridge_listener_node.hpp"

#include <algorithm>
#include <string>

ROSbridgeListenerNode::ROSbridgeListenerNode() : Node("rosbridge_listener") {

  hand_pose_sub_ = this->create_subscription<sample_msgs::msg::VRHandPose>(
      "teleop", 10,
      std::bind(&ROSbridgeListenerNode::hand_pose_callback, this,
                std::placeholders::_1));
}

std::string ROSbridgeListenerNode::hand_pose_to_string(
    const sample_msgs::msg::VRHandPose::SharedPtr pose) {
  std::string s;
  std::vector<float> positions = pose->positions;
  s += "[";
  for (int i = 0; i < positions.size(); i++) {
    s += std::to_string(positions[i]);
    s += ",";
  }
  s += "]";
  return s;
}

void ROSbridgeListenerNode::hand_pose_callback(
    const sample_msgs::msg::VRHandPose::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received Hand Data: %s",
              hand_pose_to_string(msg).c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSbridgeListenerNode>());
  rclcpp::shutdown();
  return 0;
}