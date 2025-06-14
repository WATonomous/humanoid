#include "rosbridge_listener_node.hpp"

#include <string>
#include <algorithm>

ROSbridgeListenerNode::ROSbridgeListenerNode() : Node("rosbridge_listener")
{

  hand_pose_sub_ = this->create_subscription<sample_msgs::msg::HandPose>(
      "teleop", 10, std::bind(&ROSbridgeListenerNode::hand_pose_callback, this, std::placeholders::_1));
}

std::string ROSbridgeListenerNode::hand_pose_to_string(const sample_msgs::msg::HandPose::SharedPtr pose)
{
  std::string s;
  std::vector<float> hand_pose = pose->data;
  s += "[";
  for(int i=0; i < hand_pose.size(); i++){
    s += std::to_string(hand_pose[i]);
    s += ",";
  }
  s += "]";
  return s;
}

void ROSbridgeListenerNode::hand_pose_callback(const sample_msgs::msg::HandPose::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received Hand Data: %s", hand_pose_to_string(msg).c_str());
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSbridgeListenerNode>());
  rclcpp::shutdown();
  return 0;
}