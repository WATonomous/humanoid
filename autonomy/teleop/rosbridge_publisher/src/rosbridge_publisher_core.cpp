#include <chrono>
#include <cmath>

#include "rosbridge_publisher_core.hpp"

ROSbridgePublisherCore::ROSbridgePublisherCore() {}

std::string ROSbridgePublisherCore::hand_pose_to_string(
    const sample_msgs::msg::HandPose pose) {
  std::string s;
  std::vector<float> hand_pose = pose.data;
  s += "[";
  for (int i = 0; i < hand_pose.size(); i++) {
    s += std::to_string(hand_pose[i]);
    s += ",";
  }
  s += "]";
  return s;
}