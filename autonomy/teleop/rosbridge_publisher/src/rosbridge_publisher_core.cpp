#include <chrono>
#include <cmath>

#include "rosbridge_publisher_core.hpp"

ROSbridgePublisherCore::ROSbridgePublisherCore() {}

std::string ROSbridgePublisherCore::hand_pose_to_string(
    const sample_msgs::msg::VRHandPose pose){
  std::string s;
  std::vector<float> positions = pose.positions;
  s += "[";
  for (int i = 0; i < positions.size(); i++){
    s += std::to_string(positions[i]);
    s += ",";
  }
  s += "]";
  return s;
}