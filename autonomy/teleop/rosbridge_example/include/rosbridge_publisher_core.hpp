#ifndef ROSBRIDGE_PUBLISHER_CORE_HPP_
#define ROSBRIDGE_PUBLISHER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/vr_hand_pose.hpp"

class ROSbridgePublisherCore {
public:
  explicit ROSbridgePublisherCore();

  std::string hand_pose_to_string(const sample_msgs::msg::VRHandPose pose);

private:
};

#endif // AGGREGATOR_CORE_HPP_
