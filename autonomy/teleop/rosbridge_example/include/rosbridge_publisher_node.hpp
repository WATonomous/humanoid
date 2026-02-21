#ifndef ROSBRIDGE_PUBLISHER_NODE_HPP_
#define ROSBRIDGE_PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/vr_hand_pose.hpp"

#include "rosbridge_publisher_core.hpp"

/**
 * Implementation of a ROS2 node that listens to the "unfiltered" and "filtered"
 * topics and echoes the operating frequency of the topic to the console.
 */
class ROSbridgePublisherNode : public rclcpp::Node {
public:
  ROSbridgePublisherNode();

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sample_msgs::msg::VRHandPose>::SharedPtr hand_pose_pub_;

  ROSbridgePublisherCore rosbridge_publisher_;
};

#endif