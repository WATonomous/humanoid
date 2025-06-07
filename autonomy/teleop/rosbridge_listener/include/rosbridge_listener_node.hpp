#ifndef ROSBRIDGE_LISTENER_NODE_HPP_
#define ROSBRIDGE_LISTENER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sample_msgs/msg/hand_pose.hpp"

#include "rosbridge_listener_core.hpp"

/**
 * Implementation of a ROS2 node that listens to the "unfiltered" and "filtered"
 * topics and echoes the operating frequency of the topic to the console.
 */
class ROSbridgeListenerNode: public rclcpp::Node {
public:
  
    ROSbridgeListenerNode();

private:

    std::string hand_pose_to_string(const sample_msgs::msg::HandPose::SharedPtr pose);

    void hand_pose_callback(const sample_msgs::msg::HandPose::SharedPtr msg);

    rclcpp::Subscription<sample_msgs::msg::HandPose>::SharedPtr hand_pose_sub_;

};

#endif