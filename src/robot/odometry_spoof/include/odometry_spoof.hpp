#ifndef ODOMETRY_SPOOF_NODE_HPP_
#define ODOMETRY_SPOOF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"

class OdometrySpoofNode : public rclcpp::Node {
  public:
    OdometrySpoofNode();

  private:
    void timerCallback();

    // Odom Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Transform Utilities
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Timer to periodically lookup transforms
    rclcpp::TimerBase::SharedPtr timer_;

    // Check if last transform was found
    bool has_last_transform_;

    // Vars to store previous transform
    rclcpp::Time last_time_;
    tf2::Vector3 last_position_;
    tf2::Quaternion last_orientation_;
};

#endif 
