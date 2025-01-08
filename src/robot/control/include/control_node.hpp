#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    // Read and load in ROS2 parameters
    void processParameters();

    // Utility: Convert quaternion to yaw
    double quaternionToYaw(double x, double y, double z, double w);

    // Callback for path
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // Callback for odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Main loop to continuously follow the path
    void followPath();

    // Timer callback
    void timerCallback();

  private:
    robot::ControlCore control_;

    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Path and robot state
    double robot_x_;
    double robot_y_;
    double robot_theta_;

    // ROS2 params
    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    
    int control_period_ms_;
    double lookahead_distance_;
    double steering_gain_;

    double max_steering_angle_;
    double linear_velocity_;
};

#endif
