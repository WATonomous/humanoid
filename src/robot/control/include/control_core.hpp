#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    void initControlCore(
      double lookahead_distance,
      double max_steering_angle, 
      double steering_gain,
      double linear_velocity
    );

    void updatePath(nav_msgs::msg::Path path);

    bool isPathEmpty();

    unsigned int findLookaheadPoint(double robot_x, double robot_y, double robot_theta);

    geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta);
  
  private:
    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
};

} 

#endif 
