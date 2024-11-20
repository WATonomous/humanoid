#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // load ROS2 yaml parameters
  process_parameters();

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_topic_, 10, 
    std::bind(
      &CostmapNode::laser_scan_callback, this, 
      std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Initialized ROS Constructs");

  costmap_.init_costmap(
    resolution_, 
    width_, 
    height_, 
    origin_,
    inflation_radius_
  );

  RCLCPP_INFO(this->get_logger(), "Initialized Costmap Core");
}

void CostmapNode::process_parameters() {
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("laserscan_topic", "/lidar");
  this->declare_parameter<std::string>("costmap_topic", "/costmap");
  this->declare_parameter<double>("costmap.resolution", 0.1);
  this->declare_parameter<int>("costmap.width", 100);
  this->declare_parameter<int>("costmap.height", 100);
  this->declare_parameter<double>("costmap.origin.position.x", -5.0);
  this->declare_parameter<double>("costmap.origin.position.y", -5.0);
  this->declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  this->declare_parameter<double>("costmap.inflation_radius", 1.0);

  // Retrieve parameters and store them in member variables
  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
}

void CostmapNode::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
  // Update the costmap according to the laser scan
  costmap_.update_costmap(msg);
  // publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.get_costmap_data();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
