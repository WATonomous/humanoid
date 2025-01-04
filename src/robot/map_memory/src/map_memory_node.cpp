#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // load ROS2 yaml parameters
  processParameters();

  // Subscribe to local costmap
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_,
    10,
    std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1)
  );

  // Subscribe to odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Publish a global costmap for downstream path planning
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    10
  );

  map_memory_.initMapMemory(
    resolution_, 
    width_, 
    height_, 
    origin_
  );

  RCLCPP_INFO(this->get_logger(), "Initialized Map Memory Core");
}

void MapMemoryNode::processParameters() {
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("local_costmap_topic", "/costmap");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<double>("global_map.resolution", 0.1);
  this->declare_parameter<int>("global_map.width", 100);
  this->declare_parameter<int>("global_map.height", 100);
  this->declare_parameter<double>("global_map.origin.position.x", -5.0);
  this->declare_parameter<double>("global_map.origin.position.y", -5.0);
  this->declare_parameter<double>("global_map.origin.orientation.w", 1.0);
  this->declare_parameter<double>("distance", 1.0);

  // Retrieve parameters and store them in member variables
  local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  resolution_ = this->get_parameter("global_map.resolution").as_double();
  width_ = this->get_parameter("global_map.width").as_int();
  height_ = this->get_parameter("global_map.height").as_int();
  origin_.position.x = this->get_parameter("global_map.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("global_map.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("global_map.origin.orientation.w").as_double();
  distance_ = this->get_parameter("distance").as_double();
}

void MapMemoryNode::localCostmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Check how far the robot has moved since last update
  if (!std::isnan(last_robot_x_))
  {
    double dist = std::hypot(robot_x_ - last_robot_x_, robot_y_ - last_robot_y_);
    if (dist < distance_)
    {
      // Robot hasn’t moved enough, skip updating the global map
      return;
    }
  }

  // Update last position
  last_robot_x_ = robot_x_;
  last_robot_y_ = robot_y_;

  // Update the global map
  map_memory_.updateMap(msg, robot_x_, robot_y_, robot_theta_);

  //6. Publish the updated global map
  nav_msgs::msg::OccupancyGrid map_msg = *map_memory_.getMapData();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "sim_world";
  global_costmap_pub_->publish(map_msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract the robot’s position and orientation from the Odometry message.
  // Assume this odometry is in the "sim_world" frame or a frame equivalent to it.
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Convert quaternion to yaw
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  robot_theta_ = quaternionToYaw(qx, qy, qz, qw);
}

// Utility: Convert quaternion to yaw
double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w)
{
  // Using tf2 to convert to RPY
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
