#include <chrono>
#include <memory>

#include "odometry_spoof.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

OdometrySpoofNode::OdometrySpoofNode() : Node("odometry_spoof") {
  // Create publisher for Odometry messages
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/filtered", 10);

  // Create a TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a timer to fetch transform & publish odometry at ~10 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&OdometrySpoofNode::timerCallback, this)
  );
}

void OdometrySpoofNode::timerCallback() {
  // We'll look up the transform from sim_world -> robot/chassis/lidar, 
  // note robot frame is usually not the lidar sensor, but we do so to make this
  // assignment easier
  const std::string target_frame = "robot/chassis/lidar";
  const std::string source_frame = "sim_world";

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      source_frame,
      target_frame,
      tf2::TimePointZero  // latest available transform
    );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to %s: %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
    return;
  }

  // Create an Odometry message
  nav_msgs::msg::Odometry odom_msg;

  // Fill header
  odom_msg.header.stamp = transform_stamped.header.stamp;
  odom_msg.header.frame_id = source_frame;  // "world" frame
  odom_msg.child_frame_id  = target_frame;  // "robot" frame

  // Pose from TF
  odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
  odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
  odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
  odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;

  // --- Compute twist from difference in transforms ---
  if (has_last_transform_)
  {
    // Compute time difference
    rclcpp::Time current_time = transform_stamped.header.stamp;
    double dt = (current_time - last_time_).seconds();

    if (dt > 0.0)
    {
      // Linear velocity
      double dx = odom_msg.pose.pose.position.x - last_position_.x();
      double dy = odom_msg.pose.pose.position.y - last_position_.y();
      double dz = odom_msg.pose.pose.position.z - last_position_.z();

      odom_msg.twist.twist.linear.x = dx / dt;
      odom_msg.twist.twist.linear.y = dy / dt;
      odom_msg.twist.twist.linear.z = dz / dt;

      // Angular velocity
      tf2::Quaternion q_last(last_orientation_.x(),
                             last_orientation_.y(),
                             last_orientation_.z(),
                             last_orientation_.w());

      tf2::Quaternion q_current(transform_stamped.transform.rotation.x,
                                transform_stamped.transform.rotation.y,
                                transform_stamped.transform.rotation.z,
                                transform_stamped.transform.rotation.w);

      // Orientation difference: q_diff = q_last.inverse() * q_current
      tf2::Quaternion q_diff = q_last.inverse() * q_current;

      double roll_diff, pitch_diff, yaw_diff;
      tf2::Matrix3x3(q_diff).getRPY(roll_diff, pitch_diff, yaw_diff);

      // Angular velocity (rad/s)
      odom_msg.twist.twist.angular.x = roll_diff  / dt;
      odom_msg.twist.twist.angular.y = pitch_diff / dt;
      odom_msg.twist.twist.angular.z = yaw_diff   / dt;
    }
    else
    {
      // If dt == 0, set velocity to zero
      odom_msg.twist.twist.linear.x  = 0.0;
      odom_msg.twist.twist.linear.y  = 0.0;
      odom_msg.twist.twist.linear.z  = 0.0;
      odom_msg.twist.twist.angular.x = 0.0;
      odom_msg.twist.twist.angular.y = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
    }
  }
  else
  {
    // No previous transform yet; set velocity to zero
    odom_msg.twist.twist.linear.x  = 0.0;
    odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Mark that we've now received at least one transform
    has_last_transform_ = true;
  }

  // Publish odom
  odom_pub_->publish(odom_msg);

  // Store current transform as "last" for next iteration
  last_time_ = transform_stamped.header.stamp;
  last_position_.setValue(
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y,
    odom_msg.pose.pose.position.z
  );
  last_orientation_.setValue(
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySpoofNode>());
  rclcpp::shutdown();
  return 0;
}
