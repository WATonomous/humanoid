void CanNode::moveMIT(double velocity, double force, double position, int id) {
  // Create CAN message for MIT (Massachusetts Institute of Technology) control mode
  // This typically involves velocity, force, and position parameters
  
  autonomy::CanMessage can_msg;
  can_msg.id = id;  // MIT control command ID
  can_msg.is_extended_id = true;
  can_msg.is_remote_frame = false;
  can_msg.dlc = 8;  // 8 bytes for the control parameters
  
  // Pack the control parameters into the CAN message data
  // Assuming IEEE 754 double precision format (8 bytes total)
  // We'll use 2 bytes for velocity, 2 for force, 2 for position, 2 reserved
  uint16_t vel_scaled = static_cast<uint16_t>(velocity * 1000.0);  // Scale for precision
  uint16_t force_scaled = static_cast<uint16_t>(force * 1000.0);
  uint16_t pos_scaled = static_cast<uint16_t>(position * 1000.0);
  
  can_msg.data[0] = (vel_scaled >> 8) & 0xFF;      // Velocity high byte
  can_msg.data[1] = vel_scaled & 0xFF;             // Velocity low byte
  can_msg.data[2] = (force_scaled >> 8) & 0xFF;    // Force high byte
  can_msg.data[3] = force_scaled & 0xFF;           // Force low byte
  can_msg.data[4] = (pos_scaled >> 8) & 0xFF;      // Position high byte
  can_msg.data[5] = pos_scaled & 0xFF;             // Position low byte
  can_msg.data[6] = 0x00;                          // Reserved
  can_msg.data[7] = 0x00;                          // Reserved
  
  // Send the CAN message
  if (can_.sendMessage(can_msg)) {
    RCLCPP_INFO(this->get_logger(), "MIT control message sent - V:%.3f, F:%.3f, P:%.3f", 
                velocity, force, position);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send MIT control message");
  }
}

void CanNode::moveProfile(const std::string& profile_type, double max_acc) {
  // Create CAN message for profile-based motion control
  // Profile types could be: "trapezoidal", "s-curve", "polynomial", etc.
  
  autonomy::CanMessage can_msg;
  can_msg.id = 0x201;  // Profile control command ID
  can_msg.is_extended_id = true;
  can_msg.is_remote_frame = false;
  can_msg.dlc = 8;  // 8 bytes for profile parameters
  
  // Encode profile type (first 4 bytes)
  std::hash<std::string> hasher;
  uint32_t profile_hash = static_cast<uint32_t>(hasher(profile_type));
  
  can_msg.data[0] = (profile_hash >> 24) & 0xFF;   // Profile type hash high byte
  can_msg.data[1] = (profile_hash >> 16) & 0xFF;   // Profile type hash mid-high byte
  can_msg.data[2] = (profile_hash >> 8) & 0xFF;    // Profile type hash mid-low byte
  can_msg.data[3] = profile_hash & 0xFF;           // Profile type hash low byte
  
  // Encode maximum acceleration (last 4 bytes)
  uint32_t acc_scaled = static_cast<uint32_t>(max_acc * 1000.0);  // Scale for precision
  
  can_msg.data[4] = (acc_scaled >> 24) & 0xFF;     // Max acc high byte
  can_msg.data[5] = (acc_scaled >> 16) & 0xFF;     // Max acc mid-high byte
  can_msg.data[6] = (acc_scaled >> 8) & 0xFF;      // Max acc mid-low byte
  can_msg.data[7] = acc_scaled & 0xFF;             // Max acc low byte
  
  // Send the CAN message
  if (can_.sendMessage(can_msg)) {
    RCLCPP_INFO(this->get_logger(), "Profile control message sent - Type:%s, MaxAcc:%.3f", 
                profile_type.c_str(), max_acc);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send profile control message");
  }
}