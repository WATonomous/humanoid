#pragma once

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <string>
#include <vector>

struct CanMessage {
  CanMessage() {}
  CanMessage(int size) : data(size, 0), dlc(size) {}
  CanMessage(int id, int size) : id(id), data(size, 0), dlc(size) {}

  uint32_t id;
  std::vector<uint8_t> data;    // up to 8 bytes, or 64 if is_fd
  uint8_t dlc;
  bool is_extended_id = false;
  bool is_remote_frame = false;
  bool is_fd = false;            // requires CanConfig::enable_fd on the interface
  bool fd_bitrate_switch = true; // CANFD_BRS; only meaningful when is_fd is true
  uint64_t timestamp_us;
};

struct CanConfig {
  std::string interface_name;
  std::string device_path;     // SLCAN serial device (e.g., "/dev/ttyACM0")
  std::string bustype;         // "socketcan" or "slcan"
  uint32_t bitrate;            // arbitration-phase bps
  uint32_t data_bitrate;       // CAN-FD data-phase bps -- not yet wired into interface setup
  uint32_t receive_timeout_ms;
  // Not supported with bustype="slcan" (SLCAN can't carry FD frames -- setupSlcan() refuses).
  // Unvalidated on real hardware; see real-hardware-safety skill before testing on the arm.
  bool enable_fd = false;
};

class CanCore {
public:
  CanCore(const rclcpp::Logger& logger);

  // CAN Interface Management
  bool initialize(const CanConfig& config);
  bool shutdown();
  bool isInitialized() const;

  // Message Transmission
  bool sendMessage(const CanMessage& message);

  // Message Reception
  bool receiveMessage(CanMessage& message);

private:
  rclcpp::Logger logger_;

  // Internal state
  int socket_fd_;
  bool initialized_;
  bool connected_;
  CanConfig config_;

  // Internal helper methods
  bool setupSocketCan();
  bool setupSlcan();
};
