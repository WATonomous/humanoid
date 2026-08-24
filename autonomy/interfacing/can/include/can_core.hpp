#pragma once

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <string>
#include <vector>

struct CanMessage {
  CanMessage() {}
  CanMessage(int size) : data(size, 0), dlc(size) {}
  CanMessage(int id, int size) : id(id), data(size, 0), dlc(size) {}

  uint32_t id;                  // CAN message ID
  std::vector<uint8_t> data;    // Message data -- up to 8 bytes classic, up to 64 if is_fd
  uint8_t dlc;                  // Byte length of data (NOT wire DLC code; see can_core.cpp)
  bool is_extended_id = false;  // Extended frame format flag
  bool is_remote_frame = false; // Remote transmission request flag
  bool is_fd = false;           // CAN-FD frame (up to 64 data bytes). Ignored unless the
                                 // interface was initialized with CanConfig::enable_fd = true.
  bool fd_bitrate_switch = true; // CANFD_BRS -- use data_bitrate for the data phase. Only
                                  // meaningful when is_fd is true.
  uint64_t timestamp_us;        // Timestamp in microseconds
};

struct CanConfig {
  std::string interface_name;  // CAN interface name (e.g., "can0")
  std::string device_path;     // Device path for SLCAN (e.g., "/dev/ttyACM0")
  std::string bustype;         // Bus type: "socketcan" or "slcan"
  uint32_t bitrate;            // Bitrate in bps for arbitration phase
  uint32_t data_bitrate;       // Data bitrate in bps for CAN-FD data phase (informational only
                                // today -- the actual data-phase rate is configured on the
                                // adapter/interface itself, e.g. via `ip link set ... dbitrate`;
                                // this field is not yet wired into that configuration step)
  uint32_t receive_timeout_ms; // Receive timeout in milliseconds
  bool enable_fd = false;      // Opt-in: enable CAN-FD frame support on this interface.
                                // NOT SUPPORTED over bustype="slcan" -- setupSlcan() will
                                // refuse to initialize if this is set, because the Lawicel
                                // SLCAN protocol this repo's setup script speaks cannot carry
                                // FD frames. Requires a CAN-FD-capable adapter/driver
                                // (e.g. gs_usb/candleLight firmware) exposed as a native
                                // SocketCAN interface with bustype="socketcan".
                                //
                                // UNVALIDATED ON REAL HARDWARE as of this change -- bench-test
                                // with real motors before relying on this in any control loop.
                                // See real-hardware-safety skill before testing on the arm.
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
