#ifndef CAN_CORE_HPP
#define CAN_CORE_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cstdint>

namespace autonomy
{

struct CanMessage {
    uint32_t id;                    // CAN message ID
    std::vector<uint8_t> data;      // Message data (up to 8 bytes for classic CAN, up to 64 bytes for CAN-FD)
    bool is_extended_id;            // Extended frame format flag
    bool is_remote_frame;           // Remote transmission request flag
    bool is_fd_frame;               // CAN-FD frame flag
    bool is_brs;                    // Bit Rate Switch (for CAN-FD)
    bool is_esi;                    // Error State Indicator (for CAN-FD)
    uint64_t timestamp_us;          // Timestamp in microseconds
};

struct CanConfig {
    std::string interface_name;     // CAN interface name (e.g., "can0")
    std::string device_path;        // Device path for SLCAN (e.g., "/dev/ttyACM0")
    std::string bustype;            // Bus type: "socketcan" or "slcan"
    uint32_t bitrate;               // Bitrate in bps for arbitration phase
    uint32_t data_bitrate;          // Data bitrate in bps for CAN-FD data phase
    uint32_t receive_timeout_ms;    // Receive timeout in milliseconds
    bool enable_canfd;              // Enable CAN-FD support
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
    bool sendMessage(uint32_t id, const std::vector<uint8_t>& data, bool is_extended_id = false);
    bool sendCanFdMessage(uint32_t id, const std::vector<uint8_t>& data, bool is_extended_id = false, bool is_brs = true);
    
    // Message Reception
    bool receiveMessage(CanMessage& message);
    bool receiveMessage(uint32_t& id, std::vector<uint8_t>& data, bool& is_extended_id);
    
    // Configuration and Status
    bool setBitrate(uint32_t bitrate);
    bool setDataBitrate(uint32_t data_bitrate);
    uint32_t getBitrate() const;
    uint32_t getDataBitrate() const;
    bool isCanFdEnabled() const;
    std::string getInterfaceInfo() const;
    bool isConnected() const;
    
    // Error Handling
    std::string getLastError() const;
    void clearErrors();
    
private:
    rclcpp::Logger logger_;
    
    // Internal state
    int socket_fd_;
    bool initialized_;
    bool connected_;
    CanConfig config_;
    std::string last_error_;
    
    // Internal helper methods
    bool setupSocketCan();
    bool setupSlcan();
    bool configureInterface();
    bool validateMessage(const CanMessage& message);
    void logCanMessage(const CanMessage& message, bool is_tx);
};

}

#endif // CAN_CORE_HPP
