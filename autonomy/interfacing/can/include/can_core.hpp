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
    std::vector<uint8_t> data;      // Message data (up to 8 bytes for standard CAN)
    bool is_extended_id;            // Extended frame format flag
    bool is_remote_frame;           // Remote transmission request flag
    uint64_t timestamp_us;          // Timestamp in microseconds
};

struct CanConfig {
    std::string interface_name;     // CAN interface name (e.g., "can0")
    std::string device_path;        // Device path for SLCAN (e.g., "/dev/ttyACM0")
    std::string bustype;            // Bus type: "socketcan" or "slcan"
    uint32_t bitrate;               // Bitrate in bps
    uint32_t receive_timeout_ms;    // Receive timeout in milliseconds
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
    
    // Message Reception
    bool receiveMessage(CanMessage& message);
    bool receiveMessage(uint32_t& id, std::vector<uint8_t>& data, bool& is_extended_id);
    
    // Configuration and Status
    bool setBitrate(uint32_t bitrate);
    uint32_t getBitrate() const;
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
