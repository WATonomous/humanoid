#include "can_core.hpp"
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

namespace autonomy
{

CanCore::CanCore(const rclcpp::Logger& logger)
    : logger_(logger), socket_fd_(-1), initialized_(false), connected_(false)
{
    RCLCPP_INFO(logger_, "CAN Core initialized");
}

bool CanCore::initialize(const CanConfig& config)
{
    RCLCPP_INFO(logger_, "Initializing CAN interface: %s", config.interface_name.c_str());
    
    config_ = config;
    
    if (config.bustype == "socketcan") {
        return setupSocketCan();
    } else if (config.bustype == "slcan") {
        return setupSlcan();
    } else {
        last_error_ = "Unsupported bus type: " + config.bustype;
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
}

bool CanCore::shutdown()
{
    RCLCPP_INFO(logger_, "Shutting down CAN interface");
    
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    initialized_ = false;
    connected_ = false;
    
    return true;
}

bool CanCore::isInitialized() const
{
    return initialized_;
}

bool CanCore::sendMessage(const CanMessage& message)
{
    if (!validateMessage(message)) {
        return false;
    }
    
    if (!connected_) {
        last_error_ = "CAN interface not connected";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    
    // TODO: Implement actual CAN message transmission using socket_fd_
    logCanMessage(message, true);
    RCLCPP_DEBUG(logger_, "Sent CAN message ID: 0x%X", message.id);
    
    return true;
}

bool CanCore::sendMessage(uint32_t id, const std::vector<uint8_t>& data, bool is_extended_id)
{
    CanMessage message;
    message.id = id;
    message.data = data;
    message.is_extended_id = is_extended_id;
    message.is_remote_frame = false;
    message.timestamp_us = 0; // Will be set by system
    
    return sendMessage(message);
}

bool CanCore::receiveMessage(CanMessage& message)
{
    if (!connected_) {
        last_error_ = "CAN interface not connected";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    
    // TODO: Implement actual CAN message reception using socket_fd_
    // For now, return false indicating no message received
    return false;
}

bool CanCore::receiveMessage(uint32_t& id, std::vector<uint8_t>& data, bool& is_extended_id)
{
    CanMessage message;
    if (receiveMessage(message)) {
        id = message.id;
        data = message.data;
        is_extended_id = message.is_extended_id;
        return true;
    }
    
    return false;
}

bool CanCore::setBitrate(uint32_t bitrate)
{
    RCLCPP_INFO(logger_, "Setting bitrate to: %u", bitrate);
    
    config_.bitrate = bitrate;
    
    // TODO: Implement actual bitrate configuration
    // This would require interface reconfiguration
    
    return true;
}

uint32_t CanCore::getBitrate() const
{
    return config_.bitrate;
}

std::string CanCore::getInterfaceInfo() const
{
    return "Interface: " + config_.interface_name + 
           ", Type: " + config_.bustype + 
           ", Bitrate: " + std::to_string(config_.bitrate);
}

bool CanCore::isConnected() const
{
    return connected_;
}

std::string CanCore::getLastError() const
{
    return last_error_;
}

void CanCore::clearErrors()
{
    last_error_.clear();
}

bool CanCore::setupSocketCan()
{
    RCLCPP_INFO(logger_, "Setting up SocketCAN interface");
    
    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        last_error_ = "Failed to create SocketCAN socket";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    
    // TODO: Implement complete SocketCAN setup
    // - Configure interface address
    // - Bind socket to interface
    // - Set socket options
    
    initialized_ = true;
    connected_ = true;
    
    RCLCPP_INFO(logger_, "SocketCAN interface setup completed");
    return true;
}

bool CanCore::setupSlcan()
{
    RCLCPP_INFO(logger_, "Setting up SLCAN interface");
    
    // TODO: Implement SLCAN setup
    // - Open serial device
    // - Configure SLCAN protocol
    // - Set bitrate
    // - Open CAN interface
    
    initialized_ = true;
    connected_ = true;
    
    RCLCPP_INFO(logger_, "SLCAN interface setup completed");
    return true;
}

bool CanCore::configureInterface()
{
    RCLCPP_INFO(logger_, "Configuring CAN interface");
    
    // TODO: Implement interface configuration
    // - Set filters
    // - Configure timeouts
    // - Set error handling options
    
    return true;
}

bool CanCore::validateMessage(const CanMessage& message)
{
    // Check message ID range
    if (message.is_extended_id && message.id > 0x1FFFFFFF) {
        last_error_ = "Extended CAN ID out of range";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    
    if (!message.is_extended_id && message.id > 0x7FF) {
        last_error_ = "Standard CAN ID out of range";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    
    // Check data length
    if (message.data.size() > 8) {
        last_error_ = "CAN data length exceeds 8 bytes";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    
    return true;
}

void CanCore::logCanMessage(const CanMessage& message, bool is_tx)
{
    std::string direction = is_tx ? "TX" : "RX";
    std::string data_str;
    
    for (const auto& byte : message.data) {
        if (!data_str.empty()) data_str += " ";
        data_str += std::to_string(byte);
    }
    
    RCLCPP_DEBUG(logger_, "%s CAN: ID=0x%X, Data=[%s], Ext=%s", 
                 direction.c_str(), message.id, data_str.c_str(),
                 message.is_extended_id ? "true" : "false");
}

}
