#include "can_core.hpp"
#include <sys/socket.h> // Core socket functions for SocketCAN [socket(), bind(), send(), recv()]
#include <linux/can.h> // Definitions for can frames
#include <linux/can/raw.h> // CAN RAW
#include <net/if.h> 
#include <sys/ioctl.h>
#include <unistd.h> 
#include <cstring>
#include <cstdlib> // For std::system to call the external script for slcand
#include <sys/stat.h> // checking if script files exist 

#include "ament_index_cpp/get_package_share_directory.hpp" // to load the bash scripts 
#include "ament_index_cpp/get_package_prefix.hpp" // defines PackageNotFoundError

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
    return 0;
}

bool CanCore::sendMessage(uint32_t id, const std::vector<uint8_t>& data, bool is_extended_id)
{
    return 0;
}

bool CanCore::receiveMessage(CanMessage& message)
{
    return 0;
}

bool CanCore::receiveMessage(uint32_t& id, std::vector<uint8_t>& data, bool& is_extended_id)
{
    return 0;
}

bool CanCore::setBitrate(uint32_t bitrate)
{
    RCLCPP_INFO(logger_, "Setting bitrate to: %u", bitrate);
    
    config_.bitrate = bitrate;
    
    // TODO: Implement actual bitrate configuration
    
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
    RCLCPP_INFO(logger_, "Setting up SLCAN interface '%s' via external script.", config_.interface_name.c_str());
    RCLCPP_INFO(logger_, "  Device path for script: %s", config_.device_path.c_str());
    RCLCPP_INFO(logger_, "  Bitrate for script: %u", config_.bitrate);

    // Run the slcand daemon to initialize the CAN bus
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("can"); // the package we're in is can
    } catch (const ament_index_cpp::PackageNotFoundError& e) {
        RCLCPP_ERROR(logger_, "Package not found for script path: %s", e.what());
        last_error_ = "Could not find package 'can' to locate setup script.";
        return false;
    }
    std::string script_path = package_share_directory + "/scripts/setup_can.sh";

    // Check if the script actually exists at this script_path
    struct stat buffer;
    if (stat(script_path.c_str(), &buffer) != 0) {
        RCLCPP_ERROR(logger_, "Setup script not found");
        last_error_ = "Setup script not found";
        return false;
    }

    std::string bitrate_code;

    // Map config_.bitrate to the slcan code (e.g., 500000 -> "-s6")
    if (config_.bitrate == 10000) bitrate_code = "-s0";
    else if (config_.bitrate == 20000) bitrate_code = "-s1";
    else if (config_.bitrate == 50000) bitrate_code = "-s2";
    else if (config_.bitrate == 100000) bitrate_code = "-s3";
    else if (config_.bitrate == 125000) bitrate_code = "-s4";
    else if (config_.bitrate == 250000) bitrate_code = "-s5";
    else if (config_.bitrate == 500000) bitrate_code = "-s6";
    else if (config_.bitrate == 750000) bitrate_code = "-s7"; 
    else if (config_.bitrate == 1000000) bitrate_code = "-s8";
    else {
        last_error_ = "Unsupported bitrate for slcan script: " + std::to_string(config_.bitrate);
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }

    std::string command = script_path + " " + config_.device_path + " " + config_.interface_name + " " + bitrate_code;
    
    RCLCPP_INFO(logger_, "Executing CAN setup script: %s", command.c_str());
    int result = std::system(command.c_str());
    
    if (result != 0) {
        last_error_ = "CAN setup script '" + script_path + "' failed with exit code " + std::to_string(result) + ". Check script output and permissions.";
        RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
        return false;
    }
    RCLCPP_INFO(logger_, "CAN setup script executed successfully.");

    // by now, a new CAN interface should be created based on the name given in the can/config/params.yml
    RCLCPP_INFO(logger_, "Proceeding to configure '%s' as a SocketCAN interface.", config_.interface_name.c_str());
    return setupSocketCan();
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
