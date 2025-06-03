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
        RCLCPP_ERROR(logger_, "Unsupported bus type: %s", config.bustype.c_str());
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
    // TODO: Implement CAN message transmission using SocketCAN
    
    RCLCPP_WARN(logger_, "sendMessage not yet implemented");
    return false;
}

bool CanCore::receiveMessage(CanMessage& message)
{
    // TODO: Implement CAN message reception using SocketCAN
    
    RCLCPP_WARN(logger_, "receiveMessage not yet implemented");
    return false;
}

bool CanCore::setupSocketCan()
{
    RCLCPP_INFO(logger_, "Setting up SocketCAN interface");
    
    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to create SocketCAN socket");
        return false;
    }
    
    // TODO: Implement complete SocketCAN setup
    // Steps to implement:
    // 1. Configure interface address using struct sockaddr_can
    // 2. Use if_nametoindex() to get interface index from config_.interface_name
    // 3. Bind socket to the CAN interface using bind()
    // 4. Set socket options (timeouts, filters, etc.) using setsockopt()
    // 5. Consider setting CAN_RAW_RECV_OWN_MSGS, CAN_RAW_FD_FRAMES if needed
    // 6. Test the connection by attempting a simple operation
    
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
        return false;
    }
    std::string script_path = package_share_directory + "/scripts/setup_can.sh";

    // Check if the script actually exists at this script_path
    struct stat buffer;
    if (stat(script_path.c_str(), &buffer) != 0) {
        RCLCPP_ERROR(logger_, "Setup script not found");
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
        RCLCPP_ERROR(logger_, "Unsupported bitrate for slcan script: %u", config_.bitrate);
        return false;
    }

    std::string command = script_path + " " + config_.device_path + " " + config_.interface_name + " " + bitrate_code;
    
    RCLCPP_INFO(logger_, "Executing CAN setup script: %s", command.c_str());
    int result = std::system(command.c_str());
    
    if (result != 0) {
        RCLCPP_ERROR(logger_, "CAN setup script failed with exit code %d. Check script output and permissions.", result);
        return false;
    }
    RCLCPP_INFO(logger_, "CAN setup script executed successfully.");

    // by now, a new CAN interface should be created based on the name given in the can/config/params.yml
    RCLCPP_INFO(logger_, "Proceeding to configure '%s' as a SocketCAN interface.", config_.interface_name.c_str());
    return setupSocketCan();
}

}
