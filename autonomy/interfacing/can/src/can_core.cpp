#include "can_core.hpp"
#include <sys/socket.h> // Core socket functions for SocketCAN [socket(), bind(), send(), recv()]
#include <linux/can.h> // Definitions for can frames and CAN FD
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
    if (!initialized_ || socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "CAN interface not initialized or socket not valid. Cannot send message.");
        return false;
    }

    RCLCPP_INFO(logger_, "Sending CAN frame:");
    RCLCPP_INFO(logger_, "  ID: 0x%X (%s)", message.id, message.is_extended_id ? "Extended" : "Standard");
    RCLCPP_INFO(logger_, "  Data Length: %zu bytes", message.data.size());
    
    // Log the actual data bytes
    std::string data_hex = "";
    for (size_t i = 0; i < message.data.size(); ++i) {
        char hex_byte[8];
        snprintf(hex_byte, sizeof(hex_byte), "%02X", message.data[i]);
        data_hex += hex_byte;
        if (i < message.data.size() - 1) data_hex += " ";
    }
    RCLCPP_INFO(logger_, "  Data: [%s]", data_hex.c_str());

    ssize_t bytes_written = -1;
    size_t expected_frame_size = 0;

    // Classic CAN frame only
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = message.id;
    if (message.is_extended_id) {
        frame.can_id |= CAN_EFF_FLAG;
    }
    if (message.is_remote_frame) {
        frame.can_id |= CAN_RTR_FLAG;
    }

    if (message.data.size() > CAN_MAX_DLEN) {
        frame.can_dlc = CAN_MAX_DLEN;
    } else {
        frame.can_dlc = static_cast<__u8>(message.data.size());
    }

    if (!message.is_remote_frame) {
        std::memcpy(frame.data, message.data.data(), frame.can_dlc);
    }

    expected_frame_size = sizeof(struct can_frame);
    bytes_written = write(socket_fd_, &frame, expected_frame_size);

    if (bytes_written < 0) {
        RCLCPP_ERROR(logger_, "Failed to write CAN frame to socket: %s", strerror(errno));
        return false;
    }

    if (static_cast<size_t>(bytes_written) < expected_frame_size) {
        RCLCPP_WARN(logger_, "Could not send full CAN frame. Sent %zd bytes, expected %zu bytes.",
                    bytes_written, expected_frame_size);
        return false;
    }

    RCLCPP_INFO(logger_, "CAN frame sent successfully! (%zd bytes written)", bytes_written);
    return true;
}

bool CanCore::receiveMessage(CanMessage& message)
{
    // TODO: Implement CAN message reception using SocketCAN
    
    RCLCPP_WARN(logger_, "receiveMessage not yet implemented");
    return false;
}

bool CanCore::setupSocketCan()
{
    RCLCPP_INFO(logger_, "Setting up SocketCAN interface: %s", config_.interface_name.c_str());

    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to create SocketCAN socket: %s", strerror(errno));
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // Ensure null termination
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "Failed to get interface index for %s: %s", config_.interface_name.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Bind socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "Failed to bind SocketCAN socket to %s: %s", config_.interface_name.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    initialized_ = true;
    connected_ = true;
    
    RCLCPP_INFO(logger_, "SocketCAN interface %s setup completed successfully (Classic CAN mode).", config_.interface_name.c_str());
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
