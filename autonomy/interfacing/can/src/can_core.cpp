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
#include <cerrno> // For errno

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
    // Log the details of the received CAN message
    RCLCPP_INFO(logger_, "sendMessage called with CanMessage:");
    RCLCPP_INFO(logger_, "  ID: 0x%X (%s)", message.id, message.is_extended_id ? "Extended" : "Standard");
    RCLCPP_INFO(logger_, "  Timestamp (us): %lu", message.timestamp_us);
    RCLCPP_INFO(logger_, "  Flags: FD=%d, BRS=%d, ESI=%d, RTR=%d",
                message.is_fd_frame, message.is_brs, message.is_esi, message.is_remote_frame);
    RCLCPP_INFO(logger_, "  Data Length: %zu bytes", message.data.size());

    if (!message.data.empty()) {
        std::stringstream ss;
        ss << "  Data: ";
        for (size_t i = 0; i < message.data.size(); ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(message.data[i]) << " ";
        }
        RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    } else {
        RCLCPP_INFO(logger_, "  Data: (empty)");
    }

    // For now, we are just logging and not actually sending.
    return true;
}

bool CanCore::receiveMessage([[maybe_unused]] CanMessage& message)
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

    // Enable CAN FD frames
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        RCLCPP_ERROR(logger_, "Failed to enable CAN FD frames on socket: %s", strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    RCLCPP_INFO(logger_, "CAN FD frames enabled on socket.");

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\\0'; // Ensure null termination

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "Failed to get interface index for '%s': %s", config_.interface_name.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    RCLCPP_INFO(logger_, "Interface index for '%s' is %d.", config_.interface_name.c_str(), ifr.ifr_ifindex);

    // Configure socket address
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind socket
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "Failed to bind SocketCAN socket to interface '%s' (index %d): %s", 
                     config_.interface_name.c_str(), addr.can_ifindex, strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    initialized_ = true;
    connected_ = true; // Assuming bind success means connected for SocketCAN raw
    
    RCLCPP_INFO(logger_, "SocketCAN interface '%s' setup and bound successfully.", config_.interface_name.c_str());
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
