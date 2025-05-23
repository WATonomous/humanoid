#include "can_core.hpp"

namespace autonomy
{

CanCore::CanCore(const rclcpp::Logger& logger) : logger_(logger) {
  RCLCPP_INFO(logger_, "CanCore object initialized.");
}

}
