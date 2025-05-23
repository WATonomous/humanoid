#ifndef CAN_CORE_HPP
#define CAN_CORE_HPP

#include "rclcpp/rclcpp.hpp"

namespace autonomy
{

class CanCore {
  public:
    CanCore(const rclcpp::Logger& logger);
  
  private:
  rclcpp::Logger logger_;
};
}

#endif // CAN_CORE_HPP
