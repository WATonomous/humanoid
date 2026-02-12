#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TestControllerNode : public rclcpp::Node {
public:
  TestControllerNode() : Node("test_controller") {
    // Create publisher for test controller messages
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("/test_controller", 10);

    // Create timer to publish messages at 1 Hz
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TestControllerNode::publish_test_data, this));

    RCLCPP_INFO(this->get_logger(), "Test Controller Node started - publishing "
                                    "to /test_controller at 1 Hz");
  }

private:
  void publish_test_data() {
    auto message = std_msgs::msg::String();

    // Create a readable test message with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    message.data =
        "Hello from WATonomous! Timestamp: " + std::to_string(time_t) +
        " Status: Active";

    // Publish the message
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestControllerNode>();

  RCLCPP_INFO(node->get_logger(), "Starting Test Controller...");

  try {
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in test controller: %s",
                 e.what());
  }

  rclcpp::shutdown();
  return 0;
}