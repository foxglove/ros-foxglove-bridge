#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <foxglove_bridge/foxglove_bridge.hpp>

class FoxgloveBridge : public rclcpp::Node {
public:
  FoxgloveBridge()
      : Node("foxglove_bridge") {
    RCLCPP_INFO(this->get_logger(), "Starting %s with %s", this->get_name(),
                foxglove::WebSocketUserAgent());
  }

private:
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FoxgloveBridge>());
  rclcpp::shutdown();
  return 0;
}
