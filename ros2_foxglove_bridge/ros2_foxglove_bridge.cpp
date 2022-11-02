#define ASIO_STANDALONE

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>

using Server = websocketpp::server<websocketpp::config::asio_tls>;
using ConnectionHdl = websocketpp::connection_hdl;
using SslContext = websocketpp::lib::asio::ssl::context;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

using namespace std::chrono_literals;

class FoxgloveBridge : public rclcpp::Node {
public:
  FoxgloveBridge() : Node("foxglove_bridge") {
    RCLCPP_INFO(this->get_logger(), "Starting %s", this->get_name());
  }

private:
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FoxgloveBridge>());
  rclcpp::shutdown();
  return 0;
}
