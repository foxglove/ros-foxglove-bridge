#define ASIO_STANDALONE

#include <ros/ros.h>
#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>

using Server = websocketpp::server<websocketpp::config::asio_tls>;
using ConnectionHdl = websocketpp::connection_hdl;
using SslContext = websocketpp::lib::asio::ssl::context;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

class FoxgloveBridge {
public:
  explicit FoxgloveBridge(ros::NodeHandle& node) {
    (void)node;
    ROS_INFO("Starting %s", ros::this_node::getName().c_str());
  }

private:
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "foxglove_bridge");

  ros::NodeHandle node;
  FoxgloveBridge bridge(node);

  ROS_INFO("Setup complete.");
  ros::spin();

  return 0;
}
