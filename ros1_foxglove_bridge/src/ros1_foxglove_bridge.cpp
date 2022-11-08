#define ASIO_STANDALONE

#include <ros/ros.h>

#include <foxglove_bridge/foxglove_bridge.hpp>

class FoxgloveBridge {
public:
  explicit FoxgloveBridge(ros::NodeHandle& node) {
    (void)node;
    ROS_INFO("Starting %s with %s", ros::this_node::getName().c_str(),
             foxglove::WebSocketUserAgent());
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
