#define ASIO_STANDALONE

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <foxglove_bridge/foxglove_bridge.hpp>

namespace foxglove_bridge {

class FoxgloveBridge : public nodelet::Nodelet {
public:
  FoxgloveBridge(){};
  virtual ~FoxgloveBridge() {}
  virtual void onInit() {
    ROS_INFO("Starting %s with %s", ros::this_node::getName().c_str(),
             foxglove::WebSocketUserAgent());
  };
};

}  // namespace foxglove_bridge

PLUGINLIB_EXPORT_CLASS(foxglove_bridge::FoxgloveBridge, nodelet::Nodelet)
