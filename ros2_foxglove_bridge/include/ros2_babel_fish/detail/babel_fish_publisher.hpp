// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_PUBLISHER_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_PUBLISHER_HPP

#include <rclcpp/publisher_base.hpp>
#include <rclcpp/publisher_options.hpp>
#include <ros2_babel_fish/messages/compound_message.hpp>

#include "ros2_babel_fish/idl/type_support.hpp"

namespace rclcpp {
class SerializedMessage;
}

namespace ros2_babel_fish {

class BabelFishPublisher : public rclcpp::PublisherBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(BabelFishPublisher)

  BabelFishPublisher(rclcpp::node_interfaces::NodeBaseInterface* node_base,
                     const rosidl_message_type_support_t& type_support, const std::string& topic,
                     const rclcpp::QoS& qos, const rclcpp::PublisherOptions& options);

  /// Called post construction, so that construction may continue after shared_from_this() works.
  virtual void post_init_setup(
    rclcpp::node_interfaces::NodeBaseInterface* node_base, const std::string& topic,
    const rclcpp::QoS& qos,
    const rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>& options);

  virtual void publish(std::unique_ptr<CompoundMessage> msg);

  virtual void publish(const CompoundMessage& msg);

  void publish(const rcl_serialized_message_t& serialized_msg);

  void publish(const rclcpp::SerializedMessage& serialized_msg);

private:
  void do_inter_process_publish(const CompoundMessage& msg);

  void do_serialized_publish(const rcl_serialized_message_t* serialized_msg);

  /// Copy of original options passed during construction.
  /**
   * It is important to save a copy of this so that the rmw payload which it
   * may contain is kept alive for the duration of the publisher.
   */
  const rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options_;
};
}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_BABEL_FISH_PUBLISHER_HPP
