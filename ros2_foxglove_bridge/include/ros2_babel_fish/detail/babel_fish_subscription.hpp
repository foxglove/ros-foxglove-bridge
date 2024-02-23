// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_SUBSCRIPTION_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_SUBSCRIPTION_HPP

#include <ros2_babel_fish/messages/compound_message.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>

namespace ros2_babel_fish
{
class BabelFish;

class BabelFishSubscription : public rclcpp::SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS( BabelFishSubscription )

  BabelFishSubscription(
      rclcpp::node_interfaces::NodeBaseInterface *node,
      MessageTypeSupport::ConstSharedPtr type_support, const std::string &topic_name,
      const rclcpp::QoS &qos,
      rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> &options );

  ~BabelFishSubscription() override;

  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

  void handle_message( std::shared_ptr<void> &message,
                       const rclcpp::MessageInfo &message_info ) override;

  void handle_serialized_message( const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
                                  const rclcpp::MessageInfo &message_info ) override;

  void handle_loaned_message( void *loaned_message, const rclcpp::MessageInfo &message_info ) override;

  void return_message( std::shared_ptr<void> &message ) override;

  void return_serialized_message( std::shared_ptr<rclcpp::SerializedMessage> &message ) override;

  bool take( CompoundMessage &message_out, rclcpp::MessageInfo &message_info_out );

  MessageTypeSupport::ConstSharedPtr get_message_type_support() const;

  std::string get_message_type() const;

private:
  RCLCPP_DISABLE_COPY( BabelFishSubscription )

  MessageTypeSupport::ConstSharedPtr type_support_;
  rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback_;
};
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_BABEL_FISH_SUBSCRIPTION_HPP
