// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_HPP

#include "ros2_babel_fish/detail/babel_fish_action_client.hpp"
#include "ros2_babel_fish/detail/babel_fish_publisher.hpp"
#include "ros2_babel_fish/detail/babel_fish_service.hpp"
#include "ros2_babel_fish/detail/babel_fish_service_client.hpp"
#include "ros2_babel_fish/detail/babel_fish_subscription.hpp"
#include "ros2_babel_fish/idl/type_support_provider.hpp"
#include "ros2_babel_fish/messages/array_message.hpp"
#include "ros2_babel_fish/messages/compound_message.hpp"
#include "ros2_babel_fish/messages/value_message.hpp"

#include <rclcpp/node.hpp>

namespace ros2_babel_fish
{

/*!
 * Allows communication using message types that are not known at compile time.
 */
class BabelFish : public std::enable_shared_from_this<BabelFish>
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS( BabelFish )

  /*!
   * Constructs an instance of BabelFish with a new instance of the default description provider.
   * If you have to use multiple BabelFish instances, it is recommended to share the
   * TypeSupportProvider to prevent multiple look ups of the same message.
   */
  BabelFish();

  explicit BabelFish( std::vector<TypeSupportProvider::SharedPtr> type_support_providers );

  ~BabelFish();

  //! Wrapper for create_subscription without type.
  template<typename CallbackT>
  BabelFishSubscription::SharedPtr
  create_subscription( rclcpp::Node &node, const std::string &topic, const rclcpp::QoS &qos,
                       CallbackT &&callback, rclcpp::CallbackGroup::SharedPtr group = nullptr,
                       rclcpp::SubscriptionOptions options = {},
                       std::chrono::nanoseconds timeout = std::chrono::nanoseconds( -1 ) )
  {
#if RCLCPP_VERSION_MAJOR >= 9
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback;
#else
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback(
        options.get_allocator() );
#endif
    any_callback.set( std::forward<CallbackT>( callback ) );
    return create_subscription( node, topic, qos, any_callback, std::move( group ),
                                std::move( options ), timeout );
  }

  /*!
   * This method will wait for the given topic until timeout expired (if a timeout is set) or the topic becomes available.
   * As soon as the topic is available, it will create a subscription for the topic with the passed options.
   * @param qos The quality of service options. Can be a number which will be the queue size, i.e., number of messages
   *   to queue for processing before dropping messages if the processing can't keep up.
   * @param timeout The maximum time this call will block before returning. Set to 0 to not block at all.
   * @return A subscription if the topic became available before the timeout expired or a nullptr otherwise.
   *
   * @throws BabelFishException If the message type for the given topic could not be loaded or a subscription could not
   *   be created for any other reason.
   */
  BabelFishSubscription::SharedPtr create_subscription(
      rclcpp::Node &node, const std::string &topic, const rclcpp::QoS &qos,
      rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr, rclcpp::SubscriptionOptions options = {},
      std::chrono::nanoseconds timeout = std::chrono::nanoseconds( -1 ) );

  //! Wrapper for create_subscription using type.
  template<typename CallbackT>
  BabelFishSubscription::SharedPtr
  create_subscription( rclcpp::Node &node, const std::string &topic, const std::string &type,
                       const rclcpp::QoS &qos, CallbackT &&callback,
                       rclcpp::CallbackGroup::SharedPtr group = nullptr,
                       rclcpp::SubscriptionOptions options = {} )
  {
#if RCLCPP_VERSION_MAJOR >= 9
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback;
#else
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback(
        options.get_allocator() );
#endif
    any_callback.set( std::forward<CallbackT>( callback ) );
    return create_subscription( node, topic, type, qos, any_callback, std::move( group ),
                                std::move( options ) );
  }

  /*!
   * This method will create a subscription for the given topic using the given message type.
   * Since the message type is provided, it will not wait for the topic to become available.
   * @param type The message type name for the given topic. E.g.: geometry_msgs/msg/Pose
   * @param qos The quality of service options. Can be a number which will be the queue size, i.e., number of messages
   *   to queue for processing before dropping messages if the processing can't keep up.´
   * @return A subscription to the given topic with the given message type.
   *
   * @throws BabelFishException If the given message type could not be loaded or a subscription could not
   *   be created for any other reason.
   */
  BabelFishSubscription::SharedPtr create_subscription(
      rclcpp::Node &node, const std::string &topic, const std::string &type, const rclcpp::QoS &qos,
      rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr, rclcpp::SubscriptionOptions options = {} );

  BabelFishPublisher::SharedPtr create_publisher( rclcpp::Node &node, const std::string &topic,
                                                  const std::string &type, const rclcpp::QoS &qos,
                                                  rclcpp::PublisherOptions options = {} );

  template<typename CallbackT>
  BabelFishService::SharedPtr
  create_service( rclcpp::Node &node, const std::string &service_name, const std::string &type,
                  CallbackT &&callback,
                  const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default,
                  rclcpp::CallbackGroup::SharedPtr group = nullptr )
  {
    AnyServiceCallback any_callback( std::forward<CallbackT>( callback ) );
    return create_service( node, service_name, type, any_callback, qos_profile, group );
  }

  BabelFishService::SharedPtr
  create_service( rclcpp::Node &node, const std::string &service_name, const std::string &type,
                  AnyServiceCallback callback,
                  const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default,
                  rclcpp::CallbackGroup::SharedPtr group = nullptr );

  BabelFishServiceClient::SharedPtr
  create_service_client( rclcpp::Node &node, const std::string &service_name, const std::string &type,
                         const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default,
                         rclcpp::CallbackGroup::SharedPtr group = nullptr );

  BabelFishActionClient::SharedPtr create_action_client(
      rclcpp::Node &node, const std::string &name, const std::string &type,
      const rcl_action_client_options_t &options = rcl_action_client_get_default_options(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr );

  /*!
   * Creates an empty message of the given type.
   * @param type The message type, e.g.: "std_msgs/Header"
   * @return An empty message of the given type
   *
   * @throws BabelFishException If the message description was not found
   */
  CompoundMessage create_message( const std::string &type ) const;

  //! @copydoc create_message
  CompoundMessage::SharedPtr create_message_shared( const std::string &type ) const;

  /*!
   * Creates a service request message for the given service type.
   * @param type The type of the service, e.g., rosapi/GetParam
   * @return An empty service request message that can be used to call a service of the given type
   *
   * @throws BabelFishException If the service description was not found
   */
  CompoundMessage create_service_request( const std::string &type ) const;

  //! @copydoc create_service_request
  CompoundMessage::SharedPtr create_service_request_shared( const std::string &type ) const;

  MessageTypeSupport::ConstSharedPtr get_message_type_support( const std::string &type ) const;

  ServiceTypeSupport::ConstSharedPtr get_service_type_support( const std::string &type ) const;

  ActionTypeSupport::ConstSharedPtr get_action_type_support( const std::string &type ) const;

  std::vector<TypeSupportProvider::SharedPtr> type_support_providers();

private:
  std::vector<TypeSupportProvider::SharedPtr> type_support_providers_;
};
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_BABEL_FISH_HPP
