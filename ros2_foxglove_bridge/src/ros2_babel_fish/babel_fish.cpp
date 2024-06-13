// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/babel_fish.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/wait_set.hpp>

#include "logging.hpp"
#include "ros2_babel_fish/detail/babel_fish_service_client.hpp"
#include "ros2_babel_fish/detail/topic.hpp"
#include "ros2_babel_fish/exceptions/babel_fish_exception.hpp"
#include "ros2_babel_fish/idl/providers/local_type_support_provider.hpp"

namespace ros2_babel_fish {

BabelFish::BabelFish() {
  type_support_providers_.push_back(std::make_shared<LocalTypeSupportProvider>());
}

BabelFish::BabelFish(std::vector<TypeSupportProvider::SharedPtr> type_support_providers)
    : type_support_providers_(std::move(type_support_providers)) {}

BabelFish::~BabelFish() = default;

BabelFishSubscription::SharedPtr BabelFish::create_subscription(
  rclcpp::Node& node, const std::string& topic, const rclcpp::QoS& qos,
  rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
  rclcpp::CallbackGroup::SharedPtr group, rclcpp::SubscriptionOptions options,
  std::chrono::nanoseconds timeout) {
  const std::string& resolved_topic = resolve_topic(node, topic);
  std::vector<std::string> types;
  if (!wait_for_topic_and_type(node, resolved_topic, types, timeout)) return nullptr;
  if (types.empty()) {
    RBF2_ERROR("Could not subscribe to '%s'.Topic is available but has no type!",
               resolved_topic.c_str());
    return nullptr;
  }

  if (types.size() > 1) {
    RBF2_INFO("Topic '%s' has more than one type. Selecting the first arbitrarily: '%s'.",
              resolved_topic.c_str(), types[0].c_str());
  }

  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support(types[0]);
  if (type_support == nullptr) {
    throw BabelFishException("Failed to create a subscriber for type: " + types[0] +
                             ". Type not found!");
  }
  try {
    // TODO Add support for topic statistics?
    auto subscription =
      std::make_shared<BabelFishSubscription>(node.get_node_base_interface().get(), type_support,
                                              topic, qos, std::move(callback), std::move(options));
    node.get_node_topics_interface()->add_subscription(subscription, std::move(group));
    return subscription;
  } catch (const std::runtime_error& ex) {
    throw BabelFishException("Failed to create Subscription: " + std::string(ex.what()));
  }
}

BabelFishSubscription::SharedPtr BabelFish::create_subscription(
  rclcpp::Node& node, const std::string& topic, const std::string& type, const rclcpp::QoS& qos,
  rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
  rclcpp::CallbackGroup::SharedPtr group, rclcpp::SubscriptionOptions options) {
  const std::string& resolved_topic = resolve_topic(node, topic);

  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("Failed to create a subscriber for type: " + type +
                             ". Type not found!");
  }
  try {
    auto subscription =
      std::make_shared<BabelFishSubscription>(node.get_node_base_interface().get(), type_support,
                                              topic, qos, std::move(callback), std::move(options));

    node.get_node_topics_interface()->add_subscription(subscription, std::move(group));
    return subscription;
  } catch (const std::runtime_error& ex) {
    throw BabelFishException("Failed to create Subscription: " + std::string(ex.what()));
  }
}

BabelFishPublisher::SharedPtr BabelFish::create_publisher(rclcpp::Node& node,
                                                          const std::string& topic,
                                                          const std::string& type,
                                                          const rclcpp::QoS& qos,
                                                          rclcpp::PublisherOptions options) {
  // Extract the NodeTopicsInterface from the NodeT.
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics = get_node_topics_interface(node);

  options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Disable;  // Currently not supported by ROS2 for serialized
                                           // messages
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("Failed to create a publisher for type: " + type +
                             ". Type not found!");
  }
  auto result = BabelFishPublisher::make_shared(
    node.get_node_base_interface().get(), type_support->type_support_handle, topic, qos, options);
  result->post_init_setup(node.get_node_base_interface().get(), topic, qos, options);
  // Add the publisher to the node topics interface.
  node_topics->add_publisher(result, options.callback_group);
  return result;
}

BabelFishService::SharedPtr BabelFish::create_service(rclcpp::Node& node,
                                                      const std::string& service_name,
                                                      const std::string& type,
                                                      AnyServiceCallback callback,
                                                      const rmw_qos_profile_t& qos_profile,
                                                      rclcpp::CallbackGroup::SharedPtr group) {
  ServiceTypeSupport::ConstSharedPtr type_support = get_service_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("Failed to create a service for type: " + type + ". Type not found!");
  }
  std::string expanded_name = resolve_topic(node, service_name);
  rcl_service_options_t options = rcl_service_get_default_options();
  options.qos = qos_profile;
  auto result =
    BabelFishService::make_shared(node.get_node_base_interface()->get_shared_rcl_node_handle(),
                                  expanded_name, type_support, std::move(callback), options);
  node.get_node_services_interface()->add_service(result, std::move(group));
  return result;
}

BabelFishServiceClient::SharedPtr BabelFish::create_service_client(
  rclcpp::Node& node, const std::string& service_name, const std::string& type,
  const rmw_qos_profile_t& qos_profile, rclcpp::CallbackGroup::SharedPtr group) {
  ServiceTypeSupport::ConstSharedPtr type_support = get_service_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("Failed to create a service client for type: " + type +
                             ". Type not found!");
  }
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;
  auto result = BabelFishServiceClient::make_shared(node.get_node_base_interface().get(),
                                                    node.get_node_graph_interface(), service_name,
                                                    type_support, options);

  node.get_node_services_interface()->add_client(result, std::move(group));
  return result;
}

BabelFishActionClient::SharedPtr BabelFish::create_action_client(
  rclcpp::Node& node, const std::string& name, const std::string& type,
  const rcl_action_client_options_t& options, rclcpp::CallbackGroup::SharedPtr group) {
  ActionTypeSupport::ConstSharedPtr type_support = get_action_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("Failed to create an action client for type: " + type +
                             ". Type not found!");
  }
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
    node.get_node_waitables_interface();
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = (nullptr == group.get());

  auto deleter = [weak_node, weak_group, group_is_null](BabelFishActionClient* ptr) {
    if (nullptr == ptr) {
      return;
    }
    auto shared_node = weak_node.lock();
    if (shared_node) {
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<BabelFishActionClient> fake_shared_ptr(ptr, [](BabelFishActionClient*) {});

      if (group_is_null) {
        // Was added to default group
        shared_node->remove_waitable(fake_shared_ptr, nullptr);
      } else {
        // Was added to a specific group
        auto shared_group = weak_group.lock();
        if (shared_group) {
          shared_node->remove_waitable(fake_shared_ptr, shared_group);
        }
      }
    }
    delete ptr;
  };

  std::shared_ptr<BabelFishActionClient> action_client(
    new BabelFishActionClient(node.get_node_base_interface(), node.get_node_graph_interface(),
                              node.get_node_logging_interface(), name, type_support, options),
    deleter);

  node.get_node_waitables_interface()->add_waitable(action_client, std::move(group));
  return action_client;
}

CompoundMessage BabelFish::create_message(const std::string& type) const {
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("BabelFish doesn't know a message of type: " + type);
  }
  return CompoundMessage(*type_support);
}

CompoundMessage::SharedPtr BabelFish::create_message_shared(const std::string& type) const {
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("BabelFish doesn't know a message of type: " + type);
  }
  return CompoundMessage::make_shared(*type_support);
}

CompoundMessage BabelFish::create_service_request(const std::string& type) const {
  const ServiceTypeSupport::ConstSharedPtr& type_support = get_service_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("BabelFish doesn't know a service of type: " + type);
  }
  return CompoundMessage(type_support->request());
}

CompoundMessage::SharedPtr BabelFish::create_service_request_shared(const std::string& type) const {
  const ServiceTypeSupport::ConstSharedPtr& type_support = get_service_type_support(type);
  if (type_support == nullptr) {
    throw BabelFishException("BabelFish doesn't know a service of type: " + type);
  }
  return CompoundMessage::make_shared(type_support->request());
}

MessageTypeSupport::ConstSharedPtr BabelFish::get_message_type_support(
  const std::string& type) const {
  for (const auto& provider : type_support_providers_) {
    MessageTypeSupport::ConstSharedPtr result = provider->getMessageTypeSupport(type);
    if (result == nullptr) continue;
    return result;
  }
  return nullptr;
}

ServiceTypeSupport::ConstSharedPtr BabelFish::get_service_type_support(
  const std::string& type) const {
  for (const auto& provider : type_support_providers_) {
    ServiceTypeSupport::ConstSharedPtr result = provider->getServiceTypeSupport(type);
    if (result == nullptr) continue;
    return result;
  }
  return nullptr;
}

ActionTypeSupport::ConstSharedPtr BabelFish::get_action_type_support(
  const std::string& type) const {
  for (const auto& provider : type_support_providers_) {
    ActionTypeSupport::ConstSharedPtr result = provider->getActionTypeSupport(type);
    if (result == nullptr) continue;
    return result;
  }
  return nullptr;
}

std::vector<TypeSupportProvider::SharedPtr> BabelFish::type_support_providers() {
  return type_support_providers_;
}
}  // namespace ros2_babel_fish
