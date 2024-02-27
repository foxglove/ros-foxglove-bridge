// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/detail/babel_fish_service_client.hpp"

#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros2_babel_fish/idl/serialization.hpp"

namespace ros2_babel_fish {

BabelFishServiceClient::BabelFishServiceClient(
  rclcpp::node_interfaces::NodeBaseInterface* node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  const std::string& service_name, ServiceTypeSupport::ConstSharedPtr type_support,
  rcl_client_options_t client_options)
    : ClientBase(node_base, std::move(node_graph))
    , type_support_(std::move(type_support)) {
  rcl_ret_t ret =
    rcl_client_init(this->get_client_handle().get(), this->get_rcl_node_handle(),
                    &type_support_->type_support_handle, service_name.c_str(), &client_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(service_name, rcl_node_get_name(rcl_node_handle),
                                           rcl_node_get_namespace(rcl_node_handle), true);
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
  }
}

bool BabelFishServiceClient::take_response(CompoundMessage& response_out,
                                           rmw_request_id_t& request_header_out) {
  std::shared_ptr<void> type_erased = create_response();
  if (type_erased == nullptr || !take_type_erased_response(type_erased.get(), request_header_out))
    return false;
  response_out = CompoundMessage(type_support_->response(), std::move(type_erased));
  return true;
}

std::shared_ptr<void> BabelFishServiceClient::create_response() {
  return createContainer(type_support_->response());
}

std::shared_ptr<rmw_request_id_t> BabelFishServiceClient::create_request_header() {
  return std::make_shared<rmw_request_id_t>();
}

void BabelFishServiceClient::handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                                             std::shared_ptr<void> response) {
  std::unique_lock<std::mutex> lock(pending_requests_mutex_);
  const int64_t sequence_number = request_header->sequence_number;
  auto it = pending_requests_.find(sequence_number);
  if (it == pending_requests_.end()) {
    RCUTILS_LOG_ERROR_NAMED("rclcpp", "Received invalid sequence number. Ignoring...");
    return;
  }
  auto& request = it->second;
  auto call_promise = std::get<0>(request);
  auto callback = std::get<1>(request);
  auto future = std::get<2>(request);
  pending_requests_.erase(it);
  // Unlock since the callback might call this recursively
  lock.unlock();

  call_promise->set_value(CompoundMessage::make_shared(type_support_->response(), response));
  callback(future);
}

BabelFishServiceClient::SharedFuture BabelFishServiceClient::async_send_request(
  SharedRequest request) {
  return async_send_request(request, [](SharedFuture) {});
}
}  // namespace ros2_babel_fish
