// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/detail/babel_fish_service.hpp"

#include <utility>

#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros2_babel_fish/idl/serialization.hpp"

namespace ros2_babel_fish {

BabelFishService::BabelFishService(std::shared_ptr<rcl_node_t> node_base,
                                   const std::string& service_name,
                                   ServiceTypeSupport::ConstSharedPtr type_support,
                                   AnyServiceCallback callback, rcl_service_options_t options)
    : ServiceBase(std::move(node_base))
    , type_support_(std::move(type_support))
    , callback_(std::move(callback)) {
  // rcl does the static memory allocation here
  service_handle_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t, [handle = node_handle_](rcl_service_t* service) {
      if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
                     "Error in destruction of rcl service handle: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete service;
    });
  *service_handle_ = rcl_get_zero_initialized_service();

  rcl_ret_t ret =
    rcl_service_init(service_handle_.get(), node_handle_.get(), &type_support_->type_support_handle,
                     service_name.c_str(), &options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto rcl_node_handle = get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(service_name, rcl_node_get_name(rcl_node_handle),
                                           rcl_node_get_namespace(rcl_node_handle), true);
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create service");
  }
  TRACEPOINT(rclcpp_service_callback_added, static_cast<const void*>(get_service_handle().get()),
             static_cast<const void*>(&callback_));
#ifndef TRACETOOLS_DISABLED
  callback_.register_callback_for_tracing();
#endif
}

bool BabelFishService::take_request(CompoundMessage& request_out,
                                    rmw_request_id_t& request_id_out) {
  std::shared_ptr<void> type_erased = create_request();
  if (!take_type_erased_request(type_erased.get(), request_id_out)) return false;
  request_out = CompoundMessage(type_support_->request(), std::move(type_erased));
  return true;
}

void BabelFishService::send_response(rmw_request_id_t& request_id, CompoundMessage& response) {
  rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &request_id,
                                    response.type_erased_message().get());

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
  }
}

std::shared_ptr<void> BabelFishService::create_request() {
  return createContainer(type_support_->request());
}

std::shared_ptr<rmw_request_id_t> BabelFishService::create_request_header() {
  return std::shared_ptr<rmw_request_id_t>();
}

void BabelFishService::handle_request(std::shared_ptr<rmw_request_id_t> request_header,
                                      std::shared_ptr<void> request) {
  auto typed_request = CompoundMessage::make_shared(type_support_->request(), request);
  auto response = CompoundMessage::make_shared(type_support_->response());
  callback_.dispatch(this->shared_from_this(), request_header, typed_request, response);
  send_response(*request_header, *response);
}
}  // namespace ros2_babel_fish
