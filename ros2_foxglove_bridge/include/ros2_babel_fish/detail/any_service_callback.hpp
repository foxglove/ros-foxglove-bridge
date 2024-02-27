// Copyright (c) 2022 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_ANY_SERVICE_CALLBACK_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_ANY_SERVICE_CALLBACK_HPP

#include <rclcpp/any_service_callback.hpp>

namespace ros2_babel_fish {
class BabelFishService;
class CompoundMessage;

/**
 * Designed to be compatible with rclcpp::AnyServiceCallback compatible callbacks with equivalent
 * tracing support.
 */
class AnyServiceCallback {
  template <typename CallbackT, typename std::enable_if<
                                  rclcpp::detail::can_be_nullptr<CallbackT>::value, int>::type = 0>
  bool isNullptr(CallbackT&& callback) {
    return !callback;
  }

  template <typename CallbackT, typename std::enable_if<
                                  !rclcpp::detail::can_be_nullptr<CallbackT>::value, int>::type = 0>
  constexpr bool isNullptr(CallbackT&&) {
    return false;
  }

public:
  template <typename CallbackT>
  explicit AnyServiceCallback(CallbackT&& callback) {
    if (isNullptr(callback)) {
      throw std::invalid_argument("Service callback cannot be nullptr!");
    }
    if constexpr (rclcpp::function_traits::same_arguments<CallbackT, SharedPtrCallback>::value) {
      callback_.template emplace<SharedPtrCallback>(callback);
    } else if constexpr (rclcpp::function_traits::same_arguments<
                           CallbackT, SharedPtrWithRequestHeaderCallback>::value) {
      callback_.template emplace<SharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (rclcpp::function_traits::same_arguments<
                           CallbackT, SharedPtrDeferResponseCallback>::value) {
      callback_.template emplace<SharedPtrDeferResponseCallback>(callback);
    } else if constexpr (rclcpp::function_traits::same_arguments<
                           CallbackT, SharedPtrDeferResponseCallbackWithServiceHandle>::value) {
      callback_.template emplace<SharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else {
      static_assert("Invalid callback type passed to AnyServiceCallback!");
    }
  }

  void dispatch(const std::shared_ptr<BabelFishService>& service_handle,
                const std::shared_ptr<rmw_request_id_t>& request_header,
                std::shared_ptr<CompoundMessage> request,
                std::shared_ptr<CompoundMessage> response) {
    TRACEPOINT(callback_start, static_cast<const void*>(this), false);
    if (std::holds_alternative<SharedPtrCallback>(callback_)) {
      (void)request_header;
      const auto& cb = std::get<SharedPtrCallback>(callback_);
      cb(std::move(request), response);
    } else if (std::holds_alternative<SharedPtrWithRequestHeaderCallback>(callback_)) {
      const auto& cb = std::get<SharedPtrWithRequestHeaderCallback>(callback_);
      cb(request_header, std::move(request), response);
    } else if (std::holds_alternative<SharedPtrDeferResponseCallback>(callback_)) {
      const auto& cb = std::get<SharedPtrDeferResponseCallback>(callback_);
      cb(request_header, std::move(request));
    } else if (std::holds_alternative<SharedPtrDeferResponseCallbackWithServiceHandle>(callback_)) {
      const auto& cb = std::get<SharedPtrDeferResponseCallbackWithServiceHandle>(callback_);
      cb(service_handle, request_header, std::move(request));
    }
    TRACEPOINT(callback_end, static_cast<const void*>(this));
  }

  void register_callback_for_tracing() {
#ifndef TRACETOOLS_DISABLED
    std::visit(
      [this](auto&& arg) {
        TRACEPOINT(rclcpp_callback_register, static_cast<const void*>(this),
                   tracetools::get_symbol(arg));
      },
      callback_);
#endif  // TRACETOOLS_DISABLED
  }

private:
  using SharedPtrCallback =
    std::function<void(std::shared_ptr<CompoundMessage>, std::shared_ptr<CompoundMessage>)>;
  using SharedPtrWithRequestHeaderCallback =
    std::function<void(std::shared_ptr<rmw_request_id_t>, std::shared_ptr<CompoundMessage>,
                       std::shared_ptr<CompoundMessage>)>;
  using SharedPtrDeferResponseCallback =
    std::function<void(std::shared_ptr<rmw_request_id_t>, std::shared_ptr<CompoundMessage>)>;
  using SharedPtrDeferResponseCallbackWithServiceHandle =
    std::function<void(std::shared_ptr<BabelFishService>, std::shared_ptr<rmw_request_id_t>,
                       std::shared_ptr<CompoundMessage>)>;

  std::variant<SharedPtrCallback, SharedPtrWithRequestHeaderCallback,
               SharedPtrDeferResponseCallback, SharedPtrDeferResponseCallbackWithServiceHandle>
    callback_;
};

}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_BABEL_FISH_ANY_SERVICE_CALLBACK_HPP
