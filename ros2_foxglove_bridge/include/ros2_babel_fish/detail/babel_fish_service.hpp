// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_SERVICE_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_SERVICE_HPP

#include <rclcpp/node.hpp>

#include "ros2_babel_fish/detail/any_service_callback.hpp"
#include "ros2_babel_fish/idl/type_support.hpp"
#include "ros2_babel_fish/messages/compound_message.hpp"

namespace ros2_babel_fish {
namespace impl {
struct BabelFishService {
  using Request = CompoundMessage;
  using Response = CompoundMessage;
};
}  // namespace impl

class BabelFishService : public rclcpp::ServiceBase,
                         std::enable_shared_from_this<BabelFishService> {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(BabelFishService)

  //! Do not call directly, this is private API and might change. Use BabelFish::create_service.
  BabelFishService(std::shared_ptr<rcl_node_t> node, const std::string& service_name,
                   ServiceTypeSupport::ConstSharedPtr type_support, AnyServiceCallback callback,
                   rcl_service_options_t options);

  bool take_request(CompoundMessage& request_out, rmw_request_id_t& request_id_out);

  void send_response(rmw_request_id_t& request_id, CompoundMessage& response);

  std::shared_ptr<void> create_request() override;

  std::shared_ptr<rmw_request_id_t> create_request_header() override;

  void handle_request(std::shared_ptr<rmw_request_id_t> request_header,
                      std::shared_ptr<void> request) override;

private:
  RCLCPP_DISABLE_COPY(BabelFishService)

  ServiceTypeSupport::ConstSharedPtr type_support_;
  AnyServiceCallback callback_;
};
}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_BABEL_FISH_SERVICE_HPP
