// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_ACTION_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_ACTION_HPP

#include <rclcpp/ac.hpp>
#include <ros2_babel_fish/idl/type_support.hpp>
#include <ros2_babel_fish/messages/compound_message.hpp>

namespace ros2_babel_fish
{

class BabelFishService
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS( BabelFishService )

  BabelFishService(
      rclcpp::Node *node, const std::string &name, ServiceTypeSupport::ConstSharedPtr type_support,
      std::function<void( const rmw_request_id_t &, const CompoundMessage &, CompoundMessage & )> callback,
      rcl_service_options_t options );

  rclcpp::ServiceBase::ConstSharedPtr getService() const;

  rclcpp::ServiceBase::SharedPtr getService();

private:
  rclcpp::ServiceBase::SharedPtr service_;
};
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_BABEL_FISH_ACTION_HPP
