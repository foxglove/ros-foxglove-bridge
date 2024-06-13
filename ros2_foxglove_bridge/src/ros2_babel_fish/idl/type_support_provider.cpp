// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/idl/type_support_provider.hpp"

#include <regex>

#include <rosidl_typesupport_c/type_support_map.h>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros2_babel_fish/messages/message_types.hpp"

namespace ros2_babel_fish {

TypeSupportProvider::TypeSupportProvider() = default;

MessageTypeSupport::ConstSharedPtr TypeSupportProvider::getMessageTypeSupport(
  const std::string& type) const {
  // Check cache
  auto it = message_type_supports_.find(type);
  if (it != message_type_supports_.end()) return it->second;

  return getMessageTypeSupportImpl(type);
}

ServiceTypeSupport::ConstSharedPtr TypeSupportProvider::getServiceTypeSupport(
  const std::string& type) const {
  // Check cache
  auto it = service_type_supports_.find(type);
  if (it != service_type_supports_.end()) return it->second;

  return getServiceTypeSupportImpl(type);
}

ActionTypeSupport::ConstSharedPtr TypeSupportProvider::getActionTypeSupport(
  const std::string& type) const {
  // Check cache
  auto it = action_type_supports_.find(type);
  if (it != action_type_supports_.end()) return it->second;

  return getActionTypeSupportImpl(type);
}

MessageTypeSupport::ConstSharedPtr TypeSupportProvider::registerMessage(
  const std::string& name, const std::shared_ptr<void>& type_support_library,
  rosidl_message_type_support_t type_support,
  const std::shared_ptr<void>& introspection_type_support_library,
  rosidl_message_type_support_t introspection_type_support) const {
  auto result = std::make_shared<MessageTypeSupport>();
  result->name = name;
  result->type_support_library = type_support_library;
  result->type_support_handle = type_support;
  result->introspection_type_support_library = introspection_type_support_library;
  result->introspection_type_support_handle = introspection_type_support;
  message_type_supports_.insert({name, result});
  return result;
}

ServiceTypeSupport::ConstSharedPtr TypeSupportProvider::registerService(
  const std::string& name, const std::shared_ptr<void>& type_support_library,
  rosidl_service_type_support_t type_support,
  const std::shared_ptr<void>& introspection_type_support_library,
  rosidl_service_type_support_t introspection_type_support) const {
  auto result = std::make_shared<ServiceTypeSupport>();
  result->name = name;
  result->type_support_library = type_support_library;
  result->type_support_handle = type_support;
  result->introspection_type_support_library = introspection_type_support_library;
  result->introspection_type_support_handle = introspection_type_support;
  service_type_supports_.insert({name, result});
  return result;
}

ActionTypeSupport::ConstSharedPtr TypeSupportProvider::registerAction(
  const std::string& name, ActionTypeSupport::ConstSharedPtr type_support) const {
  action_type_supports_.insert({name, type_support});
  return type_support;
}
}  // namespace ros2_babel_fish
