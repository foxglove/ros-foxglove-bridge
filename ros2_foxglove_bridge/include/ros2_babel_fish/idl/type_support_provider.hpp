// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_DESCRIPTION_PROVIDER_H
#define ROS2_BABEL_FISH_DESCRIPTION_PROVIDER_H

#include <unordered_map>

#include "ros2_babel_fish/idl/type_support.hpp"

namespace ros2_babel_fish {

class TypeSupportProvider {
public:
  using SharedPtr = std::shared_ptr<TypeSupportProvider>;
  using ConstSharedPtr = std::shared_ptr<const TypeSupportProvider>;

  TypeSupportProvider();

  MessageTypeSupport::ConstSharedPtr getMessageTypeSupport(const std::string& type) const;

  ServiceTypeSupport::ConstSharedPtr getServiceTypeSupport(const std::string& type) const;

  ActionTypeSupport::ConstSharedPtr getActionTypeSupport(const std::string& type) const;

protected:
  //! Implementations should call registerMessage if the type support can be cached which is usually
  //! the case.
  virtual MessageTypeSupport::ConstSharedPtr getMessageTypeSupportImpl(
    const std::string& type) const = 0;

  virtual ServiceTypeSupport::ConstSharedPtr getServiceTypeSupportImpl(
    const std::string& type) const = 0;

  virtual ActionTypeSupport::ConstSharedPtr getActionTypeSupportImpl(
    const std::string& type) const = 0;

  MessageTypeSupport::ConstSharedPtr registerMessage(
    const std::string& name, const std::shared_ptr<void>& type_support_library,
    rosidl_message_type_support_t type_support,
    const std::shared_ptr<void>& introspection_type_support_library,
    rosidl_message_type_support_t introspection_type_support) const;

  ServiceTypeSupport::ConstSharedPtr registerService(
    const std::string& name, const std::shared_ptr<void>& type_support_library,
    rosidl_service_type_support_t type_support,
    const std::shared_ptr<void>& introspection_type_support_library,
    rosidl_service_type_support_t introspection_type_support) const;

  ActionTypeSupport::ConstSharedPtr registerAction(
    const std::string& name, ActionTypeSupport::ConstSharedPtr type_support) const;

private:
  mutable std::unordered_map<std::string, MessageTypeSupport::ConstSharedPtr>
    message_type_supports_;
  mutable std::unordered_map<std::string, ServiceTypeSupport::ConstSharedPtr>
    service_type_supports_;
  mutable std::unordered_map<std::string, ActionTypeSupport::ConstSharedPtr> action_type_supports_;
};
}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_DESCRIPTION_PROVIDER_H
