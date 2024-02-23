// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
#define ROS2_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H

#include "ros2_babel_fish/idl/type_support_provider.hpp"

namespace ros2_babel_fish
{

/**
 * @brief Looks up message libraries that are available locally on the machine.
 */
class LocalTypeSupportProvider : public TypeSupportProvider
{
public:
  LocalTypeSupportProvider();

protected:
  MessageTypeSupport::ConstSharedPtr getMessageTypeSupportImpl( const std::string &type ) const override;

  ServiceTypeSupport::ConstSharedPtr getServiceTypeSupportImpl( const std::string &type ) const override;

  ActionTypeSupport::ConstSharedPtr getActionTypeSupportImpl( const std::string &type ) const override;
};
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
