// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_EXCEPTION_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_EXCEPTION_HPP

#include <rclcpp/exceptions.hpp>

namespace ros2_babel_fish
{

class BabelFishException : public std::runtime_error
{
public:
  explicit BabelFishException( const std::string &msg ) : std::runtime_error( msg ) { }
};
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_BABEL_FISH_EXCEPTION_HPP
