#pragma once

#include <optional>

#include <ros2_babel_fish/babel_fish.hpp>

namespace foxglove_bridge {

/**
 * Convert a JSON-serialized message with a given named schema to a ROS message
 * using ros2_babel_fish. The output message is allocated as a shared pointer
 * and assigned to the outputMessage argument. The return value is an optional
 * exception, which if set indicates that an error occurred during the
 * conversion and `outputMessage` is not valid.
 */
std::optional<std::exception> jsonMessageToRos(
  const std::string_view jsonMessage, const std::string& schemaName,
  ros2_babel_fish::BabelFish::SharedPtr babelFish,
  ros2_babel_fish::CompoundMessage::SharedPtr& outputMessage);

}  // namespace foxglove_bridge
