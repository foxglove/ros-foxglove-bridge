#pragma once

#include <future>
#include <string>

namespace foxglove_bridge {

/**
 * Opens a socket to the service server and retrieves the service type from the connection header.
 * This is necessary as the service type is not stored on the ROS master.
 */
std::future<std::string> retrieveServiceType(const std::string& serviceName);

}  // namespace foxglove_bridge
