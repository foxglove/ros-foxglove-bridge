#pragma once

#include <regex>
#include <string>
#include <vector>

#include <xmlrpcpp/XmlRpc.h>

#include <foxglove_bridge/parameter.hpp>

namespace foxglove_bridge {

foxglove_ws::Parameter fromRosParam(const std::string& name, const XmlRpc::XmlRpcValue& value);
XmlRpc::XmlRpcValue toRosParam(const foxglove_ws::ParameterValue& param);

std::vector<std::regex> parseRegexPatterns(const std::vector<std::string>& strings);

}  // namespace foxglove_bridge
