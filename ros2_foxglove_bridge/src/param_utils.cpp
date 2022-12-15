

#include <foxglove_bridge/param_utils.hpp>

namespace foxglove_bridge {

void declareParameters(rclcpp::Node* node) {
  auto portDescription = rcl_interfaces::msg::ParameterDescriptor{};
  portDescription.name = PARAM_PORT;
  portDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  portDescription.description = "The TCP port to bind the WebSocket server to";
  portDescription.read_only = true;
  portDescription.additional_constraints =
    "Must be a valid TCP port number, or 0 to use a random port";
  portDescription.integer_range.resize(1);
  portDescription.integer_range[0].from_value = 0;
  portDescription.integer_range[0].to_value = 65535;
  portDescription.integer_range[0].step = 1;
  node->declare_parameter(PARAM_PORT, DEFAULT_PORT, portDescription);

  auto addressDescription = rcl_interfaces::msg::ParameterDescriptor{};
  addressDescription.name = PARAM_ADDRESS;
  addressDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  addressDescription.description = "The host address to bind the WebSocket server to";
  addressDescription.read_only = true;
  node->declare_parameter(PARAM_ADDRESS, DEFAULT_ADDRESS, addressDescription);

  auto useTlsDescription = rcl_interfaces::msg::ParameterDescriptor{};
  useTlsDescription.name = PARAM_USETLS;
  useTlsDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  useTlsDescription.description = "Use Transport Layer Security for encrypted communication";
  useTlsDescription.read_only = true;
  node->declare_parameter(PARAM_USETLS, false, useTlsDescription);

  auto certfileDescription = rcl_interfaces::msg::ParameterDescriptor{};
  certfileDescription.name = PARAM_CERTFILE;
  certfileDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  certfileDescription.description = "Path to the certificate to use for TLS";
  certfileDescription.read_only = true;
  node->declare_parameter(PARAM_CERTFILE, "", certfileDescription);

  auto keyfileDescription = rcl_interfaces::msg::ParameterDescriptor{};
  keyfileDescription.name = PARAM_KEYFILE;
  keyfileDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  keyfileDescription.description = "Path to the private key to use for TLS";
  keyfileDescription.read_only = true;
  node->declare_parameter(PARAM_KEYFILE, "", keyfileDescription);

  auto maxQosDepthDescription = rcl_interfaces::msg::ParameterDescriptor{};
  maxQosDepthDescription.name = PARAM_MAX_QOS_DEPTH;
  maxQosDepthDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  maxQosDepthDescription.description = "Maximum depth used for the QoS profile of subscriptions.";
  maxQosDepthDescription.read_only = true;
  maxQosDepthDescription.additional_constraints = "Must be a non-negative integer";
  maxQosDepthDescription.integer_range.resize(1);
  maxQosDepthDescription.integer_range[0].from_value = 0;
  maxQosDepthDescription.integer_range[0].to_value = INT32_MAX;
  maxQosDepthDescription.integer_range[0].step = 1;
  node->declare_parameter(PARAM_MAX_QOS_DEPTH, DEFAULT_MAX_QOS_DEPTH, maxQosDepthDescription);

  auto topicWhiteListDescription = rcl_interfaces::msg::ParameterDescriptor{};
  topicWhiteListDescription.name = PARAM_TOPIC_WHITELIST;
  topicWhiteListDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  topicWhiteListDescription.description =
    "List of regular expressions (ECMAScript) of whitelisted topic names.";
  topicWhiteListDescription.read_only = true;
  node->declare_parameter(PARAM_TOPIC_WHITELIST, std::vector<std::string>({".*"}),
                          topicWhiteListDescription);

  auto bestEffortTopicsListDescription = rcl_interfaces::msg::ParameterDescriptor{};
  bestEffortTopicsListDescription.name = PARAM_BEST_EFFORT_TOPICS;
  bestEffortTopicsListDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  bestEffortTopicsListDescription.description =
    "List of regular expressions (ECMAScript) of topic names of which messages can be dropped in "
    "case the client can't keep up with the server's sending rate.";
  bestEffortTopicsListDescription.read_only = true;
  node->declare_parameter(PARAM_BEST_EFFORT_TOPICS, std::vector<std::string>(),
                          bestEffortTopicsListDescription);

  auto bestEffortBuffSizeLimit = rcl_interfaces::msg::ParameterDescriptor{};
  bestEffortBuffSizeLimit.name = PARAM_BEST_EFFORT_BUFF_SIZE_LIMIT;
  bestEffortBuffSizeLimit.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  bestEffortBuffSizeLimit.description =
    "When a connection's send buffer reaches this limit best-effort published topics (or "
    "topics that are treated as such) will be dropped to avoid a queue of old messages building "
    "up.";
  maxQosDepthDescription.integer_range.resize(1);
  maxQosDepthDescription.integer_range[0].from_value = 0;
  maxQosDepthDescription.integer_range[0].to_value = 100000;
  bestEffortBuffSizeLimit.read_only = true;
  node->declare_parameter(PARAM_BEST_EFFORT_BUFF_SIZE_LIMIT, DEFAULT_BEST_EFFORT_BUFF_SIZE_LIMIT_KB,
                          bestEffortBuffSizeLimit);
}

std::vector<std::regex> parseRegexStrings(rclcpp::Node* node,
                                          const std::vector<std::string>& strings) {
  std::vector<std::regex> regexVector;
  regexVector.reserve(strings.size());

  for (const auto& pattern : strings) {
    try {
      regexVector.push_back(
        std::regex(pattern, std::regex_constants::ECMAScript | std::regex_constants::icase));
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(node->get_logger(), "Ignoring invalid regular expression '%s': %s",
                   pattern.c_str(), ex.what());
    }
  }

  return regexVector;
}

}  // namespace foxglove_bridge
