#include "foxglove_bridge/parameter_interface.hpp"

#include <nlohmann/json.hpp>

namespace {

constexpr char PARAM_SEP = '.';

static std::pair<std::string, std::string> getNodeAndParamName(
  const std::string& nodeNameAndParamName) {
  return {nodeNameAndParamName.substr(0UL, nodeNameAndParamName.find(PARAM_SEP)),
          nodeNameAndParamName.substr(nodeNameAndParamName.find(PARAM_SEP) + 1UL)};
}

static std::string prependNodeNameToParamName(const std::string& paramName,
                                              const std::string& nodeName) {
  return nodeName + PARAM_SEP + paramName;
}

static rclcpp::Parameter toRosParam(const foxglove::Parameter& p) {
  using foxglove::Parameter;
  using foxglove::ParameterType;

  const auto paramType = p.getType();
  if (paramType == ParameterType::PARAMETER_BOOL) {
    return rclcpp::Parameter(p.getName(), p.getValue<bool>());
  } else if (paramType == ParameterType::PARAMETER_INTEGER) {
    return rclcpp::Parameter(p.getName(), p.getValue<int64_t>());
  } else if (paramType == ParameterType::PARAMETER_DOUBLE) {
    return rclcpp::Parameter(p.getName(), p.getValue<double>());
  } else if (paramType == ParameterType::PARAMETER_STRING) {
    return rclcpp::Parameter(p.getName(), p.getValue<std::string>());
  } else if (paramType == ParameterType::PARAMETER_BOOL_ARRAY) {
    return rclcpp::Parameter(p.getName(), p.getValue<std::vector<bool>>());
  } else if (paramType == ParameterType::PARAMETER_INTEGER_ARRAY) {
    return rclcpp::Parameter(p.getName(), p.getValue<std::vector<int64_t>>());
  } else if (paramType == ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return rclcpp::Parameter(p.getName(), p.getValue<std::vector<double>>());
  } else if (paramType == ParameterType::PARAMETER_STRING_ARRAY) {
    return rclcpp::Parameter(p.getName(), p.getValue<std::vector<std::string>>());
  } else if (paramType == ParameterType::PARAMETER_NOT_SET) {
    throw std::runtime_error("Unintialized parameter");
  }

  return rclcpp::Parameter();
}

static foxglove::Parameter fromRosParam(const rclcpp::Parameter& p) {
  const auto type = p.get_type();

  if (type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return foxglove::Parameter();
  } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
    return foxglove::Parameter(p.get_name(), p.as_bool());
  } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return foxglove::Parameter(p.get_name(), p.as_int());
  } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return foxglove::Parameter(p.get_name(), p.as_double());
  } else if (type == rclcpp::ParameterType::PARAMETER_STRING) {
    return foxglove::Parameter(p.get_name(), p.as_string());
  } else if (type == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
    return foxglove::Parameter(p.get_name(), p.as_bool_array());
  } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    return foxglove::Parameter(p.get_name(), p.as_integer_array());
  } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return foxglove::Parameter(p.get_name(), p.as_double_array());
  } else if (type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    return foxglove::Parameter(p.get_name(), p.as_string_array());
  } else {
    throw std::runtime_error("Unsupported parameter type");
  }
}

}  // namespace

namespace foxglove_bridge {

ParameterInterface::ParameterInterface(rclcpp::Node* node,
                                       std::vector<std::regex> paramWhitelistPatterns)
    : _node(node)
    , _paramWhitelistPatterns(paramWhitelistPatterns)
    , _callbackGroup(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {}

ParameterList ParameterInterface::getParams(const std::vector<std::string>& paramNames,
                                            const std::chrono::duration<double>& timeout) {
  std::lock_guard<std::mutex> lock(_mutex);

  std::unordered_map<std::string, std::vector<std::string>> paramNamesByNodeName;

  if (!paramNames.empty()) {
    // Break apart fully qualified {node_name}.{param_name} strings and build a
    // mape of node names to the list of parameters for each node
    for (const auto& fullParamName : paramNames) {
      const auto& [nodeName, paramName] = getNodeAndParamName(fullParamName);
      paramNamesByNodeName[nodeName].push_back(paramName);
    }

    RCLCPP_INFO(_node->get_logger(), "Getting %zu parameters from %zu nodes...", paramNames.size(),
                paramNamesByNodeName.size());
  } else {
    // Make a map of node names to empty parameter lists
    for (const auto& nodeName : _node->get_node_names()) {
      paramNamesByNodeName.insert({nodeName, {}});
    }

    RCLCPP_INFO(_node->get_logger(), "Getting all parameters from %zu nodes...",
                paramNamesByNodeName.size());
  }

  std::vector<std::future<ParameterList>> getParametersFuture;
  for (const auto& [nodeName, nodeParamNames] : paramNamesByNodeName) {
    const auto this_name = _node->get_fully_qualified_name();

    if (nodeName == this_name) {
      continue;
    }

    auto [paramClientIt, wasNewlyCreated] = _paramClientsByNode.try_emplace(
      nodeName, rclcpp::AsyncParametersClient::make_shared(
                  _node, nodeName, rmw_qos_profile_parameters, _callbackGroup));

    getParametersFuture.emplace_back(
      std::async(std::launch::async, &ParameterInterface::getNodeParameters, this,
                 paramClientIt->second, nodeName, nodeParamNames, timeout));
  }

  ParameterList result;
  for (auto& future : getParametersFuture) {
    try {
      const auto params = future.get();
      result.insert(result.begin(), params.begin(), params.end());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(_node->get_logger(), "Exception when getting parameters: %s", e.what());
    }
  }

  return result;
}

void ParameterInterface::setParams(const ParameterList& parameters,
                                   const std::chrono::duration<double>& timeout) {
  std::lock_guard<std::mutex> lock(_mutex);

  rclcpp::ParameterMap paramsByNode;
  for (const auto& param : parameters) {
    if (!isWhitelistedParam(param.getName())) {
      return;
    }

    const auto rosParam = toRosParam(param);
    const auto& [nodeName, paramName] = getNodeAndParamName(rosParam.get_name());
    paramsByNode[nodeName].emplace_back(paramName, rosParam.get_parameter_value());
  }

  std::vector<std::future<void>> setParametersFuture;
  for (const auto& [nodeName, params] : paramsByNode) {
    auto [paramClientIt, wasNewlyCreated] = _paramClientsByNode.try_emplace(
      nodeName, rclcpp::AsyncParametersClient::make_shared(
                  _node, nodeName, rmw_qos_profile_parameters, _callbackGroup));

    setParametersFuture.emplace_back(std::async(std::launch::async,
                                                &ParameterInterface::setNodeParameters, this,
                                                paramClientIt->second, nodeName, params, timeout));
  }

  for (auto& future : setParametersFuture) {
    try {
      future.get();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(_node->get_logger(), "Exception when setting parameters: %s", e.what());
    }
  }
}

void ParameterInterface::subscribeParams(const std::vector<std::string>& paramNames) {
  std::lock_guard<std::mutex> lock(_mutex);

  std::unordered_set<std::string> nodesToSubscribe;
  for (const auto& paramName : paramNames) {
    if (!isWhitelistedParam(paramName)) {
      return;
    }

    const auto& [nodeName, paramN] = getNodeAndParamName(paramName);
    auto [subscribedParamsit, wasNewlyCreated] = _subscribedParamsByNode.try_emplace(nodeName);

    auto& subscribedNodeParams = subscribedParamsit->second;
    subscribedNodeParams.insert(paramN);

    if (wasNewlyCreated) {
      nodesToSubscribe.insert(nodeName);
    }
  }

  for (const auto& nodeName : nodesToSubscribe) {
    auto [paramClientIt, wasNewlyCreated] = _paramClientsByNode.try_emplace(
      nodeName, rclcpp::AsyncParametersClient::make_shared(
                  _node, nodeName, rmw_qos_profile_parameters, _callbackGroup));
    auto& paramClient = paramClientIt->second;

    _paramSubscriptionsByNode[nodeName] = paramClient->on_parameter_event(
      [this, nodeName](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr msg) {
        RCLCPP_DEBUG(_node->get_logger(), "Retrieved param update for node %s: %zu params changed",
                     nodeName.c_str(), msg->changed_parameters.size());

        ParameterList result;
        const auto& subscribedNodeParams = _subscribedParamsByNode[nodeName];
        for (const auto& param : msg->changed_parameters) {
          if (subscribedNodeParams.find(param.name) != subscribedNodeParams.end()) {
            result.push_back(fromRosParam(
              rclcpp::Parameter(prependNodeNameToParamName(param.name, nodeName), param.value)));
          }
        }

        if (!result.empty() && _paramUpdateFunc) {
          _paramUpdateFunc(result);
        }
      });
  }
}

void ParameterInterface::unsubscribeParams(const std::vector<std::string>& paramNames) {
  std::lock_guard<std::mutex> lock(_mutex);

  for (const auto& paramName : paramNames) {
    const auto& [nodeName, paramN] = getNodeAndParamName(paramName);

    const auto subscribedNodeParamsIt = _subscribedParamsByNode.find(nodeName);
    if (subscribedNodeParamsIt != _subscribedParamsByNode.end()) {
      subscribedNodeParamsIt->second.erase(subscribedNodeParamsIt->second.find(paramN));

      if (subscribedNodeParamsIt->second.empty()) {
        _subscribedParamsByNode.erase(subscribedNodeParamsIt);
        _paramSubscriptionsByNode.erase(_paramSubscriptionsByNode.find(nodeName));
      }
    }
  }
}

void ParameterInterface::setParamUpdateCallback(ParamUpdateFunc paramUpdateFunc) {
  std::lock_guard<std::mutex> lock(_mutex);
  _paramUpdateFunc = paramUpdateFunc;
}

ParameterList ParameterInterface::getNodeParameters(
  const rclcpp::AsyncParametersClient::SharedPtr paramClient, const std::string& nodeName,
  const std::vector<std::string>& paramNames, const std::chrono::duration<double>& timeout) {
  if (!paramClient->service_is_ready()) {
    throw std::runtime_error("Parameter service for node '" + nodeName + "' is not ready");
  }

  auto paramsToRequest = paramNames;
  if (paramsToRequest.empty()) {
    // `paramNames` is empty, list all parameter names for this node
    auto future = paramClient->list_parameters({}, 0UL);
    if (std::future_status::ready != future.wait_for(timeout)) {
      throw std::runtime_error("Failed to retrieve parameter names for node '" + nodeName + "'");
    }
    paramsToRequest = future.get().names;
  }

  // Start parameter fetches and wait for them to complete
  auto getParamsFuture = paramClient->get_parameters(paramsToRequest);
  if (std::future_status::ready != getParamsFuture.wait_for(timeout)) {
    throw std::runtime_error("Timed out waiting for " + std::to_string(paramsToRequest.size()) +
                             " parameter(s) from node '" + nodeName + "'");
  }
  const auto params = getParamsFuture.get();

  ParameterList result;
  for (const auto& param : params) {
    const auto fullParamName = prependNodeNameToParamName(param.get_name(), nodeName);
    if (isWhitelistedParam(fullParamName)) {
      result.push_back(fromRosParam(rclcpp::Parameter(fullParamName, param.get_parameter_value())));
    }
  }
  return result;
}

void ParameterInterface::setNodeParameters(rclcpp::AsyncParametersClient::SharedPtr paramClient,
                                           const std::string& nodeName,
                                           const std::vector<rclcpp::Parameter>& params,
                                           const std::chrono::duration<double>& timeout) {
  if (!paramClient->service_is_ready()) {
    throw std::runtime_error("Parameter service for node '" + nodeName + "' is not ready");
  }

  auto future = paramClient->set_parameters(params);
  if (std::future_status::ready != future.wait_for(timeout)) {
    throw std::runtime_error("Param client failed to set " + std::to_string(params.size()) +
                             " parameter(s) for node '" + nodeName + "' within the given timeout");
  }

  const auto setParamResults = future.get();
  for (auto& result : setParamResults) {
    if (!result.successful) {
      RCLCPP_WARN(_node->get_logger(), "Failed to set one or more parameters for node '%s': %s",
                  nodeName.c_str(), result.reason.c_str());
    }
  }
}

bool ParameterInterface::isWhitelistedParam(const std::string& paramName) {
  return std::find_if(_paramWhitelistPatterns.begin(), _paramWhitelistPatterns.end(),
                      [paramName](const auto& regex) {
                        return std::regex_match(paramName, regex);
                      }) != _paramWhitelistPatterns.end();
}

}  // namespace foxglove_bridge
