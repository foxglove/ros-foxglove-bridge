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

ParameterInterface::ParameterInterface(rclcpp::Node* node)
    : _node(node)
    , _callbackGroup(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {}

ParameterList ParameterInterface::getParams(const std::vector<std::string>& paramNames,
                                            const std::chrono::duration<double>& timeout) {
  std::lock_guard<std::mutex> lock(_mutex);

  std::unordered_map<std::string, std::vector<std::string>> paramNamesByNodeName;
  for (const auto& paramName : paramNames) {
    const auto& [nodeName, paramN] = getNodeAndParamName(paramName);
    paramNamesByNodeName[nodeName].push_back(paramN);
  }

  if (paramNamesByNodeName.empty()) {
    const auto nodeNames = _node->get_node_names();
    for (const auto& nodeName : nodeNames) {
      paramNamesByNodeName.insert({nodeName, {}});
    }
  }

  RCLCPP_INFO(_node->get_logger(), "Getting %zu paramters for %zu nodes...", paramNames.size(),
              paramNamesByNodeName.size());

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
      RCLCPP_ERROR(_node->get_logger(), "Exception when getting paramters: %s", e.what());
    }
  }

  return result;
}

void ParameterInterface::setParams(const ParameterList& parameters,
                                   const std::chrono::duration<double>& timeout) {
  std::lock_guard<std::mutex> lock(_mutex);

  rclcpp::ParameterMap paramsByNode;
  for (const auto& param : parameters) {
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
      RCLCPP_ERROR(_node->get_logger(), "Exception when setting paramters: %s", e.what());
    }
  }
}

void ParameterInterface::subscribeParams(const std::vector<std::string>& paramNames) {
  std::lock_guard<std::mutex> lock(_mutex);

  std::unordered_set<std::string> nodesToSubscribe;
  for (const auto& paramName : paramNames) {
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
  const auto deadline = std::chrono::system_clock::now() + timeout;

  if (!paramClient->service_is_ready() || !paramClient->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error("Param client for node '" + nodeName + "' not ready");
  }

  auto paramsToRequest = paramNames;
  if (paramsToRequest.empty()) {
    auto future = paramClient->list_parameters({}, 0UL);
    if (std::future_status::ready != future.wait_until(deadline)) {
      throw std::runtime_error("Failed to retrieve parameter names for node '" + nodeName + "'");
    }
    paramsToRequest = future.get().names;
  }

  auto getParamsFuture = paramClient->get_parameters(paramsToRequest);
  if (std::future_status::ready != getParamsFuture.wait_until(deadline)) {
    throw std::runtime_error("Failed to get parameter values for node '" + nodeName + "'");
  }
  const auto params = getParamsFuture.get();

  ParameterList result;
  for (const auto& param : params) {
    result.push_back(fromRosParam(rclcpp::Parameter(
      prependNodeNameToParamName(param.get_name(), nodeName), param.get_parameter_value())));
  }
  return result;
}

void ParameterInterface::setNodeParameters(rclcpp::AsyncParametersClient::SharedPtr paramClient,
                                           const std::string& nodeName,
                                           const std::vector<rclcpp::Parameter>& params,
                                           const std::chrono::duration<double>& timeout) {
  if (!paramClient->service_is_ready() || !paramClient->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error("Param client for node '" + nodeName + "' not ready");
  }

  auto future = paramClient->set_parameters(params);
  if (std::future_status::ready != future.wait_for(timeout)) {
    throw std::runtime_error("Param client failed to set parameters for node '" + nodeName +
                             "' within the given timeout");
  }

  const auto setParamResults = future.get();
  for (auto& result : setParamResults) {
    if (!result.successful) {
      RCLCPP_WARN(_node->get_logger(), "Failed to set a paramter for node '%s': %s",
                  nodeName.c_str(), result.reason.c_str());
    }
  }
}

}  // namespace foxglove_bridge
