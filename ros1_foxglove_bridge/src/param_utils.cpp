
#include <iostream>
#include <stdexcept>

#include <foxglove_bridge/param_utils.hpp>

namespace foxglove_bridge {

foxglove_ws::ParameterValue fromRosParam(const XmlRpc::XmlRpcValue& value) {
  const auto type = value.getType();

  if (type == XmlRpc::XmlRpcValue::Type::TypeBoolean) {
    return foxglove_ws::ParameterValue(static_cast<bool>(value));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeInt) {
    return foxglove_ws::ParameterValue(static_cast<int64_t>(static_cast<int>(value)));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeDouble) {
    return foxglove_ws::ParameterValue(static_cast<double>(value));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeString) {
    return foxglove_ws::ParameterValue(static_cast<std::string>(value));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeStruct) {
    std::unordered_map<std::string, foxglove_ws::ParameterValue> paramMap;
    for (const auto& [elementName, elementVal] : value) {
      paramMap.insert({elementName, fromRosParam(elementVal)});
    }
    return foxglove_ws::ParameterValue(paramMap);
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeArray) {
    std::vector<foxglove_ws::ParameterValue> paramVec;
    for (int i = 0; i < value.size(); ++i) {
      paramVec.push_back(fromRosParam(value[i]));
    }
    return foxglove_ws::ParameterValue(paramVec);
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeInvalid) {
    throw std::runtime_error("Parameter not set");
  } else {
    throw std::runtime_error("Unsupported parameter type: " + std::to_string(type));
  }
}

foxglove_ws::Parameter fromRosParam(const std::string& name, const XmlRpc::XmlRpcValue& value) {
  return foxglove_ws::Parameter(name, fromRosParam(value));
}

XmlRpc::XmlRpcValue toRosParam(const foxglove_ws::ParameterValue& param) {
  const auto paramType = param.getType();
  if (paramType == foxglove_ws::ParameterType::PARAMETER_BOOL) {
    return param.getValue<bool>();
  } else if (paramType == foxglove_ws::ParameterType::PARAMETER_INTEGER) {
    return static_cast<int>(param.getValue<int64_t>());
  } else if (paramType == foxglove_ws::ParameterType::PARAMETER_DOUBLE) {
    return param.getValue<double>();
  } else if (paramType == foxglove_ws::ParameterType::PARAMETER_STRING) {
    return param.getValue<std::string>();
  } else if (paramType == foxglove_ws::ParameterType::PARAMETER_STRUCT) {
    XmlRpc::XmlRpcValue valueStruct;
    const auto& paramMap =
      param.getValue<std::unordered_map<std::string, foxglove_ws::ParameterValue>>();
    for (const auto& [paramName, paramElement] : paramMap) {
      valueStruct[paramName] = toRosParam(paramElement);
    }
    return valueStruct;
  } else if (paramType == foxglove_ws::ParameterType::PARAMETER_ARRAY) {
    XmlRpc::XmlRpcValue arr;
    const auto vec = param.getValue<std::vector<foxglove_ws::ParameterValue>>();
    for (int i = 0; i < static_cast<int>(vec.size()); ++i) {
      arr[i] = toRosParam(vec[i]);
    }
    return arr;
  } else {
    throw std::runtime_error("Unsupported parameter type");
  }

  return XmlRpc::XmlRpcValue();
}

std::vector<std::regex> parseRegexPatterns(const std::vector<std::string>& patterns) {
  std::vector<std::regex> result;
  for (const auto& pattern : patterns) {
    try {
      result.push_back(
        std::regex(pattern, std::regex_constants::ECMAScript | std::regex_constants::icase));
    } catch (...) {
      continue;
    }
  }
  return result;
}

}  // namespace foxglove_bridge
