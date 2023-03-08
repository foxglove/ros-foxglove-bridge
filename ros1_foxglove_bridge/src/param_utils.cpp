
#include <stdexcept>

#include <foxglove_bridge/param_utils.hpp>

namespace {

template <typename T>
std::vector<T> toVector(const XmlRpc::XmlRpcValue& value) {
  std::vector<T> arr;
  arr.reserve(static_cast<size_t>(value.size()));
  for (int i = 0; i < value.size(); i++) {
    arr.push_back(static_cast<T>(value[i]));
  }
  return arr;
}

}  // namespace

namespace foxglove_bridge {

foxglove::Parameter fromRosParam(const std::string& name, const XmlRpc::XmlRpcValue& value) {
  const auto type = value.getType();

  if (type == XmlRpc::XmlRpcValue::Type::TypeBoolean) {
    return foxglove::Parameter(name, static_cast<bool>(value));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeInt) {
    return foxglove::Parameter(name, static_cast<int64_t>(static_cast<int>(value)));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeDouble) {
    return foxglove::Parameter(name, static_cast<double>(value));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeString) {
    return foxglove::Parameter(name, static_cast<std::string>(value));
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeArray) {
    if (value.size() == 0) {
      // We can't defer the type in this case so we simply treat it as a int list
      return foxglove::Parameter(name, std::vector<int>());
    } else {
      const auto firstElement = value[0];
      const auto firstElementType = firstElement.getType();
      if (firstElementType == XmlRpc::XmlRpcValue::Type::TypeBoolean) {
        return foxglove::Parameter(name, toVector<bool>(value));
      } else if (firstElementType == XmlRpc::XmlRpcValue::Type::TypeInt) {
        return foxglove::Parameter(name, toVector<int>(value));
      } else if (firstElementType == XmlRpc::XmlRpcValue::Type::TypeDouble) {
        return foxglove::Parameter(name, toVector<double>(value));
      } else if (firstElementType == XmlRpc::XmlRpcValue::Type::TypeString) {
        return foxglove::Parameter(name, toVector<std::string>(value));
      } else {
        throw std::runtime_error("Parameter '" + name + "': Unsupported parameter array type");
      }
    }
  } else if (type == XmlRpc::XmlRpcValue::Type::TypeInvalid) {
    throw std::runtime_error("Parameter '" + name + "': Not set");
  } else {
    throw std::runtime_error("Parameter '" + name + "': Unsupported parameter type");
  }

  return foxglove::Parameter();
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
