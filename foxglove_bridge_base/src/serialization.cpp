#include <foxglove_bridge/serialization.hpp>

namespace foxglove {

void to_json(nlohmann::json& j, const Parameter& p) {
  const auto paramType = p.getType();
  if (paramType == ParameterType::PARAMETER_BOOL) {
    j["value"] = p.getValue<bool>();
  } else if (paramType == ParameterType::PARAMETER_INTEGER) {
    j["value"] = p.getValue<int64_t>();
  } else if (paramType == ParameterType::PARAMETER_DOUBLE) {
    j["value"] = p.getValue<double>();
  } else if (paramType == ParameterType::PARAMETER_STRING) {
    j["value"] = p.getValue<std::string>();
  } else if (paramType == ParameterType::PARAMETER_BOOL_ARRAY) {
    j["value"] = p.getValue<std::vector<bool>>();
  } else if (paramType == ParameterType::PARAMETER_INTEGER_ARRAY) {
    j["value"] = p.getValue<std::vector<int64_t>>();
  } else if (paramType == ParameterType::PARAMETER_DOUBLE_ARRAY) {
    j["value"] = p.getValue<std::vector<double>>();
  } else if (paramType == ParameterType::PARAMETER_STRING_ARRAY) {
    j["value"] = p.getValue<std::vector<std::string>>();
  } else if (paramType == ParameterType::PARAMETER_NOT_SET) {
    // empty value.
  }

  j["name"] = p.getName();
}

void from_json(const nlohmann::json& j, Parameter& p) {
  const auto name = j["name"].get<std::string>();

  if (j.find("value") == j.end()) {
    p = Parameter(name);  // Value is not set (undefined).
    return;
  }

  const auto value = j["value"];
  const auto jsonType = j["value"].type();

  if (jsonType == nlohmann::detail::value_t::string) {
    p = Parameter(name, value.get<std::string>());
  } else if (jsonType == nlohmann::detail::value_t::boolean) {
    p = Parameter(name, value.get<bool>());
  } else if (jsonType == nlohmann::detail::value_t::number_integer) {
    p = Parameter(name, value.get<int64_t>());
  } else if (jsonType == nlohmann::detail::value_t::number_unsigned) {
    p = Parameter(name, value.get<int64_t>());
  } else if (jsonType == nlohmann::detail::value_t::number_float) {
    p = Parameter(name, value.get<double>());
  } else if (jsonType == nlohmann::detail::value_t::array) {
    if (value.empty()) {
      // We do not know the type when an empty array is received.
      throw std::runtime_error("Setting empty arrays is currently unsupported.");
    }

    if (value.front().is_string()) {
      p = Parameter(name, value.get<std::vector<std::string>>());
    } else if (value.front().is_boolean()) {
      p = Parameter(name, value.get<std::vector<bool>>());
    } else if (value.front().is_number_integer()) {
      p = Parameter(name, value.get<std::vector<int64_t>>());
    } else if (value.front().is_number_unsigned()) {
      p = Parameter(name, value.get<std::vector<int64_t>>());
    } else if (value.front().is_number_float()) {
      p = Parameter(name, value.get<std::vector<double>>());
    } else {
      throw std::runtime_error("Unsupported array type");
    }
  } else {
    throw std::runtime_error("Unsupported type");
  }
}

}  // namespace foxglove
