#include <foxglove_bridge/base64.hpp>
#include <foxglove_bridge/serialization.hpp>

namespace foxglove {

void to_json(nlohmann::json& j, const Channel& c) {
  j = {
    {"id", c.id},
    {"topic", c.topic},
    {"encoding", c.encoding},
    {"schemaName", c.schemaName},
    {"schema", c.schema},
  };

  if (c.schemaEncoding.has_value()) {
    j["schemaEncoding"] = c.schemaEncoding.value();
  }
}
void from_json(const nlohmann::json& j, Channel& c) {
  const auto schemaEncoding =
    j.find("schemaEncoding") == j.end()
      ? std::optional<std::string>(std::nullopt)
      : std::optional<std::string>(j["schemaEncoding"].get<std::string>());

  ChannelWithoutId channelWithoutId{j["topic"].get<std::string>(), j["encoding"].get<std::string>(),
                                    j["schemaName"].get<std::string>(),
                                    j["schema"].get<std::string>(), schemaEncoding};
  c = Channel(j["id"].get<ChannelId>(), channelWithoutId);
}

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
  } else if (paramType == ParameterType::PARAMETER_BYTE_ARRAY) {
    const auto& paramValue = p.getValue<std::vector<unsigned char>>();
    const std::string_view strValue(reinterpret_cast<const char*>(paramValue.data()),
                                    paramValue.size());
    j["value"] = base64Encode(strValue);
    j["type"] = "byte_array";
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
    if (j.find("type") == j.end()) {
      p = Parameter(name, value.get<std::string>());
    } else if (j["type"] == "byte_array") {
      p = Parameter(name, base64Decode(value.get<std::string>()));
    } else {
      throw std::runtime_error("Unsupported parameter 'type' value: " + j.dump());
    }
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

void to_json(nlohmann::json& j, const Service& service) {
  j = {
    {"id", service.id},
    {"name", service.name},
    {"type", service.type},
    {"requestSchema", service.requestSchema},
    {"responseSchema", service.responseSchema},
  };
}

void from_json(const nlohmann::json& j, Service& p) {
  p.id = j["id"].get<ServiceId>();
  p.name = j["name"].get<std::string>();
  p.type = j["type"].get<std::string>();
  p.requestSchema = j["requestSchema"].get<std::string>();
  p.responseSchema = j["responseSchema"].get<std::string>();
}

void ServiceResponse::read(const uint8_t* data, size_t dataLength) {
  size_t offset = 0;
  this->serviceId = ReadUint32LE(data + offset);
  offset += 4;
  this->callId = ReadUint32LE(data + offset);
  offset += 4;
  const size_t encondingLength = static_cast<size_t>(ReadUint32LE(data + offset));
  offset += 4;
  this->encoding = std::string(reinterpret_cast<const char*>(data + offset), encondingLength);
  offset += encondingLength;
  const auto payloadLength = dataLength - offset;
  this->data.resize(payloadLength);
  std::memcpy(this->data.data(), data + offset, payloadLength);
}

void ServiceResponse::write(uint8_t* data) const {
  size_t offset = 0;
  foxglove::WriteUint32LE(data + offset, this->serviceId);
  offset += 4;
  foxglove::WriteUint32LE(data + offset, this->callId);
  offset += 4;
  foxglove::WriteUint32LE(data + offset, static_cast<uint32_t>(this->encoding.size()));
  offset += 4;
  std::memcpy(data + offset, this->encoding.data(), this->encoding.size());
  offset += this->encoding.size();
  std::memcpy(data + offset, this->data.data(), this->data.size());
}

}  // namespace foxglove
