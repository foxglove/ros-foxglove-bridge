#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace foxglove {

constexpr char SUPPORTED_SUBPROTOCOL[] = "foxglove.websocket.v1";
constexpr char CAPABILITY_CLIENT_PUBLISH[] = "clientPublish";
constexpr char CAPABILITY_TIME[] = "time";
constexpr char CAPABILITY_PARAMETERS[] = "parameters";
constexpr char CAPABILITY_PARAMETERS_SUBSCRIBE[] = "parametersSubscribe";
constexpr char CAPABILITY_SERVICES[] = "services";

using ChannelId = uint32_t;
using ClientChannelId = uint32_t;
using SubscriptionId = uint32_t;
using ServiceId = uint32_t;

enum class BinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
  TIME_DATA = 2,
  SERVICE_CALL_RESPONSE = 3,
};

enum class ClientBinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
  SERVICE_CALL_REQUEST = 2,
};

struct ClientAdvertisement {
  ClientChannelId channelId;
  std::string topic;
  std::string encoding;
  std::string schemaName;
  std::vector<uint8_t> schema;
};

struct ServiceWithoutId {
  std::string name;
  std::string type;
  std::string requestSchema;
  std::string responseSchema;
};

struct Service : ServiceWithoutId {
  ServiceId id;

  Service() = default;
  Service(const ServiceWithoutId& s, const ServiceId& id)
      : ServiceWithoutId(s)
      , id(id) {}
};

struct ServiceResponse {
  ServiceId serviceId;
  uint32_t callId;
  std::string encoding;
  std::vector<uint8_t> data;

  size_t size() const {
    return 4 + 4 + 4 + encoding.size() + data.size();
  }
  void read(const uint8_t* data, size_t size);
  void write(uint8_t* data) const;

  bool operator==(const ServiceResponse& other) const {
    return serviceId == other.serviceId && callId == other.callId && encoding == other.encoding &&
           data == other.data;
  }
};

using ServiceRequest = ServiceResponse;

}  // namespace foxglove
