#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace foxglove {

constexpr char SUPPORTED_SUBPROTOCOL[] = "foxglove.websocket.v1";

using ChannelId = uint32_t;
using ClientChannelId = uint32_t;
using SubscriptionId = uint32_t;

enum class BinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
};

enum class ClientBinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
};

struct ClientAdvertisement {
  ClientChannelId channelId;
  std::string topic;
  std::string encoding;
  std::string schemaName;
  std::vector<uint8_t> schema;
};

}  // namespace foxglove
