#ifdef ENABLE_JSON_MESSAGES

#include "foxglove_bridge/json_to_ros.hpp"

#include <nlohmann/json.hpp>

namespace foxglove_bridge {

static void assignJsonToMessageField(ros2_babel_fish::Message& field, const nlohmann::json& value) {
  using MessageType = ros2_babel_fish::MessageType;

  if (value.is_null()) {
    return;
  }

  switch (field.type()) {
    case MessageType::None:
      break;
    case MessageType::Float:
      field = value.get<float>();
      break;
    case MessageType::Double:
      field = value.get<double>();
      break;
    case MessageType::LongDouble:
      field = value.get<long double>();
      break;
    case MessageType::Char:
      field = value.get<char>();
      break;
    case MessageType::WChar:
      field = value.get<wchar_t>();
      break;
    case MessageType::Bool:
      field = value.get<bool>();
      break;
    case MessageType::Octet:
    case MessageType::UInt8:
      field = value.get<uint8_t>();
      break;
    case MessageType::Int8:
      field = value.get<int8_t>();
      break;
    case MessageType::UInt16:
      field = value.get<uint16_t>();
      break;
    case MessageType::Int16:
      field = value.get<int16_t>();
      break;
    case MessageType::UInt32:
      field = value.get<uint32_t>();
      break;
    case MessageType::Int32:
      field = value.get<int32_t>();
      break;
    case MessageType::UInt64:
      field = value.get<uint64_t>();
      break;
    case MessageType::Int64:
      field = value.get<int64_t>();
      break;
    case MessageType::String:
      field = value.get<std::string>();
      break;
    case MessageType::WString:
      field = value.get<std::wstring>();
      break;
    case MessageType::Compound: {
      // Recursively convert compound messages
      auto& compound = field.as<ros2_babel_fish::CompoundMessage>();
      for (const auto& key : compound.keys()) {
        assignJsonToMessageField(compound[key], value[key]);
      }
      break;
    }
    case MessageType::Array: {
      // Ensure the JSON value is an array
      if (!value.is_array()) {
        break;
      }

      auto& array = field.as<ros2_babel_fish::CompoundArrayMessage>();
      if (array.isFixedSize() || array.isBounded()) {
        const size_t limit = std::min(array.maxSize(), value.size());
        for (size_t i = 0; i < limit; ++i) {
          auto& arrayEntry = array.size() > i ? array[i] : array.appendEmpty();
          assignJsonToMessageField(arrayEntry, value[i]);
        }
      } else {
        array.clear();
        for (const auto& jsonArrayEntry : value) {
          auto& arrayEntry = array.appendEmpty();
          assignJsonToMessageField(arrayEntry, jsonArrayEntry);
        }
      }
      break;
    }
  }
}

std::optional<std::exception> jsonMessageToRos(
  const std::string_view jsonMessage, const std::string& schemaName,
  ros2_babel_fish::BabelFish::SharedPtr babelFish,
  ros2_babel_fish::CompoundMessage::SharedPtr& outputMessage) {
  // Decode the JSON message
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(jsonMessage);
  } catch (const nlohmann::json::parse_error& e) {
    return e;
  }

  // Convert the JSON message to a ROS message using ros2_babel_fish
  ros2_babel_fish::CompoundMessage::SharedPtr rosMsgPtr;
  try {
    rosMsgPtr = babelFish->create_message_shared(schemaName);
  } catch (const ros2_babel_fish::BabelFishException& e) {
    return e;
  }
  auto& rosMsg = *rosMsgPtr;
  for (const auto& key : rosMsg.keys()) {
    if (!json.contains(key)) {
      continue;
    }

    try {
      assignJsonToMessageField(rosMsg[key], json[key]);
    } catch (const std::exception& e) {
      return e;
    }
  }

  outputMessage = rosMsgPtr;
  return std::nullopt;
}

}  // namespace foxglove_bridge

#endif
