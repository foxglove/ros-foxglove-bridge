#include <foxglove_bridge/message_throttler.hpp>

namespace foxglove_bridge {

ThrottledMessage::ThrottledMessage(ThrottledTopicInfo* topicInfo,
                                   const rcl_serialized_message_t& serializedMsg)
    : _topicInfo(topicInfo)
    , _serializedMsg(serializedMsg) {
  tryDecode();
}

template <>
std::optional<std::string> ThrottledMessage::getDecodedMessageField(const std::string& fieldName) {
  if (!_topicInfo->parser) {
    return std::nullopt;
  }

  for (auto& field : _decodedMsg.name) {
    std::string fieldPath = field.first.toStdString();
    if (fieldPath.find(fieldName) != std::string::npos) {
      return std::make_optional<std::string>(field.second);
    }
  }

  return std::nullopt;
}

void ThrottledMessage::tryDecode() {
  if (!_topicInfo->parser) {
    _frameid = "";
    return;
  }

  RosMsgParser::ROS2_Deserializer deserializer;
  auto serializedMsgSpan = nonstd::span(_serializedMsg.buffer, _serializedMsg.buffer_length);
  {
    std::lock_guard<std::mutex> lock(_topicInfo->parserLock);
    if (!_topicInfo->parser->get()->deserialize(serializedMsgSpan, &_decodedMsg, &deserializer)) {
      throw std::runtime_error(
        "Failed to parse message, likely an array with too many elements, adjust parser max "
        "array policy");
    }
  }  // drop lock

  _frameid = getDecodedMessageField<std::string>("frame_id").value_or("");
}

bool ThrottledMessage::isAllowedThrough(Nanoseconds currentTime) {
  // either message has not been seen before or interval has passed
  std::shared_lock<std::shared_mutex> lock(_topicInfo->frameIdLastRecievedLock);
  return !_topicInfo->frameIdLastRecieved.count(_frameid) ||
         currentTime - _topicInfo->frameIdLastRecieved[_frameid] > _topicInfo->throttleInterval;
}

void ThrottledMessage::updateLastSeen(Nanoseconds currentTime) {
  std::unique_lock<std::shared_mutex> lock(_topicInfo->frameIdLastRecievedLock);
  _topicInfo->frameIdLastRecieved[_frameid] = currentTime;
}

std::optional<TypeSchema> MessageThrottleManager::getTypeSchema(const TypeName& type) {
  auto channelsAndLock = _server->getChannels();
  for (const auto& [_, channel] : channelsAndLock.channels) {
    if (channel.schemaName == type) {
      return channel.schema;
    }
  }

  return std::nullopt;
}

std::optional<TypeName> MessageThrottleManager::getTypeFromTopic(const TopicName& topic) {
  auto channelsAndLock = _server->getChannels();
  for (const auto& [_, channel] : channelsAndLock.channels) {
    if (channel.topic == topic) {
      return channel.schemaName;
    }
  }

  return std::nullopt;
}

template <typename Lock>
std::optional<std::shared_ptr<RosMsgParser::Parser>> MessageThrottleManager::createParser(
  const TopicName& topic, [[maybe_unused]] Lock& lock) {
  assert(lock.mutex() == &_topicInfoLock && "Must have unique lock on _topicInfoLock");

  auto optType = getTypeFromTopic(topic);
  if (!optType) {
    return std::nullopt;
  }
  TypeName type = optType.value();

  auto schemaOpt = getTypeSchema(type);
  if (!schemaOpt) {
    return std::nullopt;
  }
  TypeSchema schema = schemaOpt.value();

  return std::make_optional(std::make_unique<RosMsgParser::Parser>(topic, type, schema));
}

bool MessageThrottleManager::shouldThrottleAndUpdate(ThrottledMessage& msg,
                                                     const Nanoseconds time) {
  if (!msg.isAllowedThrough(time)) {
    return true;
  }

  msg.updateLastSeen(time);
  return false;
}

bool MessageThrottleManager::isUnthrottledTopic(const TopicName& topic) {
  std::shared_lock<std::shared_mutex> lock(_unthrottledTopicLock);
  return _unthrottledTopics.count(topic) > 0;
}

void MessageThrottleManager::addUnthrottledTopic(const TopicName& topic) {
  std::unique_lock<std::shared_mutex> lock(_unthrottledTopicLock);
  _unthrottledTopics.emplace(topic);
}

template <typename Lock>
std::optional<bool> MessageThrottleManager::tryThrottleIfKnown(
  const TopicName& topic, const rcl_serialized_message_t& serializedMsg, const Nanoseconds now,
  [[maybe_unused]] Lock& lock) {
  assert(lock.mutex() == &_topicInfoLock && "Must have lock on _topicInfoLock");
  auto topicInfoIt = _throttledTopics.find(topic);
  if (topicInfoIt == _throttledTopics.end()) {
    return std::nullopt;
  }

  auto msg = ThrottledMessage(topicInfoIt->second.get(), serializedMsg);
  return std::make_optional(shouldThrottleAndUpdate(msg, now));
}

std::optional<bool> MessageThrottleManager::tryThrottleIfKnown(
  const TopicName& topic, const rcl_serialized_message_t& serializedMsg, const Nanoseconds now) {
  std::shared_lock<std::shared_mutex> lock(_topicInfoLock);
  auto topicInfoIt = _throttledTopics.find(topic);
  if (topicInfoIt == _throttledTopics.end()) {
    return std::nullopt;
  }

  auto msg = ThrottledMessage(topicInfoIt->second.get(), serializedMsg);
  return std::make_optional(shouldThrottleAndUpdate(msg, now));
}

bool MessageThrottleManager::shouldThrottle(const TopicName& topic,
                                            const rcl_serialized_message_t& serializedMsg,
                                            const Nanoseconds now) {
  if (auto knownTopic = tryThrottleIfKnown(topic, serializedMsg, now)) {
    return knownTopic.value();
  }

  if (isUnthrottledTopic(topic)) {
    return false;
  }

  auto optThrottleInterval = getTopicThrottleInterval(topic);
  // doesn't match any regex patterns
  if (!optThrottleInterval) {
    addUnthrottledTopic(topic);
    return false;
  }
  Nanoseconds throttleInterval = optThrottleInterval.value();

  std::unique_lock<std::shared_mutex> lock(_topicInfoLock);
  // make sure while waiting for lock another thread didn't handle this for the first time
  if (auto knownTopic = tryThrottleIfKnown(topic, serializedMsg, now, lock)) {
    return knownTopic.value();
  }

  auto topicInfo =
    std::make_unique<ThrottledTopicInfo>(throttleInterval, createParser(topic, lock));
  auto [newTopicInfoIt, wasInserted] = _throttledTopics.emplace(topic, std::move(topicInfo));
  assert(wasInserted &&
         "Tried to replace a topic already in _throttledTopics, should've been caught by initial "
         "already seen check");
  auto msg = ThrottledMessage(newTopicInfoIt->second.get(), serializedMsg);
  return shouldThrottleAndUpdate(msg, now);
}

std::optional<Nanoseconds> MessageThrottleManager::getTopicThrottleInterval(
  const TopicName& topic) {
  // NOTE: assumes throttle patterns and rates arrays are of same length
  // neither of these arrays should change after object initialization, thus no need for lock
  for (size_t i = 0; i < _topicThrottlePatterns.size(); i++) {
    if (std::regex_match(topic, _topicThrottlePatterns[i])) {
      return static_cast<Nanoseconds>(1 / _topicThrottleRates[i] * NANOSECONDS_IN_SECOND);
    }
  }

  return std::nullopt;
}

}  // namespace foxglove_bridge
