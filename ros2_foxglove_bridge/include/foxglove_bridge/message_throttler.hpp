#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rosx_introspection/ros_parser.hpp>
#include <websocketpp/common/connection_hdl.hpp>

#include <foxglove_bridge/server_interface.hpp>

#define NANOSECONDS_IN_SECOND 1'000'000'000

namespace foxglove_bridge {
using ConnectionHandle = websocketpp::connection_hdl;
using TypeName = std::string;
using Frameid = std::string;
using TopicName = std::string;
using TypeSchema = std::string;
using Nanoseconds = rcl_time_point_value_t;

struct ThrottledTopicInfo {
  TopicName topic;
  Nanoseconds throttleInterval;
  std::mutex parserLock;
  // NOTE: should be assigned value only on object creation
  std::optional<std::shared_ptr<RosMsgParser::Parser>> parser;
  // NOTE: for topics with no frame id, we use "" as frame id, map should only have one element
  std::unordered_map<Frameid, Nanoseconds> frameIdLastRecieved;
  std::shared_mutex frameIdLastRecievedLock;

  ThrottledTopicInfo(Nanoseconds interval, std::optional<std::shared_ptr<RosMsgParser::Parser>> p,
                     std::unordered_map<Frameid, Nanoseconds> frameMap = {})
      : throttleInterval(interval)
      , parser(p)
      , frameIdLastRecieved(std::move(frameMap)) {}
};

class ThrottledMessage {
public:
  ThrottledMessage(ThrottledTopicInfo* topicInfo, const rcl_serialized_message_t& serializedMsg);

  bool isAllowedThrough(Nanoseconds currentTime);

  void updateLastSeen(Nanoseconds currentTime);

  template <typename T>
  std::optional<T> getDecodedMessageField(const std::string& fieldName) {
    if (!_topicInfo->parser) {
      return std::nullopt;
    }

    for (auto& field : _decodedMsg.value) {
      std::string fieldPath = field.first.toStdString();
      if (fieldPath.find(fieldName) != std::string::npos) {
        return std::make_optional<T>(field.second.extract<T>());
      }
    }

    return std::nullopt;
  }

private:
  ThrottledTopicInfo* _topicInfo;
  Frameid _frameid;
  RosMsgParser::FlatMessage _decodedMsg;
  const rcl_serialized_message_t& _serializedMsg;

  void tryDecode();
};

class MessageThrottleManager {
public:
  MessageThrottleManager(foxglove::ServerInterface<ConnectionHandle>* server,
                         std::vector<double>& topicThrottleRates,
                         std::vector<std::regex>& topicThrottlePatterns)
      : _server(server)
      , _topicThrottleRates(topicThrottleRates)
      , _topicThrottlePatterns(topicThrottlePatterns) {}

  bool shouldThrottle(const TopicName& topic, const rcl_serialized_message_t& serializedMsg,
                      const Nanoseconds now);

private:
  std::unordered_map<TypeName, std::shared_ptr<RosMsgParser::Parser>> _messageParsers;
  std::unordered_map<TopicName, std::unique_ptr<ThrottledTopicInfo>> _throttledTopics;
  std::shared_mutex _topicInfoLock;
  std::unordered_set<TopicName> _unthrottledTopics;
  std::shared_mutex _unthrottledTopicLock;
  foxglove::ServerInterface<ConnectionHandle>* _server;
  const std::vector<double>& _topicThrottleRates;
  const std::vector<std::regex>& _topicThrottlePatterns;

  /// @param lock Must be a unique_lock on _topicInfoLock
  template <typename Lock>
  std::optional<std::shared_ptr<RosMsgParser::Parser>> createParser(const TopicName& topic,
                                                                    Lock& lock);

  std::optional<TypeSchema> getTypeSchema(const TypeName& type);

  std::optional<TypeName> getTypeFromTopic(const TopicName& topic);

  std::optional<Nanoseconds> getTopicThrottleInterval(const TopicName& topic);

  std::mutex& waitForParserLock(const TopicName& topic);

  bool shouldThrottleAndUpdate(ThrottledMessage& msg, const Nanoseconds time);

  /// Checks if a topic should be throttled based on existing throttle configuration.
  /// Two overloads are provided:
  /// 1. Convenience version that acquires its own shared lock (use when you don't need to hold the
  /// lock afterward)
  /// 2. Template version that accepts caller-provided lock (use when you already have a lock or
  /// need to hold it longer)
  /// @return std::nullopt if topic is unknown, otherwise bool indicating if message should be
  /// throttled
  std::optional<bool> tryThrottleIfKnown(const TopicName& topic,
                                         const rcl_serialized_message_t& serializedMsg,
                                         const Nanoseconds now);

  /// @param lock Must be a shared_lock or unique_lock on _topicInfoLock
  template <typename Lock>
  std::optional<bool> tryThrottleIfKnown(const TopicName& topic,
                                         const rcl_serialized_message_t& serializedMsg,
                                         const Nanoseconds now, Lock& lock);

  bool isUnthrottledTopic(const TopicName& topic);

  void addUnthrottledTopic(const TopicName& topic);
};
}  // namespace foxglove_bridge
