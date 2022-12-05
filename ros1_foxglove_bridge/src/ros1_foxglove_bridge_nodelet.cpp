#define ASIO_STANDALONE

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <shared_mutex>
#include <string>
#include <unordered_set>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_event.h>
#include <ros/ros.h>
#include <ros_babel_fish/babel_fish_message.h>
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
#include <rosgraph_msgs/Clock.h>

#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/websocket_server.hpp>

namespace foxglove_bridge {

constexpr int DEFAULT_PORT = 8765;
constexpr char DEFAULT_ADDRESS[] = "0.0.0.0";
constexpr int DEFAULT_MAX_UPDATE_MS = 5000;
constexpr char ROS1_CHANNEL_ENCODING[] = "ros1";
constexpr uint32_t SUBSCRIPTION_QUEUE_LENGTH = 10;
constexpr double MIN_UPDATE_PERIOD_MS = 100.0;
constexpr uint32_t PUBLICATION_QUEUE_LENGTH = 10;

using TopicAndDatatype = std::pair<std::string, std::string>;
using SubscriptionsByClient = std::map<foxglove::ConnHandle, ros::Subscriber, std::owner_less<>>;
using ClientPublications = std::unordered_map<foxglove::ClientChannelId, ros::Publisher>;
using PublicationsByClient = std::map<foxglove::ConnHandle, ClientPublications, std::owner_less<>>;

class FoxgloveBridge : public nodelet::Nodelet {
public:
  FoxgloveBridge() = default;
  virtual void onInit() {
    auto& nhp = getPrivateNodeHandle();
    const auto address = nhp.param<std::string>("address", DEFAULT_ADDRESS);
    const int port = nhp.param<int>("port", DEFAULT_PORT);
    const auto useTLS = nhp.param<bool>("tls", false);
    const auto certfile = nhp.param<std::string>("certfile", "");
    const auto keyfile = nhp.param<std::string>("keyfile", "");
    _maxUpdateMs = static_cast<size_t>(nhp.param<int>("max_update_ms", DEFAULT_MAX_UPDATE_MS));

    const auto regexPatterns = nhp.param<std::vector<std::string>>("topic_whitelist", {".*"});
    _topicWhitelistPatterns.reserve(regexPatterns.size());
    for (const auto& pattern : regexPatterns) {
      try {
        _topicWhitelistPatterns.push_back(
          std::regex(pattern, std::regex_constants::ECMAScript | std::regex_constants::icase));
      } catch (const std::exception& ex) {
        ROS_ERROR("Ignoring invalid regular expression '%s': %s", pattern.c_str(), ex.what());
      }
    }

    ROS_INFO("Starting %s with %s", ros::this_node::getName().c_str(),
             foxglove::WebSocketUserAgent());

    try {
      const auto logHandler =
        std::bind(&FoxgloveBridge::logHandler, this, std::placeholders::_1, std::placeholders::_2);
      if (useTLS) {
        _server = std::make_unique<foxglove::Server<foxglove::WebSocketTls>>(
          "foxglove_bridge", std::move(logHandler), certfile, keyfile);
      } else {
        _server = std::make_unique<foxglove::Server<foxglove::WebSocketNoTls>>(
          "foxglove_bridge", std::move(logHandler));
      }

      _server->setSubscribeHandler(std::bind(&FoxgloveBridge::subscribeHandler, this,
                                             std::placeholders::_1, std::placeholders::_2));
      _server->setUnsubscribeHandler(std::bind(&FoxgloveBridge::unsubscribeHandler, this,
                                               std::placeholders::_1, std::placeholders::_2));
      _server->setClientAdvertiseHandler(std::bind(&FoxgloveBridge::clientAdvertiseHandler, this,
                                                   std::placeholders::_1, std::placeholders::_2));
      _server->setClientUnadvertiseHandler(std::bind(&FoxgloveBridge::clientUnadvertiseHandler,
                                                     this, std::placeholders::_1,
                                                     std::placeholders::_2));
      _server->setClientMessageHandler(std::bind(&FoxgloveBridge::clientMessageHandler, this,
                                                 std::placeholders::_1, std::placeholders::_2));
      _server->start(address, static_cast<uint16_t>(port));

      updateAdvertisedTopics(ros::TimerEvent());
    } catch (const std::exception& err) {
      ROS_ERROR("Failed to start websocket server: %s", err.what());
      // Rethrow exception such that the nodelet is unloaded.
      throw err;
    }
  };
  virtual ~FoxgloveBridge() {
    if (_server) {
      _server->stop();
    }
  }

private:
  struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
      return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

  void subscribeHandler(foxglove::ChannelId channelId, foxglove::ConnHandle clientHandle) {
    std::lock_guard<std::mutex> lock(_subscriptionsMutex);

    auto it = _channelToTopicAndDatatype.find(channelId);
    if (it == _channelToTopicAndDatatype.end()) {
      ROS_WARN("Received subscribe request for unknown channel %d", channelId);
      return;
    }
    auto& topicAndDatatype = it->second;
    auto topic = topicAndDatatype.first;
    auto datatype = topicAndDatatype.second;
    auto it2 = _advertisedTopics.find(topicAndDatatype);
    if (it2 == _advertisedTopics.end()) {
      ROS_ERROR("Channel %d for topic \"%s\" (%s) is not advertised", channelId, topic.c_str(),
                datatype.c_str());
      return;
    }
    const auto& channel = it2->second;

    // Get client subscriptions for this channel or insert an empty map.
    auto [subscriptionsIt, firstSubscription] =
      _subscriptions.emplace(channelId, SubscriptionsByClient());
    auto& subscriptionsByClient = subscriptionsIt->second;

    if (!firstSubscription &&
        subscriptionsByClient.find(clientHandle) != subscriptionsByClient.end()) {
      ROS_WARN("Client is already subscribed to channel %d", channelId);
      return;
    }

    try {
      subscriptionsByClient.emplace(
        clientHandle, getMTNodeHandle().subscribe<ros_babel_fish::BabelFishMessage>(
                        topic, SUBSCRIPTION_QUEUE_LENGTH,
                        std::bind(&FoxgloveBridge::rosMessageHandler, this, channel, clientHandle,
                                  std::placeholders::_1)));
      if (firstSubscription) {
        ROS_INFO("Subscribed to topic \"%s\" (%s) on channel %d", topic.c_str(), datatype.c_str(),
                 channelId);

      } else {
        ROS_INFO("Added subscriber #%zu to topic \"%s\" (%s) on channel %d",
                 subscriptionsByClient.size(), topic.c_str(), datatype.c_str(), channelId);
      }
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to subscribe to topic \"%s\" (%s): %s", topic.c_str(), datatype.c_str(),
                ex.what());
    }
  }

  void unsubscribeHandler(foxglove::ChannelId channelId, foxglove::ConnHandle clientHandle) {
    std::lock_guard<std::mutex> lock(_subscriptionsMutex);

    auto it = _channelToTopicAndDatatype.find(channelId);
    TopicAndDatatype topicAndDatatype =
      it != _channelToTopicAndDatatype.end()
        ? it->second
        : std::make_pair<std::string, std::string>("[Unknown]", "[Unknown]");

    auto it2 = _subscriptions.find(channelId);
    if (it2 == _subscriptions.end()) {
      ROS_WARN("Received unsubscribe request for unknown channel %d", channelId);
      return;
    }

    auto subscriptionsIt = _subscriptions.find(channelId);
    if (subscriptionsIt == _subscriptions.end()) {
      ROS_WARN("Received unsubscribe request for channel %d that was not subscribed to", channelId);
      return;
    }

    auto& subscriptionsByClient = subscriptionsIt->second;
    const auto clientSubscription = subscriptionsByClient.find(clientHandle);
    if (clientSubscription == subscriptionsByClient.end()) {
      ROS_WARN(
        "Received unsubscribe request for channel %d from a client that was not subscribed to this "
        "channel",
        channelId);
      return;
    }

    subscriptionsByClient.erase(clientSubscription);
    if (subscriptionsByClient.empty()) {
      ROS_INFO("Unsubscribing from topic \"%s\" (%s) on channel %d", topicAndDatatype.first.c_str(),
               topicAndDatatype.second.c_str(), channelId);
      _subscriptions.erase(it2);
    } else {
      ROS_INFO("Removed one subscription from channel %d (%zu subscription(s) left)", channelId,
               subscriptionsByClient.size());
    }
  }

  void clientAdvertiseHandler(const foxglove::ClientAdvertisement& channel,
                              foxglove::ConnHandle clientHandle) {
    if (channel.encoding != ROS1_CHANNEL_ENCODING) {
      ROS_ERROR("Unsupported encoding. Only '%s' encoding is supported at the moment.",
                ROS1_CHANNEL_ENCODING);
      return;
    }

    std::unique_lock<std::shared_mutex> lock(_publicationsMutex);

    // Get client publications or insert an empty map.
    auto [clientPublicationsIt, isFirstPublication] =
      _clientAdvertisedTopics.emplace(clientHandle, ClientPublications());

    auto& clientPublications = clientPublicationsIt->second;
    if (!isFirstPublication &&
        clientPublications.find(channel.channelId) != clientPublications.end()) {
      ROS_WARN("Received client advertisement from %s for channel %d it had already advertised",
               _server->remoteEndpointString(clientHandle).c_str(), channel.channelId);
      return;
    }

    const auto msgDescription = _rosTypeInfoProvider.getMessageDescription(channel.schemaName);
    if (!msgDescription) {
      ROS_ERROR(
        "Failed to retrieve type information of data type '%s'. Unable to advertise topic '%s'",
        channel.schemaName.c_str(), channel.topic.c_str());
      return;
    }

    ros::AdvertiseOptions advertiseOptions;
    advertiseOptions.datatype = channel.schemaName;
    advertiseOptions.has_header = false;  // TODO
    advertiseOptions.latch = false;
    advertiseOptions.md5sum = msgDescription->md5;
    advertiseOptions.message_definition = msgDescription->message_definition;
    advertiseOptions.queue_size = PUBLICATION_QUEUE_LENGTH;
    advertiseOptions.topic = channel.topic;
    auto publisher = getMTNodeHandle().advertise(advertiseOptions);

    if (publisher) {
      clientPublications.insert({channel.channelId, std::move(publisher)});
      ROS_INFO("Client %s is advertising \"%s\" (%s) on channel %d",
               _server->remoteEndpointString(clientHandle).c_str(), channel.topic.c_str(),
               channel.schemaName.c_str(), channel.channelId);
    } else {
      ROS_ERROR("Failed to create publisher for topic \"%s\" (%s)", channel.topic.c_str(),
                channel.schemaName.c_str());
    }
  }

  void clientUnadvertiseHandler(foxglove::ClientChannelId channelId,
                                foxglove::ConnHandle clientHandle) {
    std::unique_lock<std::shared_mutex> lock(_publicationsMutex);

    auto clientPublicationsIt = _clientAdvertisedTopics.find(clientHandle);
    if (clientPublicationsIt == _clientAdvertisedTopics.end()) {
      ROS_DEBUG(
        "Ignoring client unadvertisement from %s for unknown channel %d, client has no "
        "advertised topics",
        _server->remoteEndpointString(clientHandle).c_str(), channelId);
      return;
    }

    auto& clientPublications = clientPublicationsIt->second;

    auto channelPublicationIt = clientPublications.find(channelId);
    if (channelPublicationIt == clientPublications.end()) {
      ROS_WARN(
        "Ignoring client unadvertisement from %s for unknown channel %d, client has %zu "
        "advertised topic(s)",
        _server->remoteEndpointString(clientHandle).c_str(), channelId, clientPublications.size());
      return;
    }

    const auto& publisher = channelPublicationIt->second;
    ROS_INFO("Client %s is no longer advertising %s (%d subscribers) on channel %d",
             _server->remoteEndpointString(clientHandle).c_str(), publisher.getTopic().c_str(),
             publisher.getNumSubscribers(), channelId);
    clientPublications.erase(channelPublicationIt);

    if (clientPublications.empty()) {
      _clientAdvertisedTopics.erase(clientPublicationsIt);
    }
  }

  void clientMessageHandler(const foxglove::ClientMessage& clientMsg,
                            foxglove::ConnHandle clientHandle) {
    ros_babel_fish::BabelFishMessage::Ptr msg(new ros_babel_fish::BabelFishMessage);
    msg->read(clientMsg);

    const auto channelId = clientMsg.advertisement.channelId;
    std::shared_lock<std::shared_mutex> lock(_publicationsMutex);

    auto clientPublicationsIt = _clientAdvertisedTopics.find(clientHandle);
    if (clientPublicationsIt == _clientAdvertisedTopics.end()) {
      ROS_WARN(
        "Dropping client message from %s for unknown channel %d, client has no "
        "advertised topics",
        _server->remoteEndpointString(clientHandle).c_str(), channelId);
      return;
    }

    auto& clientPublications = clientPublicationsIt->second;

    auto channelPublicationIt = clientPublications.find(clientMsg.advertisement.channelId);
    if (channelPublicationIt == clientPublications.end()) {
      ROS_WARN(
        "Dropping client message from %s for unknown channel %d, client has %zu "
        "advertised topic(s)",
        _server->remoteEndpointString(clientHandle).c_str(), channelId, clientPublications.size());
      return;
    }
    channelPublicationIt->second.publish(msg);
  }

  void updateAdvertisedTopics(const ros::TimerEvent&) {
    _updateTimer.stop();
    if (!ros::ok()) {
      return;
    }

    // Get the current list of visible topics and datatypes from the ROS graph
    std::vector<ros::master::TopicInfo> topicNamesAndTypes;
    if (!ros::master::getTopics(topicNamesAndTypes)) {
      ROS_WARN("Failed to retrieve published topics from ROS master.");
      return;
    }

    std::unordered_set<TopicAndDatatype, PairHash> latestTopics;
    latestTopics.reserve(topicNamesAndTypes.size());
    bool hasClockTopic = false;
    for (const auto& topicNameAndType : topicNamesAndTypes) {
      const auto& topicName = topicNameAndType.name;
      const auto& datatype = topicNameAndType.datatype;

      // Check if a /clock topic is published
      hasClockTopic = hasClockTopic || (topicName == "/clock" && datatype == "rosgraph_msgs/Clock");

      // Ignore the topic if it is not on the topic whitelist
      if (std::find_if(_topicWhitelistPatterns.begin(), _topicWhitelistPatterns.end(),
                       [&topicName](const auto& regex) {
                         return std::regex_match(topicName, regex);
                       }) != _topicWhitelistPatterns.end()) {
        latestTopics.emplace(topicName, datatype);
      }
    }

    if (const auto numIgnoredTopics = topicNamesAndTypes.size() - latestTopics.size()) {
      ROS_DEBUG(
        "%zu topics have been ignored as they do not match any pattern on the topic whitelist",
        numIgnoredTopics);
    }

    // Enable or disable simulated time based on the presence of a /clock topic
    if (!_useSimTime && hasClockTopic) {
      ROS_INFO("/clock topic found, using simulated time");
      _useSimTime = true;
      _clockSubscription = getMTNodeHandle().subscribe<rosgraph_msgs::Clock>(
        "/clock", 10, [&](const rosgraph_msgs::Clock::ConstPtr msg) {
          _simTimeNs = msg->clock.toNSec();
        });
    } else if (_useSimTime && !hasClockTopic) {
      ROS_WARN("/clock topic disappeared");
      _useSimTime = false;
      _clockSubscription.shutdown();
    }

    // Create a list of topics that are new to us
    std::vector<TopicAndDatatype> newTopics;
    for (const auto& topic : latestTopics) {
      if (_advertisedTopics.find(topic) == _advertisedTopics.end()) {
        newTopics.push_back(topic);
      }
    }

    // Create a list of topics that have been removed
    std::vector<TopicAndDatatype> removedTopics;
    for (const auto& [topic, channel] : _advertisedTopics) {
      (void)channel;
      if (latestTopics.find(topic) == latestTopics.end()) {
        removedTopics.push_back(topic);
      }
    }

    // Remove advertisements for topics that have been removed
    {
      std::lock_guard<std::mutex> lock(_subscriptionsMutex);
      for (const auto& topicAndDatatype : removedTopics) {
        auto& channel = _advertisedTopics.at(topicAndDatatype);

        // Stop tracking this channel in the WebSocket server
        _server->removeChannel(channel.id);

        // Remove the subscription for this topic, if any
        _subscriptions.erase(channel.id);

        // Remove this topic+datatype tuple
        _channelToTopicAndDatatype.erase(channel.id);
        _advertisedTopics.erase(topicAndDatatype);

        ROS_DEBUG("Removed channel %d for topic \"%s\" (%s)", channel.id,
                  topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str());
      }

      // Advertise new topics
      for (const auto& topicAndDatatype : newTopics) {
        foxglove::ChannelWithoutId newChannel{};
        newChannel.topic = topicAndDatatype.first;
        newChannel.schemaName = topicAndDatatype.second;
        newChannel.encoding = ROS1_CHANNEL_ENCODING;

        try {
          const auto msgDescription =
            _rosTypeInfoProvider.getMessageDescription(topicAndDatatype.second);
          if (msgDescription) {
            newChannel.schema = msgDescription->message_definition;
          } else {
            ROS_WARN("Could not find definition for type %s", topicAndDatatype.second.c_str());

            // We still advertise the channel, but with an emtpy schema
            newChannel.schema = "";
          }
        } catch (const std::exception& err) {
          ROS_WARN("Failed to add channel for topic \"%s\" (%s): %s",
                   topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str(), err.what());
          continue;
        }

        auto channel = foxglove::Channel{_server->addChannel(newChannel), newChannel};
        ROS_DEBUG("Advertising channel %d for topic \"%s\" (%s)", channel.id, channel.topic.c_str(),
                  channel.schemaName.c_str());

        // Add a mapping from the topic+datatype tuple to the channel, and channel ID to the
        // topic+datatype tuple
        _advertisedTopics.emplace(topicAndDatatype, std::move(channel));
        _channelToTopicAndDatatype.emplace(channel.id, topicAndDatatype);
      }
    }

    if (newTopics.size() > 0) {
      _server->broadcastChannels();
    }

    // Schedule the next update using truncated exponential backoff, between `MIN_UPDATE_PERIOD_MS`
    // and `_maxUpdateMs`
    _updateCount++;
    const auto nextUpdateMs = std::max(
      MIN_UPDATE_PERIOD_MS, static_cast<double>(std::min(size_t(1) << _updateCount, _maxUpdateMs)));
    _updateTimer = getMTNodeHandle().createTimer(ros::Duration(nextUpdateMs / 1e3),
                                                 &FoxgloveBridge::updateAdvertisedTopics, this);
  }

  void logHandler(foxglove::WebSocketLogLevel level, char const* msg) {
    switch (level) {
      case foxglove::WebSocketLogLevel::Debug:
        ROS_DEBUG("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Info:
        ROS_INFO("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Warn:
        ROS_WARN("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Error:
        ROS_ERROR("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Critical:
        ROS_FATAL("[WS] %s", msg);
        break;
    }
  }

  void rosMessageHandler(
    const foxglove::Channel& channel, foxglove::ConnHandle clientHandle,
    const ros::MessageEvent<ros_babel_fish::BabelFishMessage const>& msgEvent) {
    const auto& msg = msgEvent.getConstMessage();
    const auto receiptTimeNs = _useSimTime ? _simTimeNs.load() : msgEvent.getReceiptTime().toNSec();
    _server->sendMessage(
      clientHandle, channel.id, receiptTimeNs,
      std::string_view(reinterpret_cast<const char*>(msg->buffer()), msg->size()));
  }

  std::unique_ptr<foxglove::ServerInterface> _server;
  ros_babel_fish::IntegratedDescriptionProvider _rosTypeInfoProvider;
  std::vector<std::regex> _topicWhitelistPatterns;
  std::unordered_map<TopicAndDatatype, foxglove::Channel, PairHash> _advertisedTopics;
  std::unordered_map<foxglove::ChannelId, TopicAndDatatype> _channelToTopicAndDatatype;
  std::unordered_map<foxglove::ChannelId, SubscriptionsByClient> _subscriptions;
  PublicationsByClient _clientAdvertisedTopics;
  std::mutex _subscriptionsMutex;
  std::shared_mutex _publicationsMutex;
  ros::Timer _updateTimer;
  size_t _maxUpdateMs = size_t(DEFAULT_MAX_UPDATE_MS);
  size_t _updateCount = 0;
  ros::Subscriber _clockSubscription;
  std::atomic<uint64_t> _simTimeNs = 0;
  std::atomic<bool> _useSimTime = false;
};

}  // namespace foxglove_bridge

PLUGINLIB_EXPORT_CLASS(foxglove_bridge::FoxgloveBridge, nodelet::Nodelet)
