#define ASIO_STANDALONE

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
#include <ros/xmlrpc_manager.h>
#include <ros_babel_fish/babel_fish_message.h>
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
#include <rosgraph_msgs/Clock.h>

#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/generic_service.hpp>
#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/service_utils.hpp>
#include <foxglove_bridge/websocket_server.hpp>

namespace foxglove_bridge {

constexpr int DEFAULT_PORT = 8765;
constexpr char DEFAULT_ADDRESS[] = "0.0.0.0";
constexpr int DEFAULT_MAX_UPDATE_MS = 5000;
constexpr char ROS1_CHANNEL_ENCODING[] = "ros1";
constexpr uint32_t SUBSCRIPTION_QUEUE_LENGTH = 10;
constexpr double MIN_UPDATE_PERIOD_MS = 100.0;
constexpr uint32_t PUBLICATION_QUEUE_LENGTH = 10;
constexpr int SERVICE_TYPE_RETRIEVAL_TIMEOUT_MS = 250;

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
    const auto send_buffer_limit = static_cast<size_t>(
      nhp.param<int>("send_buffer_limit", foxglove::DEFAULT_SEND_BUFFER_LIMIT_BYTES));
    const auto useTLS = nhp.param<bool>("tls", false);
    const auto certfile = nhp.param<std::string>("certfile", "");
    const auto keyfile = nhp.param<std::string>("keyfile", "");
    _maxUpdateMs = static_cast<size_t>(nhp.param<int>("max_update_ms", DEFAULT_MAX_UPDATE_MS));
    const auto useCompression = nhp.param<bool>("use_compression", false);
    _useSimTime = nhp.param<bool>("/use_sim_time", false);
    const auto sessionId = nhp.param<std::string>("/run_id", std::to_string(std::time(nullptr)));

    const auto topicWhitelistPatterns =
      nhp.param<std::vector<std::string>>("topic_whitelist", {".*"});
    _topicWhitelistPatterns = parseRegexPatterns(topicWhitelistPatterns);
    if (topicWhitelistPatterns.size() != _topicWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more topic whitelist patterns");
    }
    const auto paramWhitelist = nhp.param<std::vector<std::string>>("param_whitelist", {".*"});
    _paramWhitelistPatterns = parseRegexPatterns(paramWhitelist);
    if (paramWhitelist.size() != _paramWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more param whitelist patterns");
    }

    const auto serviceWhitelist = nhp.param<std::vector<std::string>>("service_whitelist", {".*"});
    _serviceWhitelistPatterns = parseRegexPatterns(serviceWhitelist);
    if (serviceWhitelist.size() != _serviceWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more service whitelist patterns");
    }

    ROS_INFO("Starting %s with %s", ros::this_node::getName().c_str(),
             foxglove::WebSocketUserAgent());

    try {
      foxglove::ServerOptions serverOptions;
      serverOptions.capabilities = {
        foxglove::CAPABILITY_CLIENT_PUBLISH,
        foxglove::CAPABILITY_PARAMETERS,
        foxglove::CAPABILITY_PARAMETERS_SUBSCRIBE,
        foxglove::CAPABILITY_SERVICES,
      };
      if (_useSimTime) {
        serverOptions.capabilities.push_back(foxglove::CAPABILITY_TIME);
      }
      serverOptions.supportedEncodings = {ROS1_CHANNEL_ENCODING};
      serverOptions.metadata = {{"ROS_DISTRO", std::getenv("ROS_DISTRO")}};
      serverOptions.sendBufferLimitBytes = send_buffer_limit;
      serverOptions.sessionId = sessionId;
      serverOptions.useCompression = useCompression;

      const auto logHandler =
        std::bind(&FoxgloveBridge::logHandler, this, std::placeholders::_1, std::placeholders::_2);
      if (useTLS) {
        serverOptions.certfile = certfile;
        serverOptions.keyfile = keyfile;
        _server = std::make_unique<foxglove::Server<foxglove::WebSocketTls>>(
          "foxglove_bridge", std::move(logHandler), serverOptions);
      } else {
        _server = std::make_unique<foxglove::Server<foxglove::WebSocketNoTls>>(
          "foxglove_bridge", std::move(logHandler), serverOptions);
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
      _server->setParameterRequestHandler(std::bind(&FoxgloveBridge::parameterRequestHandler, this,
                                                    std::placeholders::_1, std::placeholders::_2,
                                                    std::placeholders::_3));
      _server->setParameterChangeHandler(std::bind(&FoxgloveBridge::parameterChangeHandler, this,
                                                   std::placeholders::_1, std::placeholders::_2,
                                                   std::placeholders::_3));
      _server->setParameterSubscriptionHandler(
        std::bind(&FoxgloveBridge::parameterSubscriptionHandler, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
      _server->setServiceRequestHandler(std::bind(&FoxgloveBridge::serviceRequestHandler, this,
                                                  std::placeholders::_1, std::placeholders::_2));

      _server->start(address, static_cast<uint16_t>(port));

      xmlrpcServer.bind("paramUpdate", std::bind(&FoxgloveBridge::parameterUpdates, this,
                                                 std::placeholders::_1, std::placeholders::_2));
      xmlrpcServer.start();

      updateAdvertisedTopicsAndServices(ros::TimerEvent());

      if (_useSimTime) {
        _clockSubscription = getMTNodeHandle().subscribe<rosgraph_msgs::Clock>(
          "/clock", 10, [&](const rosgraph_msgs::Clock::ConstPtr msg) {
            _server->broadcastTime(msg->clock.toNSec());
          });
      }
    } catch (const std::exception& err) {
      ROS_ERROR("Failed to start websocket server: %s", err.what());
      // Rethrow exception such that the nodelet is unloaded.
      throw err;
    }
  };
  virtual ~FoxgloveBridge() {
    xmlrpcServer.shutdown();
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

  void updateAdvertisedTopicsAndServices(const ros::TimerEvent&) {
    _updateTimer.stop();
    if (!ros::ok()) {
      return;
    }

    updateAdvertisedTopics();
    updateAdvertisedServices();

    // Schedule the next update using truncated exponential backoff, between `MIN_UPDATE_PERIOD_MS`
    // and `_maxUpdateMs`
    _updateCount++;
    const auto nextUpdateMs = std::max(
      MIN_UPDATE_PERIOD_MS, static_cast<double>(std::min(size_t(1) << _updateCount, _maxUpdateMs)));
    _updateTimer = getMTNodeHandle().createTimer(
      ros::Duration(nextUpdateMs / 1e3), &FoxgloveBridge::updateAdvertisedTopicsAndServices, this);
  }

  void updateAdvertisedTopics() {
    // Get the current list of visible topics and datatypes from the ROS graph
    std::vector<ros::master::TopicInfo> topicNamesAndTypes;
    if (!ros::master::getTopics(topicNamesAndTypes)) {
      ROS_WARN("Failed to retrieve published topics from ROS master.");
      return;
    }

    std::unordered_set<TopicAndDatatype, PairHash> latestTopics;
    latestTopics.reserve(topicNamesAndTypes.size());
    for (const auto& topicNameAndType : topicNamesAndTypes) {
      const auto& topicName = topicNameAndType.name;
      const auto& datatype = topicNameAndType.datatype;

      // Ignore the topic if it is not on the topic whitelist
      if (isWhitelisted(topicName, _topicWhitelistPatterns)) {
        latestTopics.emplace(topicName, datatype);
      }
    }

    if (const auto numIgnoredTopics = topicNamesAndTypes.size() - latestTopics.size()) {
      ROS_DEBUG(
        "%zu topics have been ignored as they do not match any pattern on the topic whitelist",
        numIgnoredTopics);
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
  }

  void updateAdvertisedServices() {
    std::vector<std::string> serviceNames;
    XmlRpc::XmlRpcValue params, result, payload;
    params[0] = this->getName();
    if (ros::master::execute("getSystemState", params, result, payload, false) &&
        static_cast<int>(result[0]) == 1) {
      const auto& systemState = result[2];
      const auto& services = systemState[2];
      for (int i = 0; i < services.size(); ++i) {
        const std::string serviceName = services[i][0];
        serviceNames.emplace_back(serviceName);
      }
    } else {
      ROS_WARN("Failed to call getSystemState: %s", result.toXml().c_str());
      return;
    }

    std::unique_lock<std::shared_mutex> lock(_servicesMutex);

    // Remove advertisements for services that have been removed
    std::vector<foxglove::ServiceId> servicesToRemove;
    for (const auto& service : _advertisedServices) {
      const auto it =
        std::find_if(serviceNames.begin(), serviceNames.end(), [service](const auto& serviceName) {
          return serviceName == service.second.name;
        });
      if (it == serviceNames.end()) {
        servicesToRemove.push_back(service.first);
      }
    }
    for (auto serviceId : servicesToRemove) {
      _advertisedServices.erase(serviceId);
    }
    _server->removeServices(servicesToRemove);

    // Advertise new services
    std::vector<foxglove::ServiceWithoutId> newServices;
    for (const auto& serviceName : serviceNames) {
      if (std::find_if(_advertisedServices.begin(), _advertisedServices.end(),
                       [&serviceName](const auto& idWithService) {
                         return idWithService.second.name == serviceName;
                       }) != _advertisedServices.end()) {
        continue;  // Already advertised
      }

      auto serviceTypeFuture = retrieveServiceType(serviceName);
      if (serviceTypeFuture.wait_for(std::chrono::milliseconds(
            SERVICE_TYPE_RETRIEVAL_TIMEOUT_MS)) != std::future_status::ready) {
        ROS_WARN("Failed to retrieve type of service %s", serviceName.c_str());
        continue;
      }

      const auto serviceType = serviceTypeFuture.get();
      const auto srvDescription = _rosTypeInfoProvider.getServiceDescription(serviceType);

      foxglove::ServiceWithoutId service;
      service.name = serviceName;
      service.type = serviceType;

      if (srvDescription) {
        service.requestSchema = srvDescription->request->message_definition;
        service.responseSchema = srvDescription->response->message_definition;
      } else {
        ROS_ERROR("Failed to retrieve type information for service '%s' of type '%s'",
                  serviceName.c_str(), serviceType.c_str());

        // We still advertise the channel, but with empty schema.
        service.requestSchema = "";
        service.responseSchema = "";
      }
      newServices.push_back(service);
    }

    const auto serviceIds = _server->addServices(newServices);
    for (size_t i = 0; i < serviceIds.size(); ++i) {
      _advertisedServices.emplace(serviceIds[i], newServices[i]);
    }
  }

  void parameterRequestHandler(const std::vector<std::string>& parameters,
                               const std::optional<std::string>& requestId,
                               foxglove::ConnHandle hdl) {
    std::vector<std::string> parameterNames = parameters;
    if (parameterNames.empty()) {
      getMTNodeHandle().getParamNames(parameterNames);
    }

    std::vector<foxglove::Parameter> params;
    for (const auto& paramName : parameterNames) {
      if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
        ROS_WARN("Parameter '%s' is not whitelisted", paramName.c_str());
        continue;
      }

      try {
        XmlRpc::XmlRpcValue value;
        getMTNodeHandle().getParam(paramName, value);
        params.push_back(fromRosParam(paramName, value));
      } catch (const std::exception& ex) {
        ROS_ERROR("Invalid parameter: %s", ex.what());
      }
    }

    _server->publishParameterValues(hdl, params, requestId);
  }

  void parameterChangeHandler(const std::vector<foxglove::Parameter>& parameters,
                              const std::optional<std::string>& requestId,
                              foxglove::ConnHandle hdl) {
    using foxglove::ParameterType;
    auto nh = this->getMTNodeHandle();
    for (const auto& param : parameters) {
      const auto paramName = param.getName();
      if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
        ROS_WARN("Parameter '%s' is not whitelisted", paramName.c_str());
        continue;
      }

      const auto paramType = param.getType();
      if (paramType == ParameterType::PARAMETER_BOOL) {
        nh.setParam(paramName, param.getValue<bool>());
      } else if (paramType == ParameterType::PARAMETER_INTEGER) {
        nh.setParam(paramName, static_cast<int>(param.getValue<int64_t>()));
      } else if (paramType == ParameterType::PARAMETER_DOUBLE) {
        nh.setParam(paramName, param.getValue<double>());
      } else if (paramType == ParameterType::PARAMETER_STRING) {
        nh.setParam(paramName, param.getValue<std::string>());
      } else if (paramType == ParameterType::PARAMETER_BOOL_ARRAY) {
        nh.setParam(paramName, param.getValue<std::vector<bool>>());
      } else if (paramType == ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto int64Vec = param.getValue<std::vector<int64_t>>();
        std::vector<int> intVec(int64Vec.begin(), int64Vec.end());
        nh.setParam(paramName, intVec);
      } else if (paramType == ParameterType::PARAMETER_DOUBLE_ARRAY) {
        nh.setParam(paramName, param.getValue<std::vector<double>>());
      } else if (paramType == ParameterType::PARAMETER_STRING_ARRAY) {
        nh.setParam(paramName, param.getValue<std::vector<std::string>>());
      } else if (paramType == ParameterType::PARAMETER_NOT_SET) {
        nh.deleteParam(paramName);
      }
    }

    // If a request Id was given, send potentially updated parameters back to client
    if (requestId) {
      std::vector<std::string> parameterNames(parameters.size());
      for (size_t i = 0; i < parameters.size(); ++i) {
        parameterNames[i] = parameters[i].getName();
      }
      parameterRequestHandler(parameterNames, requestId, hdl);
    }
  }

  void parameterSubscriptionHandler(const std::vector<std::string>& parameters,
                                    foxglove::ParameterSubscriptionOperation op,
                                    foxglove::ConnHandle) {
    for (const auto& paramName : parameters) {
      if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
        ROS_WARN("Parameter '%s' is not whitelisted", paramName.c_str());
        continue;
      }

      XmlRpc::XmlRpcValue params, result, payload;
      params[0] = getName() + "2";
      params[1] = xmlrpcServer.getServerURI();
      params[2] = ros::names::resolve(paramName);

      const auto opVerb = (op == foxglove::ParameterSubscriptionOperation::SUBSCRIBE)
                            ? "subscribeParam"
                            : "unsubscribeParam";

      if (ros::master::execute(opVerb, params, result, payload, false)) {
        ROS_DEBUG("%s '%s'", opVerb, paramName.c_str());
      } else {
        ROS_WARN("Failed to %s '%s': %s", opVerb, paramName.c_str(), result.toXml().c_str());
      }
    }
  }

  void parameterUpdates(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    result[0] = 1;
    result[1] = std::string("");
    result[2] = 0;

    if (params.size() != 3) {
      ROS_ERROR("Parameter update called with invalid parameter size: %d", params.size());
      return;
    }

    const std::string paramName = ros::names::clean(params[1]);
    const XmlRpc::XmlRpcValue paramValue = params[2];
    try {
      const auto param = fromRosParam(paramName, paramValue);
      _server->updateParameterValues({param});
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to convert parameter: %s", ex.what());
      return;
    }
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
    const auto receiptTimeNs = msgEvent.getReceiptTime().toNSec();
    _server->sendMessage(clientHandle, channel.id, receiptTimeNs, msg->buffer(), msg->size());
  }

  void serviceRequestHandler(const foxglove::ServiceRequest& request,
                             foxglove::ConnHandle clientHandle) {
    std::shared_lock<std::shared_mutex> lock(_servicesMutex);
    const auto serviceIt = _advertisedServices.find(request.serviceId);
    if (serviceIt == _advertisedServices.end()) {
      ROS_ERROR("Service with id %d does not exist", request.serviceId);
      return;
    }
    const auto& serviceName = serviceIt->second.name;
    const auto& serviceType = serviceIt->second.type;
    ROS_DEBUG("Received a service request for service %s (%s)", serviceName.c_str(),
              serviceType.c_str());

    if (!ros::service::exists(serviceName, false)) {
      ROS_ERROR("Service '%s' does not exist", serviceName.c_str());
      return;
    }

    const auto srvDescription = _rosTypeInfoProvider.getServiceDescription(serviceType);
    if (!srvDescription) {
      ROS_ERROR("Failed to retrieve type information for service %s (%s)", serviceName.c_str(),
                serviceType.c_str());
      return;
    }

    GenericService genReq, genRes;
    genReq.type = genRes.type = serviceType;
    genReq.md5sum = genRes.md5sum = srvDescription->md5;
    genReq.data = request.data;

    if (ros::service::call(serviceName, genReq, genRes)) {
      foxglove::ServiceResponse res;
      res.serviceId = request.serviceId;
      res.callId = request.callId;
      res.encoding = request.encoding;
      res.data = genRes.data;
      _server->sendServiceResponse(clientHandle, res);
    } else {
      ROS_ERROR("Failed to call service %s (%s)", serviceName.c_str(), serviceType.c_str());
    }
  }

  std::unique_ptr<foxglove::ServerInterface> _server;
  ros_babel_fish::IntegratedDescriptionProvider _rosTypeInfoProvider;
  std::vector<std::regex> _topicWhitelistPatterns;
  std::vector<std::regex> _paramWhitelistPatterns;
  std::vector<std::regex> _serviceWhitelistPatterns;
  ros::XMLRPCManager xmlrpcServer;
  std::unordered_map<TopicAndDatatype, foxglove::Channel, PairHash> _advertisedTopics;
  std::unordered_map<foxglove::ChannelId, TopicAndDatatype> _channelToTopicAndDatatype;
  std::unordered_map<foxglove::ChannelId, SubscriptionsByClient> _subscriptions;
  std::unordered_map<foxglove::ServiceId, foxglove::ServiceWithoutId> _advertisedServices;
  PublicationsByClient _clientAdvertisedTopics;
  std::mutex _subscriptionsMutex;
  std::shared_mutex _publicationsMutex;
  std::shared_mutex _servicesMutex;
  ros::Timer _updateTimer;
  size_t _maxUpdateMs = size_t(DEFAULT_MAX_UPDATE_MS);
  size_t _updateCount = 0;
  ros::Subscriber _clockSubscription;
  bool _useSimTime = false;
};

}  // namespace foxglove_bridge

PLUGINLIB_EXPORT_CLASS(foxglove_bridge::FoxgloveBridge, nodelet::Nodelet)
