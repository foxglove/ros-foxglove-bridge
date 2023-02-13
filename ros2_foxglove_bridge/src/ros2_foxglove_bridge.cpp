#include <chrono>
#include <memory>
#include <regex>
#include <thread>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#define ASIO_STANDALONE

#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/generic_client.hpp>
#include <foxglove_bridge/message_definition_cache.hpp>
#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/parameter_interface.hpp>
#include <foxglove_bridge/websocket_server.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using LogLevel = foxglove::WebSocketLogLevel;
using Subscription = rclcpp::GenericSubscription::SharedPtr;
using SubscriptionsByClient = std::map<foxglove::ConnHandle, Subscription, std::owner_less<>>;
using Publication = rclcpp::GenericPublisher::SharedPtr;
using ClientPublications = std::unordered_map<foxglove::ClientChannelId, Publication>;
using PublicationsByClient = std::map<foxglove::ConnHandle, ClientPublications, std::owner_less<>>;

namespace foxglove_bridge {

class FoxgloveBridge : public rclcpp::Node {
public:
  using TopicAndDatatype = std::pair<std::string, std::string>;

  FoxgloveBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("foxglove_bridge", options) {
    RCLCPP_INFO(this->get_logger(), "Starting %s with %s", this->get_name(),
                foxglove::WebSocketUserAgent());

    declareParameters(this);

    const auto port = static_cast<uint16_t>(this->get_parameter(PARAM_PORT).as_int());
    const auto address = this->get_parameter(PARAM_ADDRESS).as_string();
    const auto send_buffer_limit =
      static_cast<size_t>(this->get_parameter(PARAM_SEND_BUFFER_LIMIT).as_int());
    const auto useTLS = this->get_parameter(PARAM_USETLS).as_bool();
    const auto certfile = this->get_parameter(PARAM_CERTFILE).as_string();
    const auto keyfile = this->get_parameter(PARAM_KEYFILE).as_string();
    _maxQosDepth = static_cast<size_t>(this->get_parameter(PARAM_MAX_QOS_DEPTH).as_int());
    const auto topicWhiteList = this->get_parameter(PARAM_TOPIC_WHITELIST).as_string_array();
    _topicWhitelistPatterns = parseRegexStrings(this, topicWhiteList);
    const auto serviceWhiteList = this->get_parameter(PARAM_SERVICE_WHITELIST).as_string_array();
    _serviceWhitelistPatterns = parseRegexStrings(this, serviceWhiteList);
    const auto paramWhiteList = this->get_parameter(PARAM_PARAMETER_WHITELIST).as_string_array();
    const auto paramWhitelistPatterns = parseRegexStrings(this, paramWhiteList);
    const auto useCompression = this->get_parameter(PARAM_USE_COMPRESSION).as_bool();
    _useSimTime = this->get_parameter("use_sim_time").as_bool();

    _paramInterface = std::make_shared<ParameterInterface>(this, paramWhitelistPatterns);

    const auto logHandler = std::bind(&FoxgloveBridge::logHandler, this, _1, _2);
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
    serverOptions.supportedEncodings = {"cdr"};
    serverOptions.metadata = {{"ROS_DISTRO", std::getenv("ROS_DISTRO")}};
    serverOptions.sendBufferLimitBytes = send_buffer_limit;
    serverOptions.sessionId = std::to_string(std::time(nullptr));
    serverOptions.useCompression = useCompression;

    if (useTLS) {
      serverOptions.certfile = certfile;
      serverOptions.keyfile = keyfile;
      _server = std::make_unique<foxglove::Server<foxglove::WebSocketTls>>(
        "foxglove_bridge", std::move(logHandler), serverOptions);
    } else {
      _server = std::make_unique<foxglove::Server<foxglove::WebSocketNoTls>>(
        "foxglove_bridge", std::move(logHandler), serverOptions);
    }
    _server->setSubscribeHandler(std::bind(&FoxgloveBridge::subscribeHandler, this, _1, _2));
    _server->setUnsubscribeHandler(std::bind(&FoxgloveBridge::unsubscribeHandler, this, _1, _2));
    _server->setClientAdvertiseHandler(
      std::bind(&FoxgloveBridge::clientAdvertiseHandler, this, _1, _2));
    _server->setClientUnadvertiseHandler(
      std::bind(&FoxgloveBridge::clientUnadvertiseHandler, this, _1, _2));
    _server->setClientMessageHandler(
      std::bind(&FoxgloveBridge::clientMessageHandler, this, _1, _2));
    _server->setParameterRequestHandler(
      std::bind(&FoxgloveBridge::parameterRequestHandler, this, _1, _2, _3));
    _server->setParameterChangeHandler(
      std::bind(&FoxgloveBridge::parameterChangeHandler, this, _1, _2, _3));
    _server->setParameterSubscriptionHandler(
      std::bind(&FoxgloveBridge::parameterSubscriptionHandler, this, _1, _2, _3));
    _server->setServiceRequestHandler(
      std::bind(&FoxgloveBridge::serviceRequestHandler, this, _1, _2));

    _paramInterface->setParamUpdateCallback(std::bind(&FoxgloveBridge::parameterUpdates, this, _1));

    _server->start(address, port);

    // Get the actual port we bound to
    uint16_t listeningPort = _server->localEndpoint()->port();
    if (port != listeningPort) {
      RCLCPP_DEBUG(this->get_logger(), "Reassigning \"port\" parameter from %d to %d", port,
                   listeningPort);
      this->set_parameter(rclcpp::Parameter{PARAM_PORT, listeningPort});
    }

    // Start the thread polling for rosgraph changes
    _rosgraphPollThread =
      std::make_unique<std::thread>(std::bind(&FoxgloveBridge::rosgraphPollThread, this));

    _subscriptionCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    _clientPublishCallbackGroup =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    _servicesCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    if (_useSimTime) {
      _clockSubscription = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS{rclcpp::KeepLast(1)}.best_effort(),
        [&](std::shared_ptr<rosgraph_msgs::msg::Clock> msg) {
          const auto timestamp = rclcpp::Time{msg->clock}.nanoseconds();
          assert(timestamp >= 0 && "Timestamp is negative");
          _server->broadcastTime(static_cast<uint64_t>(timestamp));
        });
    }
  }

  ~FoxgloveBridge() {
    RCLCPP_INFO(this->get_logger(), "Shutting down %s", this->get_name());
    if (_rosgraphPollThread) {
      _rosgraphPollThread->join();
    }
    _server->stop();
    RCLCPP_INFO(this->get_logger(), "Shutdown complete");
  }

  void rosgraphPollThread() {
    updateAdvertisedTopics();
    updateAdvertisedServices();

    auto graphEvent = this->get_graph_event();
    while (rclcpp::ok()) {
      this->wait_for_graph_change(graphEvent, 200ms);
      bool triggered = graphEvent->check_and_clear();
      if (triggered) {
        RCLCPP_DEBUG(this->get_logger(), "rosgraph change detected");
        updateAdvertisedTopics();
        updateAdvertisedServices();
        // Graph changes tend to come in batches, so wait a bit before checking again
        std::this_thread::sleep_for(500ms);
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "rosgraph polling thread exiting");
  }

  void updateAdvertisedTopics() {
    if (!rclcpp::ok()) {
      return;
    }

    // Get the current list of visible topics and datatypes from the ROS graph
    auto topicNamesAndTypes = this->get_node_graph_interface()->get_topic_names_and_types();
    std::unordered_set<TopicAndDatatype, PairHash> latestTopics;
    latestTopics.reserve(topicNamesAndTypes.size());
    for (const auto& topicNamesAndType : topicNamesAndTypes) {
      const auto& topicName = topicNamesAndType.first;
      const auto& datatypes = topicNamesAndType.second;

      // Ignore the topic if it is not on the topic whitelist
      if (std::find_if(_topicWhitelistPatterns.begin(), _topicWhitelistPatterns.end(),
                       [&topicName](const auto& regex) {
                         return std::regex_match(topicName, regex);
                       }) != _topicWhitelistPatterns.end()) {
        for (const auto& datatype : datatypes) {
          latestTopics.emplace(topicName, datatype);
        }
      }
    }

    if (const auto numIgnoredTopics = topicNamesAndTypes.size() - latestTopics.size()) {
      RCLCPP_DEBUG(
        this->get_logger(),
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

        RCLCPP_INFO(this->get_logger(), "Removed channel %d for topic \"%s\" (%s)", channel.id,
                    topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str());
      }

      // Advertise new topics
      for (const auto& topicAndDatatype : newTopics) {
        foxglove::ChannelWithoutId newChannel{};
        newChannel.topic = topicAndDatatype.first;
        newChannel.schemaName = topicAndDatatype.second;

        try {
          auto [format, schema] = _messageDefinitionCache.get_full_text(topicAndDatatype.second);
          switch (format) {
            case foxglove::MessageDefinitionFormat::MSG:
              newChannel.encoding = "cdr";
              newChannel.schema = schema;
              break;
            case foxglove::MessageDefinitionFormat::IDL:
              RCLCPP_WARN(this->get_logger(),
                          "IDL message definition format cannot be communicated over ws-protocol. "
                          "Topic \"%s\" (%s) may not decode correctly in clients",
                          topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str());
              newChannel.encoding = "cdr";
              newChannel.schema = schema;
              break;
          }

        } catch (const foxglove::DefinitionNotFoundError& err) {
          RCLCPP_WARN(this->get_logger(), "Could not find definition for type %s: %s",
                      topicAndDatatype.second.c_str(), err.what());
          // We still advertise the channel, but with an emtpy schema
          newChannel.schema = "";
        } catch (const std::exception& err) {
          RCLCPP_WARN(this->get_logger(), "Failed to add channel for topic \"%s\" (%s): %s",
                      topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str(), err.what());
          continue;
        }

        auto channel = foxglove::Channel{_server->addChannel(newChannel), newChannel};
        RCLCPP_DEBUG(this->get_logger(), "Advertising channel %d for topic \"%s\" (%s)", channel.id,
                     channel.topic.c_str(), channel.schemaName.c_str());

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
    if (!rclcpp::ok()) {
      return;
    }

    // Get the current list of visible services and datatypes from the ROS graph
    const auto serviceNamesAndTypes =
      this->get_node_graph_interface()->get_service_names_and_types();

    std::lock_guard<std::mutex> lock(_servicesMutex);

    // Remove advertisements for services that have been removed
    std::vector<foxglove::ServiceId> servicesToRemove;
    for (const auto& service : _advertisedServices) {
      const auto it = std::find_if(serviceNamesAndTypes.begin(), serviceNamesAndTypes.end(),
                                   [service](const auto& serviceNameAndTypes) {
                                     return serviceNameAndTypes.first == service.second.name;
                                   });
      if (it == serviceNamesAndTypes.end()) {
        servicesToRemove.push_back(service.first);
      }
    }
    for (auto serviceId : servicesToRemove) {
      _advertisedServices.erase(serviceId);
    }
    _server->removeServices(servicesToRemove);

    // Advertise new services
    std::vector<foxglove::ServiceWithoutId> newServices;
    for (const auto& serviceNamesAndType : serviceNamesAndTypes) {
      const auto& serviceName = serviceNamesAndType.first;
      const auto& datatypes = serviceNamesAndType.second;

      // Ignore the service if it's already advertised
      if (std::find_if(_advertisedServices.begin(), _advertisedServices.end(),
                       [serviceName](const auto& idWithService) {
                         return idWithService.second.name == serviceName;
                       }) != _advertisedServices.end()) {
        continue;
      }

      // Ignore the service if it is not on the service whitelist
      if (std::find_if(_serviceWhitelistPatterns.begin(), _serviceWhitelistPatterns.end(),
                       [&serviceName](const auto& regex) {
                         return std::regex_match(serviceName, regex);
                       }) == _serviceWhitelistPatterns.end()) {
        continue;
      }

      foxglove::ServiceWithoutId service;
      service.name = serviceName;
      service.type = datatypes.front();

      try {
        auto [format, reqSchema] = _messageDefinitionCache.get_full_text(service.type + "_Request");
        auto resSchema = _messageDefinitionCache.get_full_text(service.type + "_Response").second;
        switch (format) {
          case foxglove::MessageDefinitionFormat::MSG:
            service.requestSchema = reqSchema;
            service.responseSchema = resSchema;
            break;
          case foxglove::MessageDefinitionFormat::IDL:
            RCLCPP_WARN(this->get_logger(),
                        "IDL message definition format cannot be communicated over ws-protocol. "
                        "Service \"%s\" (%s) may not decode correctly in clients",
                        service.name.c_str(), service.type.c_str());
            service.requestSchema = reqSchema;
            service.responseSchema = resSchema;
            break;
        }
      } catch (const foxglove::DefinitionNotFoundError& err) {
        RCLCPP_WARN(this->get_logger(), "Could not find definition for type %s: %s",
                    service.type.c_str(), err.what());
        // We still advertise the service, but with an emtpy schema
        service.requestSchema = "";
        service.responseSchema = "";
      } catch (const std::exception& err) {
        RCLCPP_WARN(this->get_logger(), "Failed to add service \"%s\" (%s): %s",
                    service.name.c_str(), service.type.c_str(), err.what());
        continue;
      }

      newServices.push_back(service);
    }

    const auto serviceIds = _server->addServices(newServices);
    for (size_t i = 0; i < serviceIds.size(); ++i) {
      _advertisedServices.emplace(serviceIds[i], newServices[i]);
    }
  }

private:
  struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
      return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

  std::unique_ptr<foxglove::ServerInterface> _server;
  foxglove::MessageDefinitionCache _messageDefinitionCache;
  std::vector<std::regex> _topicWhitelistPatterns;
  std::vector<std::regex> _serviceWhitelistPatterns;
  std::shared_ptr<ParameterInterface> _paramInterface;
  std::unordered_map<TopicAndDatatype, foxglove::Channel, PairHash> _advertisedTopics;
  std::unordered_map<foxglove::ServiceId, foxglove::ServiceWithoutId> _advertisedServices;
  std::unordered_map<foxglove::ChannelId, TopicAndDatatype> _channelToTopicAndDatatype;
  std::unordered_map<foxglove::ChannelId, SubscriptionsByClient> _subscriptions;
  PublicationsByClient _clientAdvertisedTopics;
  std::unordered_map<foxglove::ServiceId, GenericClient::SharedPtr> _serviceClients;
  rclcpp::CallbackGroup::SharedPtr _subscriptionCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr _clientPublishCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr _servicesCallbackGroup;
  std::mutex _subscriptionsMutex;
  std::mutex _clientAdvertisementsMutex;
  std::mutex _servicesMutex;
  std::unique_ptr<std::thread> _rosgraphPollThread;
  size_t _maxQosDepth = DEFAULT_MAX_QOS_DEPTH;
  std::shared_ptr<rclcpp::Subscription<rosgraph_msgs::msg::Clock>> _clockSubscription;
  bool _useSimTime = false;

  void subscribeHandler(foxglove::ChannelId channelId, foxglove::ConnHandle clientHandle) {
    std::lock_guard<std::mutex> lock(_subscriptionsMutex);
    auto it = _channelToTopicAndDatatype.find(channelId);
    if (it == _channelToTopicAndDatatype.end()) {
      RCLCPP_WARN(this->get_logger(), "Received subscribe request for unknown channel %d",
                  channelId);
      return;
    }
    auto& topicAndDatatype = it->second;
    auto topic = topicAndDatatype.first;
    auto datatype = topicAndDatatype.second;
    auto it2 = _advertisedTopics.find(topicAndDatatype);
    if (it2 == _advertisedTopics.end()) {
      RCLCPP_ERROR(this->get_logger(), "Channel %d for topic \"%s\" (%s) is not advertised",
                   channelId, topic.c_str(), datatype.c_str());
      return;
    }
    const auto& channel = it2->second;

    // Get client subscriptions for this channel or insert an empty map.
    auto [subscriptionsIt, firstSubscription] =
      _subscriptions.emplace(channelId, SubscriptionsByClient());
    auto& subscriptionsByClient = subscriptionsIt->second;

    if (!firstSubscription &&
        subscriptionsByClient.find(clientHandle) != subscriptionsByClient.end()) {
      RCLCPP_WARN(this->get_logger(), "Client is already subscribed to channel %d", channelId);
      return;
    }

    rclcpp::SubscriptionEventCallbacks eventCallbacks;
    eventCallbacks.incompatible_qos_callback = [&](const rclcpp::QOSRequestedIncompatibleQoSInfo&) {
      RCLCPP_ERROR(this->get_logger(), "Incompatible subscriber QoS settings for topic \"%s\" (%s)",
                   topic.c_str(), datatype.c_str());
    };

    rclcpp::SubscriptionOptions subscriptionOptions;
    subscriptionOptions.event_callbacks = eventCallbacks;
    subscriptionOptions.callback_group = _subscriptionCallbackGroup;

    // Select an appropriate subscription QOS profile. This is similar to how ros2 topic echo
    // does it:
    // https://github.com/ros2/ros2cli/blob/619b3d1c9/ros2topic/ros2topic/verb/echo.py#L137-L194
    size_t depth = 0;
    size_t reliabilityReliableEndpointsCount = 0;
    size_t durabilityTransientLocalEndpointsCount = 0;

    const auto publisherInfo = this->get_publishers_info_by_topic(topic);
    for (const auto& publisher : publisherInfo) {
      const auto& qos = publisher.qos_profile();
      if (qos.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
        ++reliabilityReliableEndpointsCount;
      }
      if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        ++durabilityTransientLocalEndpointsCount;
      }
      depth = std::min(_maxQosDepth, depth + qos.depth());
    }

    rclcpp::QoS qos{rclcpp::KeepLast(std::max(depth, 1lu))};

    // If all endpoints are reliable, ask for reliable
    if (reliabilityReliableEndpointsCount == publisherInfo.size()) {
      qos.reliable();
    } else {
      if (reliabilityReliableEndpointsCount > 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Some, but not all, publishers on topic '%s' are offering QoSReliabilityPolicy.RELIABLE. "
          "Falling back to QoSReliabilityPolicy.BEST_EFFORT as it will connect to all publishers",
          topic.c_str());
      }
      qos.best_effort();
    }

    // If all endpoints are transient_local, ask for transient_local
    if (durabilityTransientLocalEndpointsCount == publisherInfo.size()) {
      qos.transient_local();
    } else {
      if (durabilityTransientLocalEndpointsCount > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Some, but not all, publishers on topic '%s' are offering "
                    "QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to "
                    "QoSDurabilityPolicy.VOLATILE as it will connect to all publishers",
                    topic.c_str());
      }
      qos.durability_volatile();
    }

    if (firstSubscription) {
      RCLCPP_INFO(this->get_logger(), "Subscribing to topic \"%s\" (%s) on channel %d",
                  topic.c_str(), datatype.c_str(), channelId);

    } else {
      RCLCPP_INFO(this->get_logger(), "Adding subscriber #%zu to topic \"%s\" (%s) on channel %d",
                  subscriptionsByClient.size(), topic.c_str(), datatype.c_str(), channelId);
    }

    try {
      auto subscriber = this->create_generic_subscription(
        topic, datatype, qos,
        std::bind(&FoxgloveBridge::rosMessageHandler, this, channel, clientHandle, _1),
        subscriptionOptions);
      subscriptionsByClient.emplace(clientHandle, std::move(subscriber));
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to topic \"%s\" (%s): %s",
                   topic.c_str(), datatype.c_str(), ex.what());
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
      RCLCPP_WARN(this->get_logger(), "Received unsubscribe request for unknown channel %d",
                  channelId);
      return;
    }

    auto subscriptionsIt = _subscriptions.find(channelId);
    if (subscriptionsIt == _subscriptions.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "Received unsubscribe request for channel %d that was not subscribed to",
                  channelId);
      return;
    }

    auto& subscriptionsByClient = subscriptionsIt->second;
    const auto clientSubscription = subscriptionsByClient.find(clientHandle);
    if (clientSubscription == subscriptionsByClient.end()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received unsubscribe request for channel %d from a client that was not subscribed to this "
        "channel",
        channelId);
      return;
    }

    subscriptionsByClient.erase(clientSubscription);
    if (subscriptionsByClient.empty()) {
      RCLCPP_INFO(this->get_logger(), "Unsubscribing from topic \"%s\" (%s) on channel %d",
                  topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str(), channelId);
      _subscriptions.erase(it2);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Removed one subscription from channel %d (%zu subscription(s) left)", channelId,
                  subscriptionsByClient.size());
    }
  }

  void clientAdvertiseHandler(const foxglove::ClientAdvertisement& advertisement,
                              foxglove::ConnHandle hdl) {
    std::lock_guard<std::mutex> lock(_clientAdvertisementsMutex);

    // Get client publications or insert an empty map.
    auto [clientPublicationsIt, isFirstPublication] =
      _clientAdvertisedTopics.emplace(hdl, ClientPublications());

    auto& clientPublications = clientPublicationsIt->second;

    if (!isFirstPublication &&
        clientPublications.find(advertisement.channelId) != clientPublications.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "Received client advertisement from %s for channel %d it had already advertised",
                  _server->remoteEndpointString(hdl).c_str(), advertisement.channelId);
      return;
    }

    // Create a new topic advertisement
    const auto& topicName = advertisement.topic;
    const auto& topicType = advertisement.schemaName;
    rclcpp::QoS qos{rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)};
    rclcpp::PublisherOptions publisherOptions{};
    publisherOptions.callback_group = _clientPublishCallbackGroup;
    auto publisher = this->create_generic_publisher(topicName, topicType, qos, publisherOptions);

    RCLCPP_INFO(this->get_logger(), "Client %s is advertising \"%s\" (%s) on channel %d",
                _server->remoteEndpointString(hdl).c_str(), topicName.c_str(), topicType.c_str(),
                advertisement.channelId);

    // Store the new topic advertisement
    clientPublications.emplace(advertisement.channelId, std::move(publisher));
  }

  void clientUnadvertiseHandler(foxglove::ChannelId channelId, foxglove::ConnHandle hdl) {
    std::lock_guard<std::mutex> lock(_clientAdvertisementsMutex);

    auto it = _clientAdvertisedTopics.find(hdl);
    if (it == _clientAdvertisedTopics.end()) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Ignoring client unadvertisement from %s for unknown channel %d, client has no "
                   "advertised topics",
                   _server->remoteEndpointString(hdl).c_str(), channelId);
      return;
    }

    auto& clientPublications = it->second;
    auto it2 = clientPublications.find(channelId);
    if (it2 == clientPublications.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring client unadvertisement from %s for unknown channel %d, client has %zu "
                  "advertised topic(s)",
                  _server->remoteEndpointString(hdl).c_str(), channelId, clientPublications.size());
      return;
    }

    const auto& publisher = it2->second;
    RCLCPP_INFO(this->get_logger(),
                "Client %s is no longer advertising %s (%zu subscribers) on channel %d",
                _server->remoteEndpointString(hdl).c_str(), publisher->get_topic_name(),
                publisher->get_subscription_count(), channelId);

    clientPublications.erase(it2);
    if (clientPublications.empty()) {
      _clientAdvertisedTopics.erase(it);
    }
  }

  void clientMessageHandler(const foxglove::ClientMessage& message, foxglove::ConnHandle hdl) {
    // Get the publisher
    rclcpp::GenericPublisher::SharedPtr publisher;
    {
      const auto channelId = message.advertisement.channelId;
      std::lock_guard<std::mutex> lock(_clientAdvertisementsMutex);

      auto it = _clientAdvertisedTopics.find(hdl);
      if (it == _clientAdvertisedTopics.end()) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropping client message from %s for unknown channel %d, client has no "
                    "advertised topics",
                    _server->remoteEndpointString(hdl).c_str(), channelId);
        return;
      }

      auto& clientPublications = it->second;
      auto it2 = clientPublications.find(channelId);
      if (it2 == clientPublications.end()) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropping client message from %s for unknown channel %d, client has %zu "
                    "advertised topic(s)",
                    _server->remoteEndpointString(hdl).c_str(), channelId,
                    clientPublications.size());
        return;
      }
      publisher = it2->second;
    }

    // Copy the message payload into a SerializedMessage object
    rclcpp::SerializedMessage serializedMessage{message.getLength()};
    auto& rclSerializedMsg = serializedMessage.get_rcl_serialized_message();
    std::memcpy(rclSerializedMsg.buffer, message.getData(), message.getLength());
    rclSerializedMsg.buffer_length = message.getLength();

    // Publish the message
    publisher->publish(serializedMessage);
  }

  void parameterChangeHandler(const std::vector<foxglove::Parameter>& parameters,
                              const std::optional<std::string>& requestId,
                              foxglove::ConnHandle hdl) {
    _paramInterface->setParams(parameters, std::chrono::seconds(5));

    // If a request Id was given, send potentially updated parameters back to client
    if (requestId) {
      std::vector<std::string> parameterNames(parameters.size());
      for (size_t i = 0; i < parameters.size(); ++i) {
        parameterNames[i] = parameters[i].getName();
      }
      parameterRequestHandler(parameterNames, requestId, hdl);
    }
  }

  void parameterRequestHandler(const std::vector<std::string>& parameters,
                               const std::optional<std::string>& requestId,
                               foxglove::ConnHandle hdl) {
    const auto params = _paramInterface->getParams(parameters, std::chrono::seconds(5));
    _server->publishParameterValues(hdl, params, requestId);
  }

  void parameterSubscriptionHandler(const std::vector<std::string>& parameters,
                                    foxglove::ParameterSubscriptionOperation op,
                                    foxglove::ConnHandle) {
    if (op == foxglove::ParameterSubscriptionOperation::SUBSCRIBE) {
      _paramInterface->subscribeParams(parameters);
    } else {
      _paramInterface->unsubscribeParams(parameters);
    }
  }

  void parameterUpdates(const std::vector<foxglove::Parameter>& parameters) {
    _server->updateParameterValues(parameters);
  }

  void logHandler(LogLevel level, char const* msg) {
    switch (level) {
      case LogLevel::Debug:
        RCLCPP_DEBUG(this->get_logger(), "[WS] %s", msg);
        break;
      case LogLevel::Info:
        RCLCPP_INFO(this->get_logger(), "[WS] %s", msg);
        break;
      case LogLevel::Warn:
        RCLCPP_WARN(this->get_logger(), "[WS] %s", msg);
        break;
      case LogLevel::Error:
        RCLCPP_ERROR(this->get_logger(), "[WS] %s", msg);
        break;
      case LogLevel::Critical:
        RCLCPP_FATAL(this->get_logger(), "[WS] %s", msg);
        break;
    }
  }

  void rosMessageHandler(const foxglove::Channel& channel, foxglove::ConnHandle clientHandle,
                         std::shared_ptr<rclcpp::SerializedMessage> msg) {
    // NOTE: Do not call any RCLCPP_* logging functions from this function. Otherwise, subscribing
    // to `/rosout` will cause a feedback loop
    const auto timestamp = this->now().nanoseconds();
    assert(timestamp >= 0 && "Timestamp is negative");
    const auto rclSerializedMsg = msg->get_rcl_serialized_message();
    _server->sendMessage(clientHandle, channel.id, static_cast<uint64_t>(timestamp),
                         rclSerializedMsg.buffer, rclSerializedMsg.buffer_length);
  }

  void serviceRequestHandler(const foxglove::ServiceRequest& request,
                             foxglove::ConnHandle clientHandle) {
    RCLCPP_DEBUG(this->get_logger(), "Received a request for service %d", request.serviceId);

    std::lock_guard<std::mutex> lock(_servicesMutex);
    const auto serviceIt = _advertisedServices.find(request.serviceId);
    if (serviceIt == _advertisedServices.end()) {
      RCLCPP_ERROR(this->get_logger(), "Service with id '%d' does not exist", request.serviceId);
      return;
    }

    auto clientIt = _serviceClients.find(request.serviceId);
    if (clientIt == _serviceClients.end()) {
      try {
        auto clientOptions = rcl_client_get_default_options();
        auto genClient = GenericClient::make_shared(
          this->get_node_base_interface().get(), this->get_node_graph_interface(),
          serviceIt->second.name, serviceIt->second.type, clientOptions);
        clientIt = _serviceClients.emplace(request.serviceId, std::move(genClient)).first;
        this->get_node_services_interface()->add_client(clientIt->second, _servicesCallbackGroup);
      } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "Failed to create service client for service %d (%s): %s",
                     request.serviceId, serviceIt->second.name.c_str(), ex.what());
        return;
      }
    }

    auto client = clientIt->second;
    if (!client->wait_for_service(1s)) {
      RCLCPP_ERROR(get_logger(), "Service %d (%s) is not available", request.serviceId,
                   serviceIt->second.name.c_str());
      return;
    }

    auto reqMessage = std::make_shared<rclcpp::SerializedMessage>(request.data.size());
    auto& rclSerializedMsg = reqMessage->get_rcl_serialized_message();
    std::memcpy(rclSerializedMsg.buffer, request.data.data(), request.data.size());
    rclSerializedMsg.buffer_length = request.data.size();

    auto responseReceivedCallback = [this, request,
                                     clientHandle](GenericClient::SharedFuture future) {
      const auto serializedResponseMsg = future.get()->get_rcl_serialized_message();
      foxglove::ServiceRequest response{request.serviceId, request.callId, request.encoding,
                                        std::vector<uint8_t>(serializedResponseMsg.buffer_length)};
      std::memcpy(response.data.data(), serializedResponseMsg.buffer,
                  serializedResponseMsg.buffer_length);
      _server->sendServiceResponse(clientHandle, response);
    };
    client->async_send_request(reqMessage, responseReceivedCallback);
  }
};

}  // namespace foxglove_bridge

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(foxglove_bridge::FoxgloveBridge)
