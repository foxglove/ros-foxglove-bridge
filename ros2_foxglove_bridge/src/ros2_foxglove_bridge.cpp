#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#define ASIO_STANDALONE

#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/message_definition_cache.hpp>
#include <foxglove_bridge/websocket_notls.hpp>
#include <foxglove_bridge/websocket_server.hpp>

constexpr uint16_t DEFAULT_PORT = 8765;
constexpr char DEFAULT_ADDRESS[] = "0.0.0.0";
constexpr int DEFAULT_NUM_THREADS = 0;

using namespace std::chrono_literals;
using namespace std::placeholders;
using LogLevel = foxglove::WebSocketLogLevel;
using Subscription = std::pair<rclcpp::GenericSubscription::SharedPtr, rclcpp::SubscriptionOptions>;
using SubscriptionsByClient = std::map<foxglove::ConnHandle, Subscription, std::owner_less<>>;

class FoxgloveBridge : public rclcpp::Node {
public:
  using TopicAndDatatype = std::pair<std::string, std::string>;

  FoxgloveBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("foxglove_bridge", options)
      , _server("foxglove_bridge", std::bind(&FoxgloveBridge::logHandler, this, _1, _2)) {
    RCLCPP_INFO(this->get_logger(), "Starting %s with %s", this->get_name(),
                foxglove::WebSocketUserAgent());

    auto portDescription = rcl_interfaces::msg::ParameterDescriptor{};
    portDescription.name = "port";
    portDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    portDescription.description = "The TCP port to bind the WebSocket server to";
    portDescription.read_only = true;
    portDescription.additional_constraints =
      "Must be a valid TCP port number, or 0 to use a random port";
    portDescription.integer_range.resize(1);
    portDescription.integer_range[0].from_value = 0;
    portDescription.integer_range[0].to_value = 65535;
    portDescription.integer_range[0].step = 1;
    this->declare_parameter("port", DEFAULT_PORT, portDescription);

    auto addressDescription = rcl_interfaces::msg::ParameterDescriptor{};
    addressDescription.name = "address";
    addressDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    addressDescription.description = "The host address to bind the WebSocket server to";
    addressDescription.read_only = true;
    this->declare_parameter("address", DEFAULT_ADDRESS, addressDescription);

    auto numThreadsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    numThreadsDescription.name = "num_threads";
    numThreadsDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    numThreadsDescription.description =
      "The number of threads to use for the ROS node executor. 0 means one thread per CPU core.";
    numThreadsDescription.read_only = true;
    numThreadsDescription.additional_constraints = "Must be a non-negative integer";
    numThreadsDescription.integer_range.resize(1);
    numThreadsDescription.integer_range[0].from_value = 0;
    numThreadsDescription.integer_range[0].to_value = INT32_MAX;
    numThreadsDescription.integer_range[0].step = 1;
    this->declare_parameter("num_threads", DEFAULT_NUM_THREADS, numThreadsDescription);

    _server.setSubscribeHandler(std::bind(&FoxgloveBridge::subscribeHandler, this, _1, _2));
    _server.setUnsubscribeHandler(std::bind(&FoxgloveBridge::unsubscribeHandler, this, _1, _2));

    auto address = this->get_parameter("address").as_string();
    uint16_t port = uint16_t(this->get_parameter("port").as_int());
    _server.start(address, port);

    // Get the actual port we bound to
    uint16_t listeningPort = _server.localEndpoint()->port();
    if (port != listeningPort) {
      RCLCPP_DEBUG(this->get_logger(), "Reassigning \"port\" parameter from %d to %d", port,
                   listeningPort);
      this->set_parameter(rclcpp::Parameter{"port", listeningPort});
    }

    // Start the thread polling for rosgraph changes
    _rosgraphPollThread =
      std::make_unique<std::thread>(std::bind(&FoxgloveBridge::rosgraphPollThread, this));
  }

  ~FoxgloveBridge() {
    RCLCPP_INFO(this->get_logger(), "Shutting down %s", this->get_name());
    if (_rosgraphPollThread) {
      _rosgraphPollThread->join();
    }
    _server.stop();
    RCLCPP_INFO(this->get_logger(), "Shutdown complete");
  }

  size_t numThreads() {
    int numThreads;
    rclcpp::spin_some(this->get_node_base_interface());
    this->get_parameter_or("num_threads", numThreads, DEFAULT_NUM_THREADS);
    return size_t(numThreads);
  }

  void rosgraphPollThread() {
    updateAdvertisedTopics();

    auto graphEvent = this->get_graph_event();
    while (rclcpp::ok()) {
      this->wait_for_graph_change(graphEvent, 200ms);
      bool triggered = graphEvent->check_and_clear();
      if (triggered) {
        RCLCPP_DEBUG(this->get_logger(), "rosgraph change detected");
        updateAdvertisedTopics();
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
    bool hasClockTopic = false;
    for (const auto& [topicName, datatypes] : topicNamesAndTypes) {
      for (const auto& datatype : datatypes) {
        // Check if a /clock topic is published
        if (topicName == "/clock" && datatype == "rosgraph_msgs/msg/Clock") {
          hasClockTopic = true;
        }
        latestTopics.emplace(topicName, datatype);
      }
    }

    // Enable or disable simulated time based on the presence of a /clock topic
    if (!_useSimTime && hasClockTopic) {
      RCLCPP_INFO(this->get_logger(), "/clock topic found, using simulated time");
      _useSimTime = true;
      _clockSubscription = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS{rclcpp::KeepLast(1)}.best_effort(),
        [&](std::shared_ptr<rosgraph_msgs::msg::Clock> msg) {
          _simTimeNs = uint64_t(rclcpp::Time{msg->clock}.nanoseconds());
        });
    } else if (_useSimTime && !hasClockTopic) {
      RCLCPP_WARN(this->get_logger(), "/clock topic disappeared");
      _useSimTime = false;
      _clockSubscription.reset();
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
        _server.removeChannel(channel.id);

        // Remove the subscription for this topic, if any
        _subscriptions.erase(channel.id);

        // Remove this topic+datatype tuple
        _channelToTopicAndDatatype.erase(channel.id);
        _advertisedTopics.erase(topicAndDatatype);

        RCLCPP_DEBUG(this->get_logger(), "Removed channel %d for topic \"%s\" (%s)", channel.id,
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

          auto channel = foxglove::Channel{_server.addChannel(newChannel), newChannel};
          RCLCPP_DEBUG(this->get_logger(), "Advertising channel %d for topic \"%s\" (%s)",
                       channel.id, channel.topic.c_str(), channel.schemaName.c_str());

          // Add a mapping from the topic+datatype tuple to the channel, and channel ID to the
          // topic+datatype tuple
          _advertisedTopics.emplace(topicAndDatatype, std::move(channel));
          _channelToTopicAndDatatype.emplace(channel.id, topicAndDatatype);
        } catch (const foxglove::DefinitionNotFoundError& err) {
          RCLCPP_WARN(this->get_logger(), "Could not find definition for topic \"%s\" (%s)",
                      topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str());

          // Add a mapping from the topic+datatype tuple to a dummy channel so we don't repeatedly
          // try to load the message definition
          auto channel = foxglove::Channel{0, newChannel};
          _advertisedTopics.emplace(topicAndDatatype, std::move(channel));
        }
      }
    }

    if (newTopics.size() > 0) {
      _server.broadcastChannels();
    }
  }

private:
  struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
      return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

  foxglove::Server<foxglove::WebSocketNoTls> _server;
  foxglove::MessageDefinitionCache _messageDefinitionCache;
  std::unordered_map<TopicAndDatatype, foxglove::Channel, PairHash> _advertisedTopics;
  std::unordered_map<foxglove::ChannelId, TopicAndDatatype> _channelToTopicAndDatatype;
  std::unordered_map<foxglove::ChannelId, SubscriptionsByClient> _subscriptions;
  std::mutex _subscriptionsMutex;
  std::unique_ptr<std::thread> _rosgraphPollThread;
  std::shared_ptr<rclcpp::Subscription<rosgraph_msgs::msg::Clock>> _clockSubscription;
  std::atomic<uint64_t> _simTimeNs = 0;
  std::atomic<bool> _useSimTime = false;

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
    subscriptionOptions.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create a QoS profile that only keeps the last message since we don't need to keep a history,
    // and use reliable delivery if all publishers are reliable otherwise best-effort
    bool reliable = true;
    bool durationVolatile = true;
    size_t depth = 1;
    for (const auto& publisher : this->get_publishers_info_by_topic(topic)) {
      const auto& qos = publisher.qos_profile();
      if (qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort) {
        reliable = false;
      }
      if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        durationVolatile = false;
      }
      depth += qos.depth();
    }

    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    if (!reliable) {
      qos.best_effort();
    }
    if (!durationVolatile) {
      qos.transient_local();
    }

    try {
      auto subscriber = this->create_generic_subscription(
        topic, datatype, qos,
        std::bind(&FoxgloveBridge::rosMessageHandler, this, channel, clientHandle, _1),
        subscriptionOptions);
      subscriptionsByClient.emplace(clientHandle,
                                    std::make_pair(std::move(subscriber), subscriptionOptions));

      if (firstSubscription) {
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic \"%s\" (%s) on channel %d",
                    topic.c_str(), datatype.c_str(), channelId);

      } else {
        RCLCPP_INFO(this->get_logger(), "Added subscriber #%ld to topic \"%s\" (%s) on channel %d",
                    subscriptionsByClient.size(), topic.c_str(), datatype.c_str(), channelId);
      }
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
                  "Removed one subscription from channel %d (%ld subscription(s) left)", channelId,
                  subscriptionsByClient.size());
    }
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
    const auto timestamp = currentTimeNs();
    const auto payload =
      std::string_view{reinterpret_cast<const char*>(msg->get_rcl_serialized_message().buffer),
                       msg->get_rcl_serialized_message().buffer_length};
    _server.sendMessage(clientHandle, channel.id, timestamp, payload);
  }

  uint64_t currentTimeNs() {
    return _useSimTime ? _simTimeNs.load() : uint64_t(this->now().nanoseconds());
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto bridgeNode = std::make_shared<FoxgloveBridge>();
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{},
                                                    bridgeNode->numThreads()};
  executor.add_node(bridgeNode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
