#include <chrono>
#include <memory>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#define ASIO_STANDALONE

#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/message_definition_cache.hpp>
#include <foxglove_bridge/websocket_notls.hpp>
#include <foxglove_bridge/websocket_server.hpp>

constexpr uint16_t DEFAULT_PORT = 8765;
constexpr int DEFAULT_MAX_UPDATE_MS = 5000;
constexpr int DEFAULT_NUM_THREADS = 0;

using namespace std::chrono_literals;
using namespace std::placeholders;
using LogLevel = foxglove::WebSocketLogLevel;

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

    auto maxUpdateDescription = rcl_interfaces::msg::ParameterDescriptor{};
    maxUpdateDescription.name = "max_update_ms";
    maxUpdateDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    maxUpdateDescription.description =
      "The maximum time in milliseconds between refreshing the list of known topics, services, "
      "actions, and parameters";
    maxUpdateDescription.read_only = false;
    maxUpdateDescription.additional_constraints = "Must be a positive integer";
    maxUpdateDescription.integer_range.resize(1);
    maxUpdateDescription.integer_range[0].from_value = 1;
    maxUpdateDescription.integer_range[0].to_value = INT32_MAX;
    maxUpdateDescription.integer_range[0].step = 1;
    this->declare_parameter("max_update_ms", DEFAULT_MAX_UPDATE_MS, maxUpdateDescription);

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

    _server.setSubscribeHandler(std::bind(&FoxgloveBridge::subscribeHandler, this, _1));
    _server.setUnsubscribeHandler(std::bind(&FoxgloveBridge::unsubscribeHandler, this, _1));

    uint16_t port = uint16_t(this->get_parameter("port").as_int());
    _server.start(port);

    // Get the actual port we bound to
    uint16_t listeningPort = _server.localEndpoint()->port();
    if (port != listeningPort) {
      RCLCPP_DEBUG(this->get_logger(), "Reassigning \"port\" parameter from %d to %d", port,
                   listeningPort);
      this->set_parameter(rclcpp::Parameter{"port", listeningPort});
    }

    _maxUpdateMs = size_t(this->get_parameter("max_update_ms").as_int());

    // Listen for parameter updates
    _parametersUpdateHandle = this->add_on_set_parameters_callback(
      std::bind(&FoxgloveBridge::parametersHandler, this, std::placeholders::_1));

    _updateTimer =
      this->create_wall_timer(1ms, std::bind(&FoxgloveBridge::updateAdvertisedTopics, this));
  }

  ~FoxgloveBridge() {
    RCLCPP_INFO(this->get_logger(), "Shutting down %s", this->get_name());
    if (_updateTimer) {
      _updateTimer.reset();
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

  void updateAdvertisedTopics() {
    _updateTimer.reset();
    if (!rclcpp::ok()) {
      return;
    }

    // Get the current list of visible topics and datatypes from the ROS graph
    // rclcpp::spin_some(this->get_node_base_interface());
    auto topicNamesAndTypes = this->get_node_graph_interface()->get_topic_names_and_types();
    std::unordered_set<TopicAndDatatype, PairHash> latestTopics;
    latestTopics.reserve(topicNamesAndTypes.size());
    for (const auto& topicNameAndType : topicNamesAndTypes) {
      for (const auto& datatype : topicNameAndType.second) {
        latestTopics.emplace(topicNameAndType.first, datatype);
      }
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

        // Remove this topic+datatype tuple
        _advertisedTopics.erase(topicAndDatatype);
        _channelToTopicAndDatatype.erase(channel.id);

        // Remove the subscription for this topic, if any
        _subscriptions.erase(channel.id);

        RCLCPP_DEBUG(this->get_logger(), "Removed channel %d for topic \"%s\" (%s)", channel.id,
                     channel.topic.c_str(), channel.schemaName.c_str());
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
        } catch (foxglove::DefinitionNotFoundError& err) {
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

    // Schedule the next update
    _updateCount++;
    auto nextUpdateMs =
      std::chrono::milliseconds(std::min(size_t(1) << _updateCount, _maxUpdateMs));
    _updateTimer = this->create_wall_timer(
      nextUpdateMs, std::bind(&FoxgloveBridge::updateAdvertisedTopics, this));
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
  std::unordered_map<foxglove::ChannelId, std::shared_ptr<rclcpp::GenericSubscription>>
    _subscriptions;
  std::mutex _subscriptionsMutex;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _parametersUpdateHandle;
  rclcpp::TimerBase::SharedPtr _updateTimer;
  size_t _maxUpdateMs = size_t(DEFAULT_MAX_UPDATE_MS);
  size_t _updateCount = 0;

  rcl_interfaces::msg::SetParametersResult parametersHandler(
    const std::vector<rclcpp::Parameter>& parameters) {
    for (const auto& parameter : parameters) {
      if (parameter.get_name() == "max_update_ms") {
        _maxUpdateMs = size_t(parameter.as_int());
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  void subscribeHandler(foxglove::ChannelId channelId) {
    std::lock_guard<std::mutex> lock(_subscriptionsMutex);
    if (_subscriptions.find(channelId) != _subscriptions.end()) {
      // Subscription already exists
      RCLCPP_DEBUG(this->get_logger(), "Subscription already exists for channel %d", channelId);
      return;
    }

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
    auto& channel = it2->second;

    rclcpp::SubscriptionEventCallbacks eventCallbacks;
    eventCallbacks.incompatible_qos_callback = [&](const rclcpp::QOSRequestedIncompatibleQoSInfo&) {
      RCLCPP_ERROR(this->get_logger(), "Incompatible subscriber QoS settings for topic \"%s\" (%s)",
                   topic.c_str(), datatype.c_str());
    };

    rclcpp::SubscriptionOptions subscriptionOptions;
    subscriptionOptions.event_callbacks = eventCallbacks;

    constexpr size_t QUEUE_LENGTH = 10;
    try {
      auto subscriber = this->create_generic_subscription(
        topic, datatype, QUEUE_LENGTH,
        [&](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          rosMessageHandler(channel, msg);
        },
        subscriptionOptions);
      _subscriptions.emplace(channelId, std::move(subscriber));
      RCLCPP_INFO(this->get_logger(), "Subscribed to topic \"%s\" (%s)", topic.c_str(),
                  datatype.c_str());
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to topic \"%s\" (%s): %s",
                   topic.c_str(), datatype.c_str(), ex.what());
    }
  }

  void unsubscribeHandler(foxglove::ChannelId channelId) {
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

    RCLCPP_INFO(this->get_logger(), "Unsubscribing from topic \"%s\" (%s)",
                topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str());
    _subscriptions.erase(it2);
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

  void rosMessageHandler(const foxglove::Channel& channel,
                         std::shared_ptr<rclcpp::SerializedMessage> msg) {
    // NOTE: Do not call any RCLCPP_* logging functions from this function. Otherwise, subscribing
    // to `/rosout` will cause a feedback loop
    auto timestamp = this->now().nanoseconds();
    auto payload =
      std::string_view{reinterpret_cast<const char*>(msg->get_rcl_serialized_message().buffer),
                       msg->get_rcl_serialized_message().buffer_length};
    _server.sendMessage(channel.id, timestamp, payload);
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
