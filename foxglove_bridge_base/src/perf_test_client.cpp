#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <regex>
#include <unordered_map>

#include <foxglove_bridge/websocket_client.hpp>
#include <foxglove_bridge/websocket_notls.hpp>

struct ChannelStats {
  std::string topic;
  size_t bytesReceived = 0;
  size_t msgsReceived = 0;
  std::chrono::time_point<std::chrono::steady_clock> firstMsgTime;
  std::chrono::time_point<std::chrono::steady_clock> lastMsgTime;
};

constexpr char DEFAULT_URI[] = "ws://localhost:8765";
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(5);

std::atomic<bool> doAbort = false;

void signal_handler(int) {
  doAbort = true;
}

int main(int argc, char** argv) {
  const auto url = argc > 1 ? argv[1] : DEFAULT_URI;
  const auto topicRegex = argc > 2 ? std::regex(argv[2]) : std::regex(".*");
  std::unordered_map<foxglove::SubscriptionId, ChannelStats> channelStatsBySubId;
  std::unordered_map<foxglove::ChannelId, foxglove::SubscriptionId> subIdByChannelId;
  foxglove::SubscriptionId nextSubId = 0;
  foxglove::Client<foxglove::WebSocketNoTls> client;
  size_t maxTopicCharLength = 0;

  client.setBinaryMessageHandler([&](const uint8_t* data, size_t dataLength) {
    if (static_cast<foxglove::BinaryOpcode>(data[0]) != foxglove::BinaryOpcode::MESSAGE_DATA) {
      return;
    }

    const auto subId = foxglove::ReadUint32LE(data + 1);
    ChannelStats& channelStats = channelStatsBySubId[subId];
    channelStats.msgsReceived++;
    channelStats.bytesReceived += dataLength;
    channelStats.lastMsgTime = std::chrono::steady_clock::now();
    if (channelStats.msgsReceived == 1) {
      channelStats.firstMsgTime = channelStats.lastMsgTime;
    }
  });

  client.setTextMessageHandler([&](const std::string& payload) {
    const auto msg = nlohmann::json::parse(payload);
    const auto& op = msg["op"].get<std::string>();
    if (op != "advertise") {
      return;
    }

    const auto channels = msg["channels"].get<std::vector<foxglove::Channel>>();
    std::vector<std::pair<foxglove::SubscriptionId, foxglove::ChannelId>> subscribePayload;

    for (const auto& channel : channels) {
      if (subIdByChannelId.find(channel.id) == subIdByChannelId.end() &&
          std::regex_match(channel.topic, topicRegex)) {
        const auto subId = nextSubId++;
        subscribePayload.push_back({subId, channel.id});
        subIdByChannelId.insert({channel.id, subId});
        ChannelStats channelStats;
        channelStats.topic = channel.topic;
        channelStatsBySubId.insert({subId, channelStats});
        maxTopicCharLength = std::max(maxTopicCharLength, channel.topic.size());
      }
    }

    if (!subscribePayload.empty()) {
      client.subscribe(subscribePayload);
    }
  });

  const auto openHandler = [&](websocketpp::connection_hdl) {
    std::cout << "Connected to " << std::string(url) << std::endl;
  };
  const auto closeHandler = [&](websocketpp::connection_hdl) {
    std::cout << "Connection closed" << std::endl;
    doAbort = true;
  };

  client.connect(url, openHandler, closeHandler);
  std::signal(SIGINT, signal_handler);

  while (!doAbort) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  client.close();

  size_t totalBytesRcvd = 0;
  size_t totalMsgsRcvd = 0;
  double totalBandwidth = 0;
  std::cout << std::endl;
  std::cout << "| " << std::setw(maxTopicCharLength) << "topic"
            << " | Msgs rcvd |   bytes rcvd | bandwidth (b/s) |" << std::endl;
  std::cout << "|-" << std::string(maxTopicCharLength, '-')
            << "-|-----------|--------------|-----------------|" << std::endl;

  for (const auto& [subId, stats] : channelStatsBySubId) {
    (void)subId;
    const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stats.lastMsgTime - stats.firstMsgTime)
        .count();
    const double bandwidth = static_cast<double>(stats.bytesReceived) / (duration / 1000.0);

    std::cout << "| " << std::setw(maxTopicCharLength) << stats.topic << " | " << std::setw(9)
              << stats.msgsReceived << " | " << std::setw(12) << stats.bytesReceived << " | "
              << std::setw(15) << std::setprecision(2) << std::scientific << bandwidth << " |"
              << std::endl;
    totalBytesRcvd += stats.bytesReceived;
    totalMsgsRcvd += stats.msgsReceived;
    if (std::isnormal(bandwidth)) {
      totalBandwidth += bandwidth;
    }
  }

  std::cout << "|-" << std::string(maxTopicCharLength, '-')
            << "-|-----------|--------------|-----------------|" << std::endl;
  std::cout << "| " << std::setw(maxTopicCharLength) << "TOTAL"
            << " | " << std::setw(9) << totalMsgsRcvd << " | " << std::setw(12) << totalBytesRcvd
            << " | " << std::setw(15) << std::setprecision(2) << std::scientific << totalBandwidth
            << " |" << std::endl;

  return EXIT_SUCCESS;
}
