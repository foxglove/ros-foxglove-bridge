#include <chrono>
#include <future>

#define ASIO_STANDALONE
#include <websocketpp/config/asio_client.hpp>

#include <foxglove_bridge/test/test_client.hpp>
#include <foxglove_bridge/websocket_client.hpp>

namespace foxglove {

std::vector<uint8_t> connectClientAndReceiveMsg(const std::string& uri,
                                                const std::string& topic_name) {
  // Set up text message handler to resolve the promise when the topic is advertised
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  std::promise<nlohmann::json> channelPromise;
  auto channelFuture = channelPromise.get_future();
  wsClient.setTextMessageHandler([&topic_name, &channelPromise](const std::string& payload) {
    const auto msg = nlohmann::json::parse(payload);
    const auto& op = msg.at("op").get<std::string>();
    if (op == "advertise") {
      for (const auto& channel : msg.at("channels")) {
        if (topic_name == channel.at("topic")) {
          channelPromise.set_value(channel);
        }
      }
    }
  });

  // Connect the client and wait for the channel future
  if (std::future_status::ready != wsClient.connect(uri).wait_for(std::chrono::seconds(5))) {
    throw std::runtime_error("Client failed to connect");
  } else if (std::future_status::ready != channelFuture.wait_for(std::chrono::seconds(5))) {
    throw std::runtime_error("Client failed to receive channel");
  }

  // Set up binary message handler to resolve when a binary message has been received
  std::promise<std::vector<uint8_t>> msgPromise;
  auto msgFuture = msgPromise.get_future();
  wsClient.setBinaryMessageHandler([&msgPromise](const uint8_t* data, size_t dataLength) {
    const size_t offset = 1 + 4 + 8;
    std::vector<uint8_t> dataCopy(dataLength - offset);
    std::memcpy(dataCopy.data(), data + offset, dataLength - offset);
    msgPromise.set_value(std::move(dataCopy));
  });

  // Subscribe to the channel that corresponds to the topic
  const auto channelId = channelFuture.get().at("id").get<foxglove::ChannelId>();
  wsClient.subscribe({{1, channelId}});

  // Wait until we have received a binary message
  if (std::future_status::ready != msgFuture.wait_for(std::chrono::seconds(5))) {
    throw std::runtime_error("Client failed to receive message");
  }
  return msgFuture.get();
}

// Explicit template instantiation
template class Client<websocketpp::config::asio_client>;

}  // namespace foxglove
