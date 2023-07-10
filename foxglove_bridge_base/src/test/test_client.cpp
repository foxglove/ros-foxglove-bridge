#include <chrono>

#include <websocketpp/config/asio_client.hpp>

#include <foxglove_bridge/serialization.hpp>
#include <foxglove_bridge/test/test_client.hpp>
#include <foxglove_bridge/websocket_client.hpp>

namespace foxglove {

constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(5);

std::vector<uint8_t> connectClientAndReceiveMsg(const std::string& uri,
                                                const std::string& topicName) {
  // Set up text message handler to resolve the promise when the topic is advertised
  auto wsClient = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
  std::promise<nlohmann::json> channelPromise;
  auto channelFuture = waitForChannel(wsClient, topicName);

  // Connect the client and wait for the channel future
  if (std::future_status::ready != wsClient->connect(uri).wait_for(DEFAULT_TIMEOUT)) {
    throw std::runtime_error("Client failed to connect");
  } else if (std::future_status::ready != channelFuture.wait_for(DEFAULT_TIMEOUT)) {
    throw std::runtime_error("Client failed to receive channel");
  }

  const auto channel = channelFuture.get();
  const SubscriptionId subscriptionId = 1;

  // Set up binary message handler to resolve when a binary message has been received
  std::promise<std::vector<uint8_t>> msgPromise;
  auto msgFuture = msgPromise.get_future();
  wsClient->setBinaryMessageHandler([&msgPromise](const uint8_t* data, size_t dataLength) {
    if (ReadUint32LE(data + 1) != subscriptionId) {
      return;
    }
    const size_t offset = 1 + 4 + 8;
    std::vector<uint8_t> dataCopy(dataLength - offset);
    std::memcpy(dataCopy.data(), data + offset, dataLength - offset);
    msgPromise.set_value(std::move(dataCopy));
  });

  // Subscribe to the channel that corresponds to the topic
  wsClient->subscribe({{subscriptionId, channel.id}});

  // Wait until we have received a binary message
  if (std::future_status::ready != msgFuture.wait_for(DEFAULT_TIMEOUT)) {
    throw std::runtime_error("Client failed to receive message");
  }
  return msgFuture.get();
}

std::future<std::vector<Parameter>> waitForParameters(std::shared_ptr<ClientInterface> client,
                                                      const std::string& requestId) {
  auto promise = std::make_shared<std::promise<std::vector<Parameter>>>();
  auto future = promise->get_future();

  client->setTextMessageHandler(
    [promise = std::move(promise), requestId](const std::string& payload) {
      const auto msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();
      const auto id = msg.value("id", "");

      if (op == "parameterValues" && (requestId.empty() || requestId == id)) {
        const auto parameters = msg["parameters"].get<std::vector<Parameter>>();
        promise->set_value(std::move(parameters));
      }
    });

  return future;
}

std::future<ServiceResponse> waitForServiceResponse(std::shared_ptr<ClientInterface> client) {
  auto promise = std::make_shared<std::promise<ServiceResponse>>();
  auto future = promise->get_future();

  client->setBinaryMessageHandler(
    [promise = std::move(promise)](const uint8_t* data, size_t dataLength) mutable {
      if (static_cast<BinaryOpcode>(data[0]) != BinaryOpcode::SERVICE_CALL_RESPONSE) {
        return;
      }

      foxglove::ServiceResponse response;
      response.read(data + 1, dataLength - 1);
      promise->set_value(response);
    });
  return future;
}

std::future<Service> waitForService(std::shared_ptr<ClientInterface> client,
                                    const std::string& serviceName) {
  auto promise = std::make_shared<std::promise<Service>>();
  auto future = promise->get_future();

  client->setTextMessageHandler(
    [promise = std::move(promise), serviceName](const std::string& payload) mutable {
      const auto msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();

      if (op == "advertiseServices") {
        const auto services = msg["services"].get<std::vector<Service>>();
        for (const auto& service : services) {
          if (service.name == serviceName) {
            promise->set_value(service);
            break;
          }
        }
      }
    });

  return future;
}

std::future<Channel> waitForChannel(std::shared_ptr<ClientInterface> client,
                                    const std::string& topicName) {
  auto promise = std::make_shared<std::promise<Channel>>();
  auto future = promise->get_future();

  client->setTextMessageHandler(
    [promise = std::move(promise), topicName](const std::string& payload) mutable {
      const auto msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();

      if (op == "advertise") {
        const auto channels = msg["channels"].get<std::vector<Channel>>();
        for (const auto& channel : channels) {
          if (channel.topic == topicName) {
            promise->set_value(channel);
            break;
          }
        }
      }
    });
  return future;
}

std::future<FetchAssetResponse> waitForFetchAssetResponse(std::shared_ptr<ClientInterface> client) {
  auto promise = std::make_shared<std::promise<FetchAssetResponse>>();
  auto future = promise->get_future();

  client->setBinaryMessageHandler(
    [promise = std::move(promise)](const uint8_t* data, size_t dataLength) mutable {
      if (static_cast<BinaryOpcode>(data[0]) != BinaryOpcode::FETCH_ASSET_RESPONSE) {
        return;
      }

      foxglove::FetchAssetResponse response;
      size_t offset = 1;
      response.requestId = ReadUint32LE(data + offset);
      offset += 4;
      response.status = static_cast<foxglove::FetchAssetStatus>(data[offset]);
      offset += 1;
      const size_t errorMsgLength = static_cast<size_t>(ReadUint32LE(data + offset));
      offset += 4;
      response.errorMessage =
        std::string(reinterpret_cast<const char*>(data + offset), errorMsgLength);
      offset += errorMsgLength;
      const auto payloadLength = dataLength - offset;
      response.data.resize(payloadLength);
      std::memcpy(response.data.data(), data + offset, payloadLength);
      promise->set_value(response);
    });
  return future;
}

// Explicit template instantiation
template class Client<websocketpp::config::asio_client>;

}  // namespace foxglove
