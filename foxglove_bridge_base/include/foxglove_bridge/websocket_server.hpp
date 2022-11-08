#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "serialization.hpp"
#include "websocket_logging.hpp"
#include "websocket_notls.hpp"

namespace foxglove {

using json = nlohmann::json;
using namespace std::placeholders;

using ConnHandle = websocketpp::connection_hdl;
using MessagePtr = WebSocketNoTlsServer::message_ptr;
using OpCode = websocketpp::frame::opcode::value;

using ChannelId = uint32_t;
using SubscriptionId = uint32_t;

static const websocketpp::log::level APP = websocketpp::log::alevel::app;
static const websocketpp::log::level RECOVERABLE = websocketpp::log::elevel::rerror;

struct ChannelWithoutId {
  std::string topic;
  std::string encoding;
  std::string schemaName;
  std::string schema;

  bool operator==(const ChannelWithoutId& other) const {
    return topic == other.topic && encoding == other.encoding && schemaName == other.schemaName &&
           schema == other.schema;
  }
};

struct Channel : ChannelWithoutId {
  ChannelId id;

  explicit Channel(ChannelId id, ChannelWithoutId ch)
      : ChannelWithoutId(std::move(ch))
      , id(id) {}

  friend void to_json(json& j, const Channel& channel) {
    j = {
      {"id", channel.id},
      {"topic", channel.topic},
      {"encoding", channel.encoding},
      {"schemaName", channel.schemaName},
      {"schema", channel.schema},
    };
  }

  bool operator==(const Channel& other) const {
    return id == other.id && ChannelWithoutId::operator==(other);
  }
};

enum class BinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
};

enum class StatusLevel : uint8_t {
  INFO = 0,
  WARNING = 1,
  ERROR = 2,
};

class Server final {
public:
  static const std::string SUPPORTED_SUBPROTOCOL;

  explicit Server(std::string name, LogCallback logger);
  ~Server();

  Server(const Server&) = delete;
  Server(Server&&) = delete;
  Server& operator=(const Server&) = delete;
  Server& operator=(Server&&) = delete;

  void start(uint16_t port);
  void stop();

  ChannelId addChannel(ChannelWithoutId channel);
  void removeChannel(ChannelId chanId);
  void broadcastChannels();

  void setSubscribeHandler(std::function<void(ChannelId)> handler);
  void setUnsubscribeHandler(std::function<void(ChannelId)> handler);

  void sendMessage(ChannelId chanId, uint64_t timestamp, std::string_view data);

  std::optional<asio::ip::tcp::endpoint> localEndpoint();

private:
  struct ClientInfo {
    std::string name;
    ConnHandle handle;
    std::unordered_map<ChannelId, SubscriptionId> subscriptionsByChannel;

    ClientInfo(const ClientInfo&) = delete;
    ClientInfo& operator=(const ClientInfo&) = delete;

    ClientInfo(ClientInfo&&) = default;
    ClientInfo& operator=(ClientInfo&&) = default;
  };

  std::string _name;
  LogCallback _logger;
  WebSocketNoTlsServer _server;
  std::unique_ptr<std::thread> _serverThread;

  uint32_t _nextChannelId = 0;
  std::map<ConnHandle, ClientInfo, std::owner_less<>> _clients;
  std::unordered_map<ChannelId, Channel> _channels;
  std::function<void(ChannelId)> _subscribeHandler;
  std::function<void(ChannelId)> _unsubscribeHandler;
  std::shared_mutex _clientsChannelMutex;

  bool validateConnection(ConnHandle hdl);
  void handleConnectionOpened(ConnHandle hdl);
  void handleConnectionClosed(ConnHandle hdl);
  void handleMessage(ConnHandle hdl, MessagePtr msg);

  void sendJson(ConnHandle hdl, json&& payload);
  void sendJsonRaw(ConnHandle hdl, const std::string& payload);
  void sendBinary(ConnHandle hdl, const std::vector<uint8_t>& payload);

  bool anySubscribed(ChannelId chanId) const;
};

inline const std::string Server::SUPPORTED_SUBPROTOCOL = "foxglove.websocket.v1";

inline Server::Server(std::string name, LogCallback logger)
    : _name(std::move(name))
    , _logger(logger) {
  // Redirect logging
  _server.get_alog().set_callback(_logger);
  _server.get_elog().set_callback(_logger);

  std::error_code ec;
  _server.init_asio(ec);
  if (ec) {
    throw std::runtime_error("Failed to initialize websocket server: " + ec.message());
  }

  _server.clear_access_channels(websocketpp::log::alevel::all);
  _server.set_access_channels(APP);
  _server.set_validate_handler(std::bind(&Server::validateConnection, this, _1));
  _server.set_open_handler(std::bind(&Server::handleConnectionOpened, this, _1));
  _server.set_close_handler(std::bind(&Server::handleConnectionClosed, this, _1));
  _server.set_message_handler(std::bind(&Server::handleMessage, this, _1, _2));
  _server.set_reuse_addr(true);
  _server.set_listen_backlog(128);
}

inline Server::~Server() {}

inline bool Server::validateConnection(ConnHandle hdl) {
  auto con = _server.get_con_from_hdl(hdl);

  const auto& subprotocols = con->get_requested_subprotocols();
  if (std::find(subprotocols.begin(), subprotocols.end(), SUPPORTED_SUBPROTOCOL) !=
      subprotocols.end()) {
    con->select_subprotocol(SUPPORTED_SUBPROTOCOL);
    return true;
  }
  _server.get_alog().write(APP, "Rejecting client " + con->get_remote_endpoint() +
                                  " which did not declare support for subprotocol " +
                                  SUPPORTED_SUBPROTOCOL);
  return false;
}

inline void Server::handleConnectionOpened(ConnHandle hdl) {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  auto con = _server.get_con_from_hdl(hdl);
  _server.get_alog().write(
    APP, "Client " + con->get_remote_endpoint() + " connected via " + con->get_resource());
  _clients.emplace(hdl, ClientInfo{con->get_remote_endpoint(), hdl, {}});

  con->send(json({
                   {"op", "serverInfo"},
                   {"name", _name},
                   {"capabilities", json::array()},
                 })
              .dump());

  json channels;
  for (const auto& [id, channel] : _channels) {
    channels.push_back(channel);
  }
  sendJson(hdl, {
                  {"op", "advertise"},
                  {"channels", std::move(channels)},
                });
}

inline void Server::handleConnectionClosed(ConnHandle hdl) {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  const auto& client = _clients.find(hdl);
  if (client == _clients.end()) {
    _server.get_elog().write(RECOVERABLE, "Client " +
                                            _server.get_con_from_hdl(hdl)->get_remote_endpoint() +
                                            " disconnected but not found in _clients");
    return;
  }

  _server.get_alog().write(APP, "Client " + client->second.name + " disconnected");

  const auto oldSubscriptionsByChannel = std::move(client->second.subscriptionsByChannel);
  _clients.erase(client);
  for (const auto& [chanId, subs] : oldSubscriptionsByChannel) {
    if (!anySubscribed(chanId) && _unsubscribeHandler) {
      _unsubscribeHandler(chanId);
    }
  }
}

inline void Server::setSubscribeHandler(std::function<void(ChannelId)> handler) {
  _subscribeHandler = std::move(handler);
}
inline void Server::setUnsubscribeHandler(std::function<void(ChannelId)> handler) {
  _unsubscribeHandler = std::move(handler);
}

inline void Server::stop() {
  if (_server.stopped()) {
    return;
  }

  _server.get_alog().write(APP, "Stopping WebSocket server");
  std::error_code ec;

  _server.stop_perpetual();

  if (_server.is_listening()) {
    _server.stop_listening(ec);
    if (ec) {
      _server.get_elog().write(RECOVERABLE, "Failed to stop listening: " + ec.message());
    }
  }

  std::vector<websocketpp::connection_hdl> connections;
  {
    std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
    connections.reserve(_clients.size());
    for (const auto& [hdl, client] : _clients) {
      connections.push_back(hdl);
    }
  }

  if (!connections.empty()) {
    _server.get_alog().write(
      APP, "Closing " + std::to_string(connections.size()) + " client connection(s)");

    // Iterate over all client connections and start the close connection handshake
    for (const auto& hdl : connections) {
      if (auto con = _server.get_con_from_hdl(hdl, ec)) {
        con->close(websocketpp::close::status::going_away, "server shutdown", ec);
        if (ec) {
          _server.get_elog().write(RECOVERABLE, "Failed to close connection: " + ec.message());
        }
      }
    }

    // Wait for all connections to close
    constexpr size_t MAX_SHUTDOWN_MS = 1000;
    constexpr size_t SLEEP_MS = 10;
    size_t durationMs = 0;
    while (!_server.stopped() && durationMs < MAX_SHUTDOWN_MS) {
      std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
      _server.poll_one();
      durationMs += SLEEP_MS;
    }

    if (!_server.stopped()) {
      _server.get_elog().write(RECOVERABLE, "Failed to close all connections, forcefully stopping");
      for (const auto& hdl : connections) {
        if (auto con = _server.get_con_from_hdl(hdl, ec)) {
          _server.get_elog().write(RECOVERABLE,
                                   "Terminating connection to " + con->get_remote_endpoint());
          con->terminate(ec);
        }
      }
      _server.stop();
    }
  }

  _server.get_alog().write(APP, "All WebSocket connections closed");

  if (_serverThread) {
    _server.get_alog().write(APP, "Waiting for WebSocket server run loop to terminate");
    _serverThread->join();
    _serverThread.reset();
    _server.get_alog().write(APP, "WebSocket server run loop terminated");
  }

  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  _clients.clear();
}

inline void Server::start(uint16_t port) {
  if (_serverThread) {
    throw std::runtime_error("Server already started");
  }

  std::error_code ec;

  _server.listen(port, ec);
  if (ec) {
    throw std::runtime_error("Failed to listen on port " + std::to_string(port) + ": " +
                             ec.message());
  }

  _server.start_accept(ec);
  if (ec) {
    throw std::runtime_error("Failed to start accepting connections: " + ec.message());
  }

  _serverThread = std::make_unique<std::thread>([this]() {
    _server.get_alog().write(APP, "WebSocket server run loop started");
    _server.run();
    _server.get_alog().write(APP, "WebSocket server run loop stopped");
  });

  if (!_server.is_listening()) {
    throw std::runtime_error("WebSocket server failed to listen on port " + std::to_string(port));
  }

  auto endpoint = _server.get_local_endpoint(ec);
  if (ec) {
    throw std::runtime_error("Failed to resolve the local endpoint: " + ec.message());
  }

  auto address = endpoint.address();
  _server.get_alog().write(APP, "WebSocket server listening at ws://" + IPAddressToString(address) +
                                  ":" + std::to_string(endpoint.port()));
}

inline void Server::sendJson(ConnHandle hdl, json&& payload) {
  try {
    _server.send(hdl, std::move(payload).dump(), OpCode::TEXT);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

inline void Server::sendJsonRaw(ConnHandle hdl, const std::string& payload) {
  try {
    _server.send(hdl, payload, OpCode::TEXT);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

inline void Server::sendBinary(ConnHandle hdl, const std::vector<uint8_t>& payload) {
  try {
    _server.send(hdl, payload.data(), payload.size(), OpCode::BINARY);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

inline void Server::handleMessage(ConnHandle hdl, MessagePtr msg) {
  std::error_code ec;
  auto con = _server.get_con_from_hdl(hdl, ec);
  if (!con) {
    _server.get_elog().write(RECOVERABLE, "get_con_from_hdl failed in handleMessage");
    return;
  }

  const std::string remoteEndpoint = con->get_remote_endpoint();

  try {
    std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
    auto& clientInfo = _clients.at(hdl);

    const auto findSubscriptionBySubId = [&clientInfo](SubscriptionId subId) {
      return std::find_if(clientInfo.subscriptionsByChannel.begin(),
                          clientInfo.subscriptionsByChannel.end(), [&subId](const auto& mo) {
                            return mo.second == subId;
                          });
    };

    const auto& payloadStr = msg->get_payload();
    const json payload = json::parse(payloadStr);
    const std::string& op = payload.at("op").get<std::string>();

    if (op == "subscribe") {
      for (const auto& sub : payload.at("subscriptions")) {
        SubscriptionId subId = sub.at("id");
        ChannelId channelId = sub.at("channelId");
        if (findSubscriptionBySubId(subId) != clientInfo.subscriptionsByChannel.end()) {
          sendJson(hdl, json{
                          {"op", "status"},
                          {"level", static_cast<uint8_t>(StatusLevel::ERROR)},
                          {"message", "Client subscription id " + std::to_string(subId) +
                                        " was already used; ignoring subscription"},
                        });
          continue;
        }
        const auto& channelIt = _channels.find(channelId);
        if (channelIt == _channels.end()) {
          sendJson(hdl, json{
                          {"op", "status"},
                          {"level", static_cast<uint8_t>(StatusLevel::WARNING)},
                          {"message", "Channel " + std::to_string(channelId) +
                                        " is not available; ignoring subscription"},
                        });
          continue;
        }
        bool firstSubscription = !anySubscribed(channelId);
        clientInfo.subscriptionsByChannel.emplace(channelId, subId);
        if (firstSubscription && _subscribeHandler) {
          _subscribeHandler(channelId);
        }
      }
    } else if (op == "unsubscribe") {
      for (const auto& subIdJson : payload.at("subscriptionIds")) {
        SubscriptionId subId = subIdJson;
        const auto& sub = findSubscriptionBySubId(subId);
        if (sub == clientInfo.subscriptionsByChannel.end()) {
          sendJson(hdl, json{
                          {"op", "status"},
                          {"level", static_cast<uint8_t>(StatusLevel::WARNING)},
                          {"message", "Client subscription id " + std::to_string(subId) +
                                        " did not exist; ignoring unsubscription"},
                        });
          continue;
        }
        ChannelId chanId = sub->second;
        clientInfo.subscriptionsByChannel.erase(sub);
        if (!anySubscribed(chanId) && _unsubscribeHandler) {
          _unsubscribeHandler(chanId);
        }
      }

    } else {
      _server.get_elog().write(RECOVERABLE, "Unrecognized client opcode: " + op);
      sendJson(hdl, {
                      {"op", "status"},
                      {"level", static_cast<uint8_t>(StatusLevel::ERROR)},
                      {"message", "Unrecognized opcode " + op},
                    });
    }
  } catch (std::exception const& ex) {
    _server.get_elog().write(RECOVERABLE,
                             "Error parsing message from " + remoteEndpoint + ": " + ex.what());
    return;
  }
}

inline ChannelId Server::addChannel(ChannelWithoutId channel) {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  const auto newId = ++_nextChannelId;
  Channel newChannel{newId, std::move(channel)};
  _channels.emplace(newId, std::move(newChannel));
  return newId;
}

inline void Server::removeChannel(ChannelId chanId) {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  _channels.erase(chanId);
  for (auto& [hdl, clientInfo] : _clients) {
    if (const auto it = clientInfo.subscriptionsByChannel.find(chanId);
        it != clientInfo.subscriptionsByChannel.end()) {
      clientInfo.subscriptionsByChannel.erase(it);
    }
    sendJson(hdl, {{"op", "unadvertise"}, {"channelIds", {chanId}}});
  }
}

inline void Server::broadcastChannels() {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);

  if (_clients.empty()) {
    return;
  }

  json channels;
  for (const auto& [id, channel] : _channels) {
    channels.push_back(channel);
  }
  std::string msg = json{{"op", "advertise"}, {"channels", std::move(channels)}}.dump();

  for (const auto& [hdl, clientInfo] : _clients) {
    sendJsonRaw(hdl, msg);
  }
}

inline void Server::sendMessage(ChannelId chanId, uint64_t timestamp, std::string_view data) {
  std::shared_lock<std::shared_mutex> lock(_clientsChannelMutex);
  std::vector<uint8_t> message;
  for (const auto& [hdl, client] : _clients) {
    const auto& subs = client.subscriptionsByChannel.find(chanId);
    if (subs == client.subscriptionsByChannel.end()) {
      continue;
    }
    const auto subId = subs->second;
    if (message.empty()) {
      message.resize(1 + 4 + 8 + data.size());
      message[0] = uint8_t(BinaryOpcode::MESSAGE_DATA);
      // message[1..4] is the subscription id, which we'll fill in per-recipient below
      foxglove::WriteUint64LE(message.data() + 5, timestamp);
      std::memcpy(message.data() + 1 + 4 + 8, data.data(), data.size());
    }
    foxglove::WriteUint32LE(message.data() + 1, subId);
    sendBinary(hdl, message);
  }
}

inline std::optional<asio::ip::tcp::endpoint> Server::localEndpoint() {
  std::error_code ec;
  auto endpoint = _server.get_local_endpoint(ec);
  if (ec) {
    return std::nullopt;
  }
  return endpoint;
}

inline bool Server::anySubscribed(ChannelId chanId) const {
  for (const auto& [hdl, client] : _clients) {
    if (client.subscriptionsByChannel.find(chanId) != client.subscriptionsByChannel.end()) {
      return true;
    }
  }
  return false;
}

}  // namespace foxglove
