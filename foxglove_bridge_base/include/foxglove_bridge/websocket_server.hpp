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
#include "websocket_tls.hpp"

namespace foxglove {

using json = nlohmann::json;

using ConnHandle = websocketpp::connection_hdl;
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

class ServerInterface {
  using Tcp = websocketpp::lib::asio::ip::tcp;
  using SubscribeUnsubscribeHandler = std::function<void(ChannelId, ConnHandle)>;

public:
  virtual void start(const std::string& host, uint16_t port) = 0;
  virtual void stop() = 0;

  virtual ChannelId addChannel(ChannelWithoutId channel) = 0;
  virtual void removeChannel(ChannelId chanId) = 0;
  virtual void broadcastChannels() = 0;

  virtual void setSubscribeHandler(SubscribeUnsubscribeHandler handler) = 0;
  virtual void setUnsubscribeHandler(SubscribeUnsubscribeHandler handler) = 0;

  virtual void sendMessage(ChannelId chanId, uint64_t timestamp, std::string_view data) = 0;
  virtual void sendMessage(ConnHandle clientHandle, ChannelId chanId, uint64_t timestamp,
                           std::string_view data) = 0;

  virtual std::optional<Tcp::endpoint> localEndpoint() = 0;

private:
  virtual void setupTlsHandler() = 0;
};

template <typename ServerConfiguration>
class Server final : public ServerInterface {
public:
  using ServerType = websocketpp::server<ServerConfiguration>;
  using ConnectionType = websocketpp::connection<ServerConfiguration>;
  using MessagePtr = typename ServerType::message_ptr;
  using Tcp = websocketpp::lib::asio::ip::tcp;
  using SubscribeUnsubscribeHandler = std::function<void(ChannelId, ConnHandle)>;

  static const std::string SUPPORTED_SUBPROTOCOL;
  static bool USES_TLS;

  explicit Server(std::string name, LogCallback logger, const std::string& certfile = "",
                  const std::string& keyfile = "");
  virtual ~Server();

  Server(const Server&) = delete;
  Server(Server&&) = delete;
  Server& operator=(const Server&) = delete;
  Server& operator=(Server&&) = delete;

  void start(const std::string& host, uint16_t port) override;
  void stop() override;

  ChannelId addChannel(ChannelWithoutId channel) override;
  void removeChannel(ChannelId chanId) override;
  void broadcastChannels() override;

  void setSubscribeHandler(SubscribeUnsubscribeHandler handler) override;
  void setUnsubscribeHandler(SubscribeUnsubscribeHandler handler) override;

  void sendMessage(ChannelId chanId, uint64_t timestamp, std::string_view data) override;
  void sendMessage(ConnHandle clientHandle, ChannelId chanId, uint64_t timestamp,
                   std::string_view data) override;

  std::optional<Tcp::endpoint> localEndpoint() override;

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
  std::string _certfile;
  std::string _keyfile;
  ServerType _server;
  std::unique_ptr<std::thread> _serverThread;

  uint32_t _nextChannelId = 0;
  std::map<ConnHandle, ClientInfo, std::owner_less<>> _clients;
  std::unordered_map<ChannelId, Channel> _channels;
  SubscribeUnsubscribeHandler _subscribeHandler;
  SubscribeUnsubscribeHandler _unsubscribeHandler;
  std::shared_mutex _clientsChannelMutex;

  void setupTlsHandler() override;
  void socketInit(ConnHandle hdl);
  bool validateConnection(ConnHandle hdl);
  void handleConnectionOpened(ConnHandle hdl);
  void handleConnectionClosed(ConnHandle hdl);
  void handleMessage(ConnHandle hdl, MessagePtr msg);

  void sendJson(ConnHandle hdl, json&& payload);
  void sendJsonRaw(ConnHandle hdl, const std::string& payload);
  void sendBinary(ConnHandle hdl, const std::vector<uint8_t>& payload);
};

template <typename ServerConfiguration>
inline const std::string Server<ServerConfiguration>::SUPPORTED_SUBPROTOCOL =
  "foxglove.websocket.v1";

template <typename ServerConfiguration>
inline Server<ServerConfiguration>::Server(std::string name, LogCallback logger,
                                           const std::string& certfile, const std::string& keyfile)
    : _name(std::move(name))
    , _logger(logger)
    , _certfile(certfile)
    , _keyfile(keyfile) {
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
  _server.set_tcp_pre_init_handler(std::bind(&Server::socketInit, this, std::placeholders::_1));
  this->setupTlsHandler();
  _server.set_validate_handler(std::bind(&Server::validateConnection, this, std::placeholders::_1));
  _server.set_open_handler(std::bind(&Server::handleConnectionOpened, this, std::placeholders::_1));
  _server.set_close_handler(
    std::bind(&Server::handleConnectionClosed, this, std::placeholders::_1));
  _server.set_message_handler(
    std::bind(&Server::handleMessage, this, std::placeholders::_1, std::placeholders::_2));
  _server.set_reuse_addr(true);
  _server.set_listen_backlog(128);
}

template <typename ServerConfiguration>
inline Server<ServerConfiguration>::~Server() {}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::socketInit(ConnHandle hdl) {
  std::error_code ec;
  _server.get_con_from_hdl(hdl)->get_raw_socket().set_option(Tcp::no_delay(true), ec);
  if (ec) {
    _server.get_elog().write(RECOVERABLE, "Failed to set TCP_NODELAY: " + ec.message());
  }
}

template <typename ServerConfiguration>
inline bool Server<ServerConfiguration>::validateConnection(ConnHandle hdl) {
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

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleConnectionOpened(ConnHandle hdl) {
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

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleConnectionClosed(ConnHandle hdl) {
  std::unordered_map<ChannelId, SubscriptionId> oldSubscriptionsByChannel;
  {
    std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
    const auto& client = _clients.find(hdl);
    if (client == _clients.end()) {
      _server.get_elog().write(RECOVERABLE, "Client " +
                                              _server.get_con_from_hdl(hdl)->get_remote_endpoint() +
                                              " disconnected but not found in _clients");
      return;
    }

    _server.get_alog().write(APP, "Client " + client->second.name + " disconnected");

    oldSubscriptionsByChannel = std::move(client->second.subscriptionsByChannel);
    _clients.erase(client);
  }

  if (_unsubscribeHandler) {
    for (const auto& [chanId, subs] : oldSubscriptionsByChannel) {
      _unsubscribeHandler(chanId, hdl);
    }
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::setSubscribeHandler(SubscribeUnsubscribeHandler handler) {
  _subscribeHandler = std::move(handler);
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::setUnsubscribeHandler(
  SubscribeUnsubscribeHandler handler) {
  _unsubscribeHandler = std::move(handler);
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::stop() {
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

  std::vector<std::shared_ptr<ConnectionType>> connections;
  {
    std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
    connections.reserve(_clients.size());
    for (const auto& [hdl, client] : _clients) {
      if (auto connection = _server.get_con_from_hdl(hdl, ec)) {
        connections.push_back(connection);
      }
    }
  }

  if (!connections.empty()) {
    _server.get_alog().write(
      APP, "Closing " + std::to_string(connections.size()) + " client connection(s)");

    // Iterate over all client connections and start the close connection handshake
    for (const auto& connection : connections) {
      connection->close(websocketpp::close::status::going_away, "server shutdown", ec);
      if (ec) {
        _server.get_elog().write(RECOVERABLE, "Failed to close connection: " + ec.message());
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

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::start(const std::string& host, uint16_t port) {
  if (_serverThread) {
    throw std::runtime_error("Server already started");
  }

  std::error_code ec;

  _server.listen(host, std::to_string(port), ec);
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

  const std::string protocol = USES_TLS ? "wss" : "ws";
  auto address = endpoint.address();
  _server.get_alog().write(APP, "WebSocket server listening at " + protocol + "://" +
                                  IPAddressToString(address) + ":" +
                                  std::to_string(endpoint.port()));
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendJson(ConnHandle hdl, json&& payload) {
  try {
    _server.send(hdl, std::move(payload).dump(), OpCode::TEXT);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendJsonRaw(ConnHandle hdl, const std::string& payload) {
  try {
    _server.send(hdl, payload, OpCode::TEXT);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendBinary(ConnHandle hdl,
                                                    const std::vector<uint8_t>& payload) {
  try {
    _server.send(hdl, payload.data(), payload.size(), OpCode::BINARY);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleMessage(ConnHandle hdl, MessagePtr msg) {
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
        clientInfo.subscriptionsByChannel.emplace(channelId, subId);
        if (_subscribeHandler) {
          _subscribeHandler(channelId, hdl);
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
        ChannelId chanId = sub->first;
        clientInfo.subscriptionsByChannel.erase(sub);
        if (_unsubscribeHandler) {
          _unsubscribeHandler(chanId, hdl);
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

template <typename ServerConfiguration>
inline ChannelId Server<ServerConfiguration>::addChannel(ChannelWithoutId channel) {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  const auto newId = ++_nextChannelId;
  Channel newChannel{newId, std::move(channel)};
  _channels.emplace(newId, std::move(newChannel));
  return newId;
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::removeChannel(ChannelId chanId) {
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

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::broadcastChannels() {
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

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendMessage(ChannelId chanId, uint64_t timestamp,
                                                     std::string_view data) {
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

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendMessage(ConnHandle clientHandle, ChannelId chanId,
                                                     uint64_t timestamp, std::string_view data) {
  SubscriptionId subId = std::numeric_limits<SubscriptionId>::max();

  {
    std::shared_lock<std::shared_mutex> lock(_clientsChannelMutex);
    const auto clientHandleAndInfoIt = _clients.find(clientHandle);
    if (clientHandleAndInfoIt == _clients.end()) {
      return;  // Client got removed in the meantime.
    }

    const auto& client = clientHandleAndInfoIt->second;
    const auto& subs = client.subscriptionsByChannel.find(chanId);
    if (subs == client.subscriptionsByChannel.end()) {
      return;  // Client not subscribed to this channel.
    }
    subId = subs->second;
  }

  std::vector<uint8_t> message(1 + 4 + 8 + data.size());
  message[0] = uint8_t(BinaryOpcode::MESSAGE_DATA);
  foxglove::WriteUint32LE(message.data() + 1, subId);
  foxglove::WriteUint64LE(message.data() + 5, timestamp);
  std::memcpy(message.data() + 1 + 4 + 8, data.data(), data.size());
  sendBinary(clientHandle, message);
}

template <typename ServerConfiguration>
inline std::optional<asio::ip::tcp::endpoint> Server<ServerConfiguration>::localEndpoint() {
  std::error_code ec;
  auto endpoint = _server.get_local_endpoint(ec);
  if (ec) {
    return std::nullopt;
  }
  return endpoint;
}

template <>
bool Server<WebSocketNoTls>::USES_TLS = false;

template <>
bool Server<WebSocketTls>::USES_TLS = true;

template <>
inline void Server<WebSocketNoTls>::setupTlsHandler() {
  _server.get_alog().write(APP, "Server running without TLS");
}

template <>
inline void Server<WebSocketTls>::setupTlsHandler() {
  _server.set_tls_init_handler([this](ConnHandle hdl) {
    (void)hdl;

    namespace asio = websocketpp::lib::asio;
    auto ctx = websocketpp::lib::make_shared<asio::ssl::context>(asio::ssl::context::sslv23);

    try {
      ctx->set_options(asio::ssl::context::default_workarounds | asio::ssl::context::no_tlsv1 |
                       asio::ssl::context::no_sslv2 | asio::ssl::context::no_sslv3);
      ctx->use_certificate_chain_file(_certfile);
      ctx->use_private_key_file(_keyfile, asio::ssl::context::pem);

      constexpr char ciphers[] =
        "ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384:"
        "ECDHE-ECDSA-AES256-GCM-SHA384:DHE-RSA-AES128-GCM-SHA256:DHE-DSS-AES128-GCM-SHA256:kEDH+"
        "AESGCM:ECDHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA:ECDHE-ECDSA-"
        "AES128-SHA:ECDHE-RSA-AES256-SHA384:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA:ECDHE-"
        "ECDSA-AES256-SHA:DHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA:DHE-DSS-AES128-SHA256:DHE-RSA-"
        "AES256-SHA256:DHE-DSS-AES256-SHA:DHE-RSA-AES256-SHA:AES128-GCM-SHA256:AES256-GCM-SHA384:"
        "AES128-SHA256:AES256-SHA256:AES128-SHA:AES256-SHA:AES:CAMELLIA:DES-CBC3-SHA:!aNULL:!eNULL:"
        "!EXPORT:!DES:!RC4:!MD5:!PSK:!aECDH:!EDH-DSS-DES-CBC3-SHA:!EDH-RSA-DES-CBC3-SHA:!KRB5-DES-"
        "CBC3-SHA";

      if (SSL_CTX_set_cipher_list(ctx->native_handle(), ciphers) != 1) {
        _server.get_elog().write(RECOVERABLE, "Error setting cipher list");
      }
    } catch (const std::exception& ex) {
      _server.get_elog().write(RECOVERABLE,
                               std::string("Exception in TLS handshake: ") + ex.what());
    }
    return ctx;
  });
}

}  // namespace foxglove
