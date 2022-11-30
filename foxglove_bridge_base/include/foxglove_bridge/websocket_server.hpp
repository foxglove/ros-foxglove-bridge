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
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "common.hpp"
#include "serialization.hpp"
#include "websocket_logging.hpp"
#include "websocket_notls.hpp"
#include "websocket_tls.hpp"

namespace foxglove {

using json = nlohmann::json;

using ConnHandle = websocketpp::connection_hdl;
using OpCode = websocketpp::frame::opcode::value;

static const websocketpp::log::level APP = websocketpp::log::alevel::app;
static const websocketpp::log::level RECOVERABLE = websocketpp::log::elevel::rerror;

constexpr uint32_t Integer(const std::string_view str) {
  uint32_t result = 0x811C9DC5;  // FNV-1a 32-bit algorithm
  for (char c : str) {
    result = (c ^ result) * 0x01000193;
  }
  return result;
}

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

struct ClientMessage {
  uint64_t logTime;
  uint64_t publishTime;
  uint32_t sequence;
  const ClientAdvertisement& advertisement;
  size_t dataLength;
  const uint8_t* data;

  ClientMessage(uint64_t logTime, uint64_t publishTime, uint32_t sequence,
                const ClientAdvertisement& advertisement, size_t dataLength, const uint8_t* data)
      : logTime(logTime)
      , publishTime(publishTime)
      , sequence(sequence)
      , advertisement(advertisement)
      , dataLength(dataLength)
      , data(data) {}

  static const size_t MSG_PAYLOAD_OFFSET = 5;

  const uint8_t* getData() const {
    return data + MSG_PAYLOAD_OFFSET;
  }
  std::size_t getLength() const {
    return dataLength - MSG_PAYLOAD_OFFSET;
  }
};

enum class StatusLevel : uint8_t {
  Info = 0,
  Warning = 1,
  Error = 2,
};

constexpr const char* StatusLevelToString(StatusLevel level) {
  switch (level) {
    case StatusLevel::Info:
      return "INFO";
    case StatusLevel::Warning:
      return "WARN";
    case StatusLevel::Error:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

class ServerInterface {
  using Tcp = websocketpp::lib::asio::ip::tcp;
  using SubscribeUnsubscribeHandler = std::function<void(ChannelId, ConnHandle)>;
  using ClientAdvertiseHandler = std::function<void(const ClientAdvertisement&, ConnHandle)>;
  using ClientUnadvertiseHandler = std::function<void(ClientChannelId, ConnHandle)>;
  using ClientMessageHandler = std::function<void(const ClientMessage&, ConnHandle)>;

public:
  virtual void start(const std::string& host, uint16_t port) = 0;
  virtual void stop() = 0;

  virtual ChannelId addChannel(ChannelWithoutId channel) = 0;
  virtual void removeChannel(ChannelId chanId) = 0;
  virtual void broadcastChannels() = 0;

  virtual void setSubscribeHandler(SubscribeUnsubscribeHandler handler) = 0;
  virtual void setUnsubscribeHandler(SubscribeUnsubscribeHandler handler) = 0;
  virtual void setClientAdvertiseHandler(ClientAdvertiseHandler handler) = 0;
  virtual void setClientUnadvertiseHandler(ClientUnadvertiseHandler handler) = 0;
  virtual void setClientMessageHandler(ClientMessageHandler handler) = 0;

  virtual void sendMessage(ConnHandle clientHandle, ChannelId chanId, uint64_t timestamp,
                           std::string_view data) = 0;

  virtual std::optional<Tcp::endpoint> localEndpoint() = 0;
  virtual std::string remoteEndpointString(ConnHandle clientHandle) = 0;

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
  using ClientAdvertiseHandler = std::function<void(const ClientAdvertisement&, ConnHandle)>;
  using ClientUnadvertiseHandler = std::function<void(ClientChannelId, ConnHandle)>;
  using ClientMessageHandler = std::function<void(const ClientMessage&, ConnHandle)>;

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
  void setClientAdvertiseHandler(ClientAdvertiseHandler handler) override;
  void setClientUnadvertiseHandler(ClientUnadvertiseHandler handler) override;
  void setClientMessageHandler(ClientMessageHandler handler) override;

  void sendMessage(ConnHandle clientHandle, ChannelId chanId, uint64_t timestamp,
                   std::string_view data) override;

  std::optional<Tcp::endpoint> localEndpoint() override;
  std::string remoteEndpointString(ConnHandle clientHandle) override;

private:
  struct ClientInfo {
    std::string name;
    ConnHandle handle;
    std::unordered_map<ChannelId, SubscriptionId> subscriptionsByChannel;
    std::unordered_set<ClientChannelId> advertisedChannels;

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
  std::map<ConnHandle, std::unordered_map<ClientChannelId, ClientAdvertisement>, std::owner_less<>>
    _clientChannels;
  SubscribeUnsubscribeHandler _subscribeHandler;
  SubscribeUnsubscribeHandler _unsubscribeHandler;
  ClientAdvertiseHandler _clientAdvertiseHandler;
  ClientUnadvertiseHandler _clientUnadvertiseHandler;
  ClientMessageHandler _clientMessageHandler;
  std::shared_mutex _clientsChannelMutex;

  void setupTlsHandler() override;
  void socketInit(ConnHandle hdl);
  bool validateConnection(ConnHandle hdl);
  void handleConnectionOpened(ConnHandle hdl);
  void handleConnectionClosed(ConnHandle hdl);
  void handleMessage(ConnHandle hdl, MessagePtr msg);
  void handleTextMessage(ConnHandle hdl, const std::string& msg);
  void handleBinaryMessage(ConnHandle hdl, const uint8_t* msg, size_t length);

  void sendJson(ConnHandle hdl, json&& payload);
  void sendJsonRaw(ConnHandle hdl, const std::string& payload);
  void sendBinary(ConnHandle hdl, const std::vector<uint8_t>& payload);
  void sendStatus(ConnHandle clientHandle, const StatusLevel level, const std::string& message);
};

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
  _server.get_alog().write(APP, "Rejecting client " + remoteEndpointString(hdl) +
                                  " which did not declare support for subprotocol " +
                                  SUPPORTED_SUBPROTOCOL);
  return false;
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleConnectionOpened(ConnHandle hdl) {
  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  auto con = _server.get_con_from_hdl(hdl);
  const auto endpoint = remoteEndpointString(hdl);
  _server.get_alog().write(APP, "Client " + endpoint + " connected via " + con->get_resource());
  _clients.emplace(hdl, ClientInfo{endpoint, hdl, {}, {}});

  con->send(json({
                   {"op", "serverInfo"},
                   {"name", _name},
                   {"capabilities", json::array({"clientPublish"})},
                 })
              .dump());

  json channels;
  for (const auto& [id, channel] : _channels) {
    (void)id;
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
  std::unordered_set<ClientChannelId> oldAdvertisedChannels;
  std::string clientName;
  {
    std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
    const auto clientIt = _clients.find(hdl);
    if (clientIt == _clients.end()) {
      _server.get_elog().write(RECOVERABLE, "Client " + remoteEndpointString(hdl) +
                                              " disconnected but not found in _clients");
      return;
    }

    const auto& client = clientIt->second;
    clientName = client.name;
    _server.get_alog().write(APP, "Client " + clientName + " disconnected");

    oldSubscriptionsByChannel = std::move(client.subscriptionsByChannel);
    oldAdvertisedChannels = std::move(client.advertisedChannels);
    _clients.erase(clientIt);
  }

  // Unadvertise all channels this client advertised
  for (const auto clientChannelId : oldAdvertisedChannels) {
    _server.get_alog().write(APP, "Client " + clientName + " unadvertising channel " +
                                    std::to_string(clientChannelId) + " due to disconnect");
    if (_clientUnadvertiseHandler) {
      _clientUnadvertiseHandler(clientChannelId, hdl);
    }
  }

  _clientChannels.erase(hdl);

  // Unsubscribe all channels this client subscribed to
  if (_unsubscribeHandler) {
    for (const auto& [chanId, subs] : oldSubscriptionsByChannel) {
      (void)subs;
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
inline void Server<ServerConfiguration>::setClientAdvertiseHandler(ClientAdvertiseHandler handler) {
  _clientAdvertiseHandler = std::move(handler);
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::setClientUnadvertiseHandler(
  ClientUnadvertiseHandler handler) {
  _clientUnadvertiseHandler = std::move(handler);
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::setClientMessageHandler(ClientMessageHandler handler) {
  _clientMessageHandler = std::move(handler);
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
      (void)client;
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
                                   "Terminating connection to " + remoteEndpointString(hdl));
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
inline void Server<ServerConfiguration>::sendStatus(ConnHandle clientHandle,
                                                    const StatusLevel level,
                                                    const std::string& message) {
  const std::string endpoint = remoteEndpointString(clientHandle);
  const std::string logMessage =
    "sendStatus(" + endpoint + ", " + StatusLevelToString(level) + ", " + message + ")";
  _server.get_elog().write(RECOVERABLE, logMessage);
  sendJson(clientHandle, json{
                           {"op", "status"},
                           {"level", static_cast<uint8_t>(level)},
                           {"message", message},
                         });
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleMessage(ConnHandle hdl, MessagePtr msg) {
  const OpCode op = msg->get_opcode();

  try {
    switch (op) {
      case OpCode::TEXT: {
        handleTextMessage(hdl, msg->get_payload());
      } break;
      case OpCode::BINARY: {
        const auto& payload = msg->get_payload();
        handleBinaryMessage(hdl, reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
      } break;
      default:
        break;
    }
  } catch (std::exception const& ex) {
    sendStatus(hdl, StatusLevel::Error, std::string{"Error parsing message: "} + ex.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleTextMessage(ConnHandle hdl, const std::string& msg) {
  const json payload = json::parse(msg);
  const std::string& op = payload.at("op").get<std::string>();

  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);
  auto& clientInfo = _clients.at(hdl);

  const auto findSubscriptionBySubId = [&clientInfo](SubscriptionId subId) {
    return std::find_if(clientInfo.subscriptionsByChannel.begin(),
                        clientInfo.subscriptionsByChannel.end(), [&subId](const auto& mo) {
                          return mo.second == subId;
                        });
  };

  constexpr auto SUBSCRIBE = Integer("subscribe");
  constexpr auto UNSUBSCRIBE = Integer("unsubscribe");
  constexpr auto ADVERTISE = Integer("advertise");
  constexpr auto UNADVERTISE = Integer("unadvertise");

  switch (Integer(op)) {
    case SUBSCRIBE: {
      for (const auto& sub : payload.at("subscriptions")) {
        SubscriptionId subId = sub.at("id");
        ChannelId channelId = sub.at("channelId");
        if (findSubscriptionBySubId(subId) != clientInfo.subscriptionsByChannel.end()) {
          sendStatus(hdl, StatusLevel::Error,
                     "Client subscription id " + std::to_string(subId) +
                       " was already used; ignoring subscription");
          continue;
        }
        const auto& channelIt = _channels.find(channelId);
        if (channelIt == _channels.end()) {
          sendStatus(
            hdl, StatusLevel::Warning,
            "Channel " + std::to_string(channelId) + " is not available; ignoring subscription");
          continue;
        }
        clientInfo.subscriptionsByChannel.emplace(channelId, subId);
        if (_subscribeHandler) {
          _subscribeHandler(channelId, hdl);
        }
      }
    } break;
    case UNSUBSCRIBE: {
      for (const auto& subIdJson : payload.at("subscriptionIds")) {
        SubscriptionId subId = subIdJson;
        const auto& sub = findSubscriptionBySubId(subId);
        if (sub == clientInfo.subscriptionsByChannel.end()) {
          sendStatus(hdl, StatusLevel::Warning,
                     "Client subscription id " + std::to_string(subId) +
                       " did not exist; ignoring unsubscription");
          continue;
        }
        ChannelId chanId = sub->first;
        clientInfo.subscriptionsByChannel.erase(sub);
        if (_unsubscribeHandler) {
          _unsubscribeHandler(chanId, hdl);
        }
      }
    } break;
    case ADVERTISE: {
      auto [clientPublicationsIt, isFirstPublication] =
        _clientChannels.emplace(hdl, std::unordered_map<ClientChannelId, ClientAdvertisement>());

      auto& clientPublications = clientPublicationsIt->second;

      for (const auto& chan : payload.at("channels")) {
        ClientChannelId channelId = chan.at("id");
        if (!isFirstPublication && clientPublications.find(channelId) != clientPublications.end()) {
          sendStatus(hdl, StatusLevel::Error,
                     "Channel " + std::to_string(channelId) + " was already advertised");
          continue;
        }
        ClientAdvertisement advertisement{};
        advertisement.channelId = channelId;
        advertisement.topic = chan.at("topic").get<std::string>();
        advertisement.encoding = chan.at("encoding").get<std::string>();
        advertisement.schemaName = chan.at("schemaName").get<std::string>();
        clientPublications.emplace(channelId, advertisement);
        clientInfo.advertisedChannels.emplace(channelId);
        if (_clientAdvertiseHandler) {
          _clientAdvertiseHandler(advertisement, hdl);
        }
      }
    } break;
    case UNADVERTISE: {
      auto clientPublicationsIt = _clientChannels.find(hdl);
      if (clientPublicationsIt == _clientChannels.end()) {
        sendStatus(hdl, StatusLevel::Error, "Client has no advertised channels");
        break;
      }

      auto& clientPublications = clientPublicationsIt->second;

      for (const auto& chanIdJson : payload.at("channelIds")) {
        ClientChannelId channelId = chanIdJson.get<ClientChannelId>();
        const auto& channelIt = clientPublications.find(channelId);
        if (channelIt == clientPublications.end()) {
          continue;
        }
        clientPublications.erase(channelIt);
        if (_clientUnadvertiseHandler) {
          _clientUnadvertiseHandler(channelId, hdl);
        }
      }
    } break;
    default: {
      sendStatus(hdl, StatusLevel::Error, "Unrecognized client opcode \"" + op + "\"");
    } break;
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleBinaryMessage(ConnHandle hdl, const uint8_t* msg,
                                                             size_t length) {
  const uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::high_resolution_clock::now().time_since_epoch())
                               .count();

  if (length < 1) {
    sendStatus(hdl, StatusLevel::Error, "Received an empty binary message");
    return;
  }

  std::unique_lock<std::shared_mutex> lock(_clientsChannelMutex);

  auto clientPublicationsIt = _clientChannels.find(hdl);
  if (clientPublicationsIt == _clientChannels.end()) {
    sendStatus(hdl, StatusLevel::Error, "Client has no advertised channels");
    return;
  }

  auto& clientPublications = clientPublicationsIt->second;

  const auto op = static_cast<ClientBinaryOpcode>(msg[0]);
  switch (op) {
    case ClientBinaryOpcode::MESSAGE_DATA: {
      if (length < 5) {
        sendStatus(hdl, StatusLevel::Error, "Invalid message length " + std::to_string(length));
        return;
      }
      const ClientChannelId channelId = *reinterpret_cast<const ClientChannelId*>(msg + 1);
      const auto& channelIt = clientPublications.find(channelId);
      if (channelIt == clientPublications.end()) {
        sendStatus(hdl, StatusLevel::Error,
                   "Channel " + std::to_string(channelId) + " is not advertised");
        return;
      }

      if (_clientMessageHandler) {
        const auto& advertisement = channelIt->second;
        const uint32_t sequence = 0;
        const ClientMessage clientMessage{timestamp,     timestamp, sequence,
                                          advertisement, length,    msg};
        _clientMessageHandler(clientMessage, hdl);
      }
    } break;
    default: {
      sendStatus(hdl, StatusLevel::Error,
                 "Unrecognized client opcode " + std::to_string(uint8_t(op)));
    } break;
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
    (void)id;
    channels.push_back(channel);
  }
  std::string msg = json{{"op", "advertise"}, {"channels", std::move(channels)}}.dump();

  for (const auto& [hdl, clientInfo] : _clients) {
    (void)clientInfo;
    sendJsonRaw(hdl, msg);
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

template <typename ServerConfiguration>
inline std::string Server<ServerConfiguration>::remoteEndpointString(ConnHandle clientHandle) {
  std::error_code ec;
  const auto con = _server.get_con_from_hdl(clientHandle, ec);
  return con ? con->get_remote_endpoint() : "(unknown)";
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

      // Ciphers are taken from the websocketpp example echo tls server:
      // https://github.com/zaphoyd/websocketpp/blob/1b11fd301/examples/echo_server_tls/echo_server_tls.cpp#L119
      constexpr char ciphers[] =
        "ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384:"
        "ECDHE-ECDSA-AES256-GCM-SHA384:DHE-RSA-AES128-GCM-SHA256:DHE-DSS-AES128-GCM-SHA256:kEDH+"
        "AESGCM:ECDHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA:ECDHE-ECDSA-"
        "AES128-SHA:ECDHE-RSA-AES256-SHA384:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA:ECDHE-"
        "ECDSA-AES256-SHA:DHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA:DHE-DSS-AES128-SHA256:DHE-RSA-"
        "AES256-SHA256:DHE-DSS-AES256-SHA:DHE-RSA-AES256-SHA:!aNULL:!eNULL:!EXPORT:!DES:!RC4:!3DES:"
        "!MD5:!PSK";

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

extern template class Server<WebSocketTls>;
extern template class Server<WebSocketNoTls>;

}  // namespace foxglove
