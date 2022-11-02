#include "foxglove_bridge/foxglove_bridge.hpp"

#define ASIO_STANDALONE
#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>

using Server = websocketpp::server<websocketpp::config::asio_tls>;
using ConnectionHdl = websocketpp::connection_hdl;
using SslContext = websocketpp::lib::asio::ssl::context;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

const char* foxglove::WebSocketUserAgent() {
  return websocketpp::user_agent;
}
