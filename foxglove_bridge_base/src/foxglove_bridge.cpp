#include "foxglove_bridge/foxglove_bridge.hpp"

#define ASIO_STANDALONE
#include "foxglove_bridge/websocket_notls.hpp"
#include "foxglove_bridge/websocket_server.hpp"
#include "foxglove_bridge/websocket_tls.hpp"

namespace foxglove {

const char* WebSocketUserAgent() {
  return websocketpp::user_agent;
}

// Explicit template instantiation for common server configurations
template class Server<WebSocketTls>;
template class Server<WebSocketNoTls>;

}  // namespace foxglove
