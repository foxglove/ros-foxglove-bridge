#pragma once

#include <websocketpp/config/asio.hpp>

#include "./websocket_logging.hpp"

namespace foxglove {

struct WebSocketTls : public websocketpp::config::asio_tls {
public:
  // Replace default stream loggers with custom loggers
  typedef CallbackLogger<concurrency_type, websocketpp::log::elevel> elog_type;
  typedef CallbackLogger<concurrency_type, websocketpp::log::alevel> alog_type;
};

}  // namespace foxglove
