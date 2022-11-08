#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "./websocket_logging.hpp"

namespace foxglove {

struct WebSocketNoTls : public websocketpp::config::asio {
public:
  // Replace default stream loggers with custom loggers
  typedef CallbackLogger<concurrency_type, websocketpp::log::elevel> elog_type;
  typedef CallbackLogger<concurrency_type, websocketpp::log::alevel> alog_type;
};

}  // namespace foxglove
