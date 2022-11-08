#pragma once

#include <functional>

#include <asio/ip/address.hpp>
#include <websocketpp/logger/basic.hpp>

namespace foxglove {

enum class WebSocketLogLevel {
  Debug,
  Info,
  Warn,
  Error,
  Critical,
};

using LogCallback = std::function<void(WebSocketLogLevel, char const*)>;

inline std::string IPAddressToString(const asio::ip::address& addr) {
  if (addr.is_v6()) {
    return "[" + addr.to_string() + "]";
  }
  return addr.to_string();
}

inline void NoOpLogCallback(WebSocketLogLevel, char const*) {}

template <typename concurrency, typename names>
class CallbackLogger : public websocketpp::log::basic<concurrency, names> {
public:
  using channel_type_hint = websocketpp::log::channel_type_hint;

  CallbackLogger(channel_type_hint::value hint = channel_type_hint::access)
      : websocketpp::log::basic<concurrency, names>(hint)
      , _channelTypeHint(hint)
      , _callback(NoOpLogCallback) {}

  CallbackLogger(websocketpp::log::level channels,
                 channel_type_hint::value hint = channel_type_hint::access)
      : websocketpp::log::basic<concurrency, names>(channels, hint)
      , _channelTypeHint(hint)
      , _callback(NoOpLogCallback) {}

  void set_callback(LogCallback callback) {
    _callback = callback;
  }

  void write(websocketpp::log::level channel, std::string const& msg) {
    write(channel, msg.c_str());
  }

  void write(websocketpp::log::level channel, char const* msg) {
    if (!this->dynamic_test(channel)) {
      return;
    }

    if (_channelTypeHint == channel_type_hint::access) {
      _callback(WebSocketLogLevel::Info, msg);
    } else {
      if (channel == websocketpp::log::elevel::devel) {
        _callback(WebSocketLogLevel::Debug, msg);
      } else if (channel == websocketpp::log::elevel::library) {
        _callback(WebSocketLogLevel::Debug, msg);
      } else if (channel == websocketpp::log::elevel::info) {
        _callback(WebSocketLogLevel::Info, msg);
      } else if (channel == websocketpp::log::elevel::warn) {
        _callback(WebSocketLogLevel::Warn, msg);
      } else if (channel == websocketpp::log::elevel::rerror) {
        _callback(WebSocketLogLevel::Error, msg);
      } else if (channel == websocketpp::log::elevel::fatal) {
        _callback(WebSocketLogLevel::Critical, msg);
      }
    }
  }

private:
  channel_type_hint::value _channelTypeHint;
  LogCallback _callback;
};

}  // namespace foxglove
