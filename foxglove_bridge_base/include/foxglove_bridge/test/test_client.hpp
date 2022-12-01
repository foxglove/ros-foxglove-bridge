#pragma once

#include <string>
#include <vector>

#include <websocketpp/config/asio_client.hpp>

#include "../websocket_client.hpp"

namespace foxglove {

std::vector<uint8_t> connectClientAndReceiveMsg(const std::string& uri,
                                                const std::string& topic_name);

extern template class Client<websocketpp::config::asio_client>;

}  // namespace foxglove
