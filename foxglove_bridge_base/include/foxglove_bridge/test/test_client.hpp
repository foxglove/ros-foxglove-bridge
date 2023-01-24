#pragma once

#include <future>
#include <string>
#include <vector>

#include <websocketpp/config/asio_client.hpp>

#include "../parameter.hpp"
#include "../websocket_client.hpp"

namespace foxglove {

std::vector<uint8_t> connectClientAndReceiveMsg(const std::string& uri,
                                                const std::string& topic_name);

std::future<std::vector<Parameter>> waitForParameters(std::shared_ptr<ClientInterface> client,
                                                      const std::string& requestId = std::string());

extern template class Client<websocketpp::config::asio_client>;

}  // namespace foxglove
