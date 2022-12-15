#pragma once

#include <regex>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

namespace foxglove_bridge {

constexpr char PARAM_PORT[] = "port";
constexpr char PARAM_ADDRESS[] = "address";
constexpr char PARAM_USETLS[] = "tls";
constexpr char PARAM_CERTFILE[] = "certfile";
constexpr char PARAM_KEYFILE[] = "keyfile";
constexpr char PARAM_MAX_QOS_DEPTH[] = "max_qos_depth";
constexpr char PARAM_TOPIC_WHITELIST[] = "topic_whitelist";
constexpr char PARAM_BEST_EFFORT_TOPICS[] = "best_effort_topics";
constexpr char PARAM_BEST_EFFORT_BUFF_SIZE_LIMIT[] = "best_effort_max_buff_size";

constexpr int64_t DEFAULT_PORT = 8765;
constexpr char DEFAULT_ADDRESS[] = "0.0.0.0";
constexpr int64_t DEFAULT_MAX_QOS_DEPTH = 10;
constexpr int64_t DEFAULT_BEST_EFFORT_BUFF_SIZE_LIMIT_KB = 1000;

void declareParameters(rclcpp::Node* node);

std::vector<std::regex> parseRegexStrings(rclcpp::Node* node,
                                          const std::vector<std::string>& strings);

}  // namespace foxglove_bridge
