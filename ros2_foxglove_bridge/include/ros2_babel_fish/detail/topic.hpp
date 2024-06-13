//
// Created by Stefan Fabian on 27.07.21.
//

#ifndef ROS2_BABEL_FISH_TOPIC_HPP
#define ROS2_BABEL_FISH_TOPIC_HPP

#include <chrono>
#include <string>
#include <vector>

namespace rclcpp {
class Node;
}

namespace ros2_babel_fish {
namespace impl {
bool wait_for_topic_nanoseconds(rclcpp::Node& node, const std::string& topic,
                                std::chrono::nanoseconds timeout);

bool wait_for_topic_and_type_nanoseconds(rclcpp::Node& node, const std::string& topic,
                                         std::vector<std::string>& types,
                                         std::chrono::nanoseconds timeout);
}  // namespace impl

template <typename RepT, typename PeriodT>
bool wait_for_topic(
  rclcpp::Node& node, const std::string& topic,
  std::chrono::duration<RepT, PeriodT> timeout = std::chrono::duration<RepT, PeriodT>(-1)) {
  return impl::wait_for_topic_nanoseconds(
    node, topic, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
}

template <typename RepT, typename PeriodT>
bool wait_for_topic_and_type(
  rclcpp::Node& node, const std::string& topic, std::vector<std::string>& types,
  std::chrono::duration<RepT, PeriodT> timeout = std::chrono::duration<RepT, PeriodT>(-1)) {
  return impl::wait_for_topic_and_type_nanoseconds(
    node, topic, types, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
}

std::string resolve_topic(const rclcpp::Node& node, const std::string& topic);
}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_TOPIC_HPP
