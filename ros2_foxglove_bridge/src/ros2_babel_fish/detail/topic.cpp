//
// Created by Stefan Fabian on 27.07.21.
//

#include "ros2_babel_fish/detail/topic.hpp"

#include <rclcpp/graph_listener.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/wait_set.hpp>

namespace ros2_babel_fish
{
namespace impl
{
namespace
{
bool has_topic( const rclcpp::Node &node, const std::string &topic, std::vector<std::string> &types )
{
  const std::map<std::string, std::vector<std::string>> &topics = node.get_topic_names_and_types();
  auto it = std::find_if( topics.begin(), topics.end(),
                          [&topic]( const auto &entry ) { return entry.first == topic; } );
  if ( it == topics.end() )
    return false;
  types = it->second;
  return true;
}
} // namespace

bool wait_for_topic_and_type_nanoseconds( rclcpp::Node &node, const std::string &topic,
                                          std::vector<std::string> &types,
                                          std::chrono::nanoseconds timeout )
{
  auto start = std::chrono::steady_clock::now();
  auto event = node.get_graph_event();
  if ( has_topic( node, topic, types ) )
    return true;
  if ( timeout == std::chrono::nanoseconds( 0 ) ) {
    // check was non-blocking, return immediately
    return false;
  }
  std::chrono::nanoseconds time_to_wait = timeout > std::chrono::nanoseconds( 0 )
                                              ? timeout - ( std::chrono::steady_clock::now() - start )
                                              : std::chrono::nanoseconds::max();
  if ( time_to_wait < std::chrono::nanoseconds( 0 ) ) {
    // check consumed entire timeout, return immediately
    return false;
  }
  do {
    if ( !rclcpp::ok() ) {
      return false;
    }
    // Limit each wait to 100ms to workaround an issue specific to the Connext RMW implementation.
    // A race condition means that graph changes for services becoming available may trigger the
    // wait set to wake up, but then not be reported as ready immediately after the wake up
    // (see https://github.com/ros2/rmw_connext/issues/201)
    // If no other graph events occur, the wait set will not be triggered again until the timeout
    // has been reached, despite the service being available, so we artificially limit the wait
    // time to limit the delay.
    node.wait_for_graph_change(
        event, std::min( time_to_wait, std::chrono::nanoseconds( RCL_MS_TO_NS( 100 ) ) ) );
    // Because of the aforementioned race condition, we check if the topic is available even if the
    // graph event wasn't triggered.
    event->check_and_clear();
    if ( has_topic( node, topic, types ) )
      return true;

    // topic not available, wait if a timeout was specified
    if ( timeout > std::chrono::nanoseconds( 0 ) ) {
      time_to_wait = timeout - ( std::chrono::steady_clock::now() - start );
    }
  } while ( time_to_wait > std::chrono::nanoseconds( 0 ) );
  return false; // timeout exceeded while waiting for the topic
}

bool wait_for_topic_nanoseconds( rclcpp::Node &node, const std::string &topic,
                                 std::chrono::nanoseconds timeout )
{
  std::vector<std::string> types;
  return wait_for_topic_and_type_nanoseconds( node, topic, types, timeout );
}
} // namespace impl

std::string resolve_topic( const rclcpp::Node &node, const std::string &topic )
{
  std::string resolved_topic =
      rclcpp::extend_name_with_sub_namespace( topic, node.get_sub_namespace() );
  // Until >=galactic we need to expand it manually // TODO Check how to solve this in galactic
  if ( !resolved_topic.empty() && resolved_topic.front() == '~' ) {
    resolved_topic = std::string( node.get_fully_qualified_name() ) + resolved_topic.substr( 1 );
  }
  //  char * output_cstr = nullptr;
  //  auto allocator = rcl_get_default_allocator();
  //  rcl_ret_t ret = rcl_node_resolve_name(
  //    node.get(),
  //    topic_.c_str(),
  //    allocator,
  //    false,
  //    true,
  //    &output_cstr);
  //  if (RCL_RET_OK != ret) {
  //    throw_from_rcl_error(ret, "failed to resolve name", rcl_get_error_state());
  //  }
  //  topic_ = output_cstr
  //  allocator.deallocate(output_cstr, allocator.state);
  return resolved_topic;
}
} // namespace ros2_babel_fish
