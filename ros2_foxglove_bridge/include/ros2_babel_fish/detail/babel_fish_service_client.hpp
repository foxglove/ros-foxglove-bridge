// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_SERVICE_CLIENT_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_SERVICE_CLIENT_HPP

#include <rclcpp/node.hpp>
#include <ros2_babel_fish/idl/type_support.hpp>
#include <ros2_babel_fish/messages/compound_message.hpp>

namespace ros2_babel_fish
{

class BabelFishServiceClient : public rclcpp::ClientBase
{
public:
  using SharedRequest = CompoundMessage::SharedPtr;
  using SharedResponse = CompoundMessage::SharedPtr;

  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;
  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;
  using CallbackType = std::function<void( SharedFuture )>;
  using CallbackWithRequestType = std::function<void( const SharedFutureWithRequest & )>;

  RCLCPP_SMART_PTR_DEFINITIONS( BabelFishServiceClient )

  BabelFishServiceClient( rclcpp::node_interfaces::NodeBaseInterface *node_base,
                          rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                          const std::string &service_name,
                          ServiceTypeSupport::ConstSharedPtr type_support,
                          rcl_client_options_t client_options );

  bool take_response( CompoundMessage &response_out, rmw_request_id_t &request_header_out );

  std::shared_ptr<void> create_response() override;

  std::shared_ptr<rmw_request_id_t> create_request_header() override;

  SharedFuture async_send_request( SharedRequest request );

  template<typename CallbackT, typename std::enable_if<rclcpp::function_traits::same_arguments<
                                   CallbackT, CallbackType>::value>::type * = nullptr>
  SharedFuture async_send_request( const SharedRequest &request, CallbackT &&cb )
  {
    std::lock_guard<std::mutex> lock( pending_requests_mutex_ );
    int64_t sequence_number;
    rcl_ret_t ret = rcl_send_request( get_client_handle().get(),
                                      request->type_erased_message().get(), &sequence_number );
    if ( RCL_RET_OK != ret ) {
      rclcpp::exceptions::throw_from_rcl_error( ret, "failed to send request_template" );
    }

    SharedPromise call_promise = std::make_shared<Promise>();
    SharedFuture f( call_promise->get_future() );
    pending_requests_[sequence_number] =
        std::make_tuple( call_promise, std::forward<CallbackType>( cb ), f );
    return f;
  }

  template<typename CallbackT, typename std::enable_if<rclcpp::function_traits::same_arguments<
                                   CallbackT, CallbackWithRequestType>::value>::type * = nullptr>
  SharedFutureWithRequest async_send_request( SharedRequest request, CallbackT &&cb )
  {
    SharedPromiseWithRequest promise = std::make_shared<PromiseWithRequest>();
    SharedFutureWithRequest future_with_request( promise->get_future() );

    auto wrapping_cb = [future_with_request, promise, request, &cb]( const SharedFuture &future ) {
      auto response = future.get();
      promise->set_value( std::make_pair( request, response ) );
      cb( future_with_request );
    };

    async_send_request( request, wrapping_cb );

    return future_with_request;
  }

  void handle_response( std::shared_ptr<rmw_request_id_t> request_header,
                        std::shared_ptr<void> response ) override;

private:
  RCLCPP_DISABLE_COPY( BabelFishServiceClient )

  std::map<int64_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
  ServiceTypeSupport::ConstSharedPtr type_support_;
  std::mutex pending_requests_mutex_;
};
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_BABEL_FISH_SERVICE_CLIENT_HPP
