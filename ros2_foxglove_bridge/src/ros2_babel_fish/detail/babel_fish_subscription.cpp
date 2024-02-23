//
// Created by stefan on 22.01.21.
//

#include "ros2_babel_fish/detail/babel_fish_subscription.hpp"
#include "../logging.hpp"
#include "ros2_babel_fish/idl/serialization.hpp"

#include <rcl/rcl.h>
#include <rclcpp/node.hpp>

namespace ros2_babel_fish
{
namespace
{
rcl_subscription_options_t create_subscription_options( const rclcpp::QoS &qos )
{
  auto options = rcl_subscription_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}
} // namespace

BabelFishSubscription::BabelFishSubscription(
    rclcpp::node_interfaces::NodeBaseInterface *node_base,
    MessageTypeSupport::ConstSharedPtr type_support, const std::string &topic_name,
    const rclcpp::QoS &qos,
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> &options )
    : rclcpp::SubscriptionBase( node_base, type_support->type_support_handle, topic_name,
                                create_subscription_options( qos ), false ),
      type_support_( std::move( type_support ) ), callback_( callback )
{

  if ( options.event_callbacks.deadline_callback ) {
    this->add_event_handler( options.event_callbacks.deadline_callback,
                             RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED );
  }
  if ( options.event_callbacks.liveliness_callback ) {
    this->add_event_handler( options.event_callbacks.liveliness_callback,
                             RCL_SUBSCRIPTION_LIVELINESS_CHANGED );
  }
  if ( options.event_callbacks.incompatible_qos_callback ) {
    this->add_event_handler( options.event_callbacks.incompatible_qos_callback,
                             RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS );
  } else if ( options.use_default_callbacks ) {
    // Register default callback when not specified
    try {
      this->add_event_handler(
          [this]( rclcpp::QOSRequestedIncompatibleQoSInfo &info ) {
            this->default_incompatible_qos_callback( info );
          },
          RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS );
    } catch ( rclcpp::UnsupportedEventTypeException & /*exc*/ ) {
      // pass
    }
  }
  if ( options.event_callbacks.message_lost_callback ) {
    this->add_event_handler( options.event_callbacks.message_lost_callback,
                             RCL_SUBSCRIPTION_MESSAGE_LOST );
  }
#if RCLCPP_VERSION_MAJOR >= 9
  if ( options.event_callbacks.message_lost_callback ) {
    this->add_event_handler( options.event_callbacks.message_lost_callback,
                             RCL_SUBSCRIPTION_MESSAGE_LOST );
  }
#endif

  // Setup intra process publishing if requested.
  if ( rclcpp::detail::resolve_use_intra_process( options, *node_base ) ) {
    using rclcpp::detail::resolve_intra_process_buffer_type;

    // Check if the QoS is compatible with intra-process.
    auto qos_profile = get_actual_qos();
    if ( qos_profile.history() != rclcpp::HistoryPolicy::KeepLast ) {
      throw std::invalid_argument(
          "intraprocess communication is not allowed with keep all history qos policy" );
    }
    if ( qos_profile.depth() == 0 ) {
      throw std::invalid_argument(
          "intraprocess communication is not allowed with 0 depth qos policy" );
    }
    if ( qos_profile.durability() != rclcpp::DurabilityPolicy::Volatile ) {
      throw std::invalid_argument(
          "intraprocess communication allowed only with volatile durability" );
    }

    // TODO: Adding intra process support [very low priority]
    //    // First create a SubscriptionIntraProcess which will be given to the intra-process manager.
    //    auto context = node_base->get_context();
    //    subscription_intra_process_ = std::make_shared<SubscriptionIntraProcessT>(
    //      callback,
    //      options.get_allocator(),
    //      context,
    //      this->get_topic_name(),  // important to get like this, as it has the fully-qualified name
    //      qos_profile,
    //      resolve_intra_process_buffer_type( options.intra_process_buffer_type, callback ));
    //    TRACEPOINT(
    //      rclcpp_subscription_init,
    //      static_cast<const void *>(get_subscription_handle().get()),
    //      static_cast<const void *>(subscription_intra_process_.get()));
    //
    //    // Add it to the intra process manager.
    //    using rclcpp::experimental::IntraProcessManager;
    //    auto ipm = context->get_sub_context<IntraProcessManager>();
    //    uint64_t intra_process_subscription_id = ipm->add_subscription( subscription_intra_process_ );
    //    this->setup_intra_process( intra_process_subscription_id, ipm );
  }

  TRACEPOINT( rclcpp_subscription_init, static_cast<const void *>( get_subscription_handle().get() ),
              static_cast<const void *>( this ) );
  TRACEPOINT( rclcpp_subscription_callback_added, static_cast<const void *>( this ),
              static_cast<const void *>( &callback_ ) );
  // The callback object gets copied, so if registration is done too early/before this point
  // (e.g. in `AnySubscriptionCallback::set()`), its address won't match any address used later
  // in subsequent tracepoints.
#ifndef TRACETOOLS_DISABLED
  callback_.register_callback_for_tracing();
#endif
  RBF2_DEBUG_STREAM( "Subscribed to: " << topic_name );
}

BabelFishSubscription::~BabelFishSubscription()
{
  RBF2_DEBUG_STREAM( "Unsubscribed from: " << get_topic_name() );
}

std::shared_ptr<void> BabelFishSubscription::create_message()
{
  return createContainer( *type_support_ );
}

std::shared_ptr<rclcpp::SerializedMessage> BabelFishSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>( 0 );
}

void BabelFishSubscription::handle_message( std::shared_ptr<void> &message,
                                            const rclcpp::MessageInfo &message_info )
{
  callback_.dispatch( CompoundMessage::make_shared( *type_support_, message ), message_info );
}

void BabelFishSubscription::handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
    const rclcpp::MessageInfo &message_info )
{
  callback_.dispatch( serialized_message, message_info );
}

void BabelFishSubscription::handle_loaned_message( void *loaned_message,
                                                   const rclcpp::MessageInfo &message_info )
{
  // Not handled
  (void)loaned_message;
  (void)message_info;
}

void BabelFishSubscription::return_message( std::shared_ptr<void> &message )
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>( message );
  return_serialized_message( typed_message );
}

void BabelFishSubscription::return_serialized_message( std::shared_ptr<rclcpp::SerializedMessage> &message )
{
  message.reset();
}

bool BabelFishSubscription::take( CompoundMessage &message_out, rclcpp::MessageInfo &info_out )
{
  std::shared_ptr<void> type_erased = create_message();
  if ( type_erased == nullptr || !take_type_erased( type_erased.get(), info_out ) )
    return false;
  message_out = CompoundMessage( *type_support_, std::move( type_erased ) );
  return true;
}

MessageTypeSupport::ConstSharedPtr BabelFishSubscription::get_message_type_support() const
{
  return type_support_;
}

std::string BabelFishSubscription::get_message_type() const { return type_support_->name; }
} // namespace ros2_babel_fish
