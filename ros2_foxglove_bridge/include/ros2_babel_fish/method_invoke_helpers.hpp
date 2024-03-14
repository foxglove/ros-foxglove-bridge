//
// Created by Stefan Fabian on 02.08.21.
//

#ifndef ROS2_BABEL_FISH_METHOD_INVOKE_HELPERS_HPP
#define ROS2_BABEL_FISH_METHOD_INVOKE_HELPERS_HPP

#include <functional>

#include "ros2_babel_fish/messages/array_message.hpp"
#include "ros2_babel_fish/messages/compound_message.hpp"

namespace ros2_babel_fish {
// Define invoke function with a "polyfill" for CPP versions below C++17
// Macro is unset at the end of the header
#if __cplusplus >= 201700L

#define RBF2_INVOKE_FN std::invoke

#else

namespace impl {
template <typename Fn, typename... Args,
          std::enable_if_t<std::is_member_pointer<std::decay_t<Fn>>{}, int> = 0>
constexpr decltype(auto) invoke(Fn&& f, Args&&... args) noexcept(
  noexcept(std::mem_fn(f)(std::forward<Args>(args)...))) {
  return std::mem_fn(f)(std::forward<Args>(args)...);
}

template <typename Fn, typename... Args,
          std::enable_if_t<!std::is_member_pointer<std::decay_t<Fn>>{}, int> = 0>
constexpr decltype(auto) invoke(Fn&& f, Args&&... args) noexcept(
  noexcept(std::forward<Fn>(f)(std::forward<Args>(args)...))) {
  return std::forward<Fn>(f)(std::forward<Args>(args)...);
}
}  // namespace impl
#define RBF2_INVOKE_FN ::ros2_babel_fish::impl::invoke

#endif

/*!
 * @brief Invokes the given functor with the given message casted to the actual type of the given
 * message and the optionally passed additional arguments. For array messages the method is invoked
 * with the given message casted to ArrayMessageBase. This can be used with invoke_for_array_message
 * to call for the concrete array message type.
 * @return The return value of the invoked method.
 */
template <typename Callable, typename... Args>
auto invoke_for_message(Message& msg, Callable&& f, Args&&... args);

//! @copydoc invoke_for_message
template <typename Callable, typename... Args>
auto invoke_for_message(const Message& msg, Callable&& f, Args&&... args);

/*!
 * @brief Invokes the given functor with the given value message casted to the actual type of the
 * given message and the optionally passed additional arguments.
 *
 * @return The return value of the invoked method.
 * @throws BabelFishException If the message is not a ValueMessage.
 */
template <typename Callable, typename... Args>
auto invoke_for_value_message(Message& msg, Callable&& f, Args&&... args);

//! @copydoc invoke_for_value_message
template <typename Callable, typename... Args>
auto invoke_for_value_message(const Message& msg, Callable&& f, Args&&... args);

/*!
 * @brief Invokes the given functor with the given message casted to the actual type of the given
 * array message and the optionally passed additional arguments.
 *
 * @return The return value of the invoked method.
 */
template <typename Callable, typename... Args>
auto invoke_for_array_message(ArrayMessageBase& msg, Callable&& f, Args&&... args);

//! @copydoc invoke_for_array_message
template <typename Callable, typename... Args>
auto invoke_for_array_message(const ArrayMessageBase& msg, Callable&& f, Args&&... args);

// ==============================================
// =============== IMPLEMENTATION ===============
// ==============================================

template <typename Callable, typename... Args>
auto invoke_for_message(Message& msg, Callable&& f, Args&&... args) {
  using namespace ::ros2_babel_fish::message_type_traits;
  using ::ros2_babel_fish::ValueMessage;
  switch (msg.type()) {
    case MessageTypes::Float:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Float>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Double:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Double>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::LongDouble:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::LongDouble>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Char:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Char>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WChar:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WChar>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Bool:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Bool>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Octet:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Octet>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::String:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::String>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WString:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WString>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Compound:
      return RBF2_INVOKE_FN(f, msg.as<CompoundMessage>(), std::forward<Args>(args)...);
    case MessageTypes::Array:
      return RBF2_INVOKE_FN(f, msg.as<ArrayMessageBase>(), std::forward<Args>(args)...);
    case MessageTypes::None:
    default:
      throw BabelFishException("invoke_for_message called with invalid message!");
  }
}

template <typename Callable, typename... Args>
auto invoke_for_message(const Message& msg, Callable&& f, Args&&... args) {
  using namespace ::ros2_babel_fish::message_type_traits;
  using ::ros2_babel_fish::ValueMessage;
  switch (msg.type()) {
    case MessageTypes::Float:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Float>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Double:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Double>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::LongDouble:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::LongDouble>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Char:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Char>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WChar:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WChar>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Bool:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Bool>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Octet:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Octet>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::String:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::String>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WString:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WString>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Compound:
      return RBF2_INVOKE_FN(f, msg.as<CompoundMessage>(), std::forward<Args>(args)...);
    case MessageTypes::Array:
      return RBF2_INVOKE_FN(f, msg.as<ArrayMessageBase>(), std::forward<Args>(args)...);
    case MessageTypes::None:
    default:
      throw BabelFishException("invoke_for_message called with invalid message!");
  }
}

template <typename Callable, typename... Args>
auto invoke_for_value_message(Message& msg, Callable&& f, Args&&... args) {
  using namespace ::ros2_babel_fish::message_type_traits;
  using ::ros2_babel_fish::ValueMessage;
  switch (msg.type()) {
    case MessageTypes::Float:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Float>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Double:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Double>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::LongDouble:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::LongDouble>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Char:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Char>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WChar:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WChar>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Bool:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Bool>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Octet:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Octet>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::String:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::String>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WString:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WString>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Compound:
    case MessageTypes::Array:
      throw BabelFishException(
        "invoke_for_value_message called with message that is not a ValueMessage!");
    case MessageTypes::None:
    default:
      throw BabelFishException("invoke_for_value_message called with invalid message!");
  }
}

template <typename Callable, typename... Args>
auto invoke_for_value_message(const Message& msg, Callable&& f, Args&&... args) {
  using namespace ::ros2_babel_fish::message_type_traits;
  using ::ros2_babel_fish::ValueMessage;
  switch (msg.type()) {
    case MessageTypes::Float:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Float>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Double:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Double>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::LongDouble:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::LongDouble>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Char:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Char>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WChar:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WChar>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Bool:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Bool>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Octet:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Octet>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int8:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int8>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int16:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int16>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int32:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int32>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::UInt64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::UInt64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Int64:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::Int64>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::String:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::String>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::WString:
      return RBF2_INVOKE_FN(f, msg.as<ValueMessage<value_type<MessageTypes::WString>::value>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Compound:
    case MessageTypes::Array:
      throw BabelFishException(
        "invoke_for_value_message called with message that is not a ValueMessage!");
    case MessageTypes::None:
    default:
      throw BabelFishException("invoke_for_value_message called with invalid message!");
  }
}

namespace impl {
template <bool BOUNDED, bool FIXED_LENGTH, typename Callable, typename... Args>
auto call_for_array_message(ArrayMessageBase& msg, Callable&& f, Args&&... args) {
  using namespace ::ros2_babel_fish::message_type_traits;
  using ::ros2_babel_fish::ArrayMessage_;
  using ::ros2_babel_fish::CompoundArrayMessage_;
  switch (msg.elementType()) {
    case MessageTypes::Float:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Float>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Double:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Double>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::LongDouble:
      return RBF2_INVOKE_FN(
        f,
        msg.as<ArrayMessage_<value_type<MessageTypes::LongDouble>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Char:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Char>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::WChar:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::WChar>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Bool:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Bool>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Octet:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Octet>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt8:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt8>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int8:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int8>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt16:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt16>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int16:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int16>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt32:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt32>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int32:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int32>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt64:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt64>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int64:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int64>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::String:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::String>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::WString:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::WString>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Compound:
      return RBF2_INVOKE_FN(f, msg.as<CompoundArrayMessage_<BOUNDED, FIXED_LENGTH>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Array:
      throw BabelFishException("Arrays of arrays are not supported in ROS2!");
    case MessageTypes::None:
    default:
      throw BabelFishException("invoke_for_array_message called with invalid message!");
  }
}

template <bool BOUNDED, bool FIXED_LENGTH, typename Callable, typename... Args>
auto call_for_array_message(const ArrayMessageBase& msg, Callable&& f, Args&&... args) {
  using namespace ::ros2_babel_fish::message_type_traits;
  using ::ros2_babel_fish::ArrayMessage_;
  using ::ros2_babel_fish::CompoundArrayMessage_;
  switch (msg.elementType()) {
    case MessageTypes::Float:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Float>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Double:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Double>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::LongDouble:
      return RBF2_INVOKE_FN(
        f,
        msg.as<ArrayMessage_<value_type<MessageTypes::LongDouble>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Char:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Char>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::WChar:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::WChar>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Bool:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Bool>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Octet:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Octet>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt8:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt8>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int8:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int8>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt16:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt16>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int16:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int16>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt32:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt32>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int32:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int32>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::UInt64:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::UInt64>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Int64:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::Int64>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::String:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::String>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::WString:
      return RBF2_INVOKE_FN(
        f, msg.as<ArrayMessage_<value_type<MessageTypes::WString>::value, BOUNDED, FIXED_LENGTH>>(),
        std::forward<Args>(args)...);
    case MessageTypes::Compound:
      return RBF2_INVOKE_FN(f, msg.as<CompoundArrayMessage_<BOUNDED, FIXED_LENGTH>>(),
                            std::forward<Args>(args)...);
    case MessageTypes::Array:
      throw BabelFishException("Arrays of arrays are not supported in ROS2!");
    case MessageTypes::None:
    default:
      throw BabelFishException("invoke_for_array_message called with invalid message!");
  }
}
}  // namespace impl

template <typename Callable, typename... Args>
auto invoke_for_array_message(ArrayMessageBase& msg, Callable&& f, Args&&... args) {
  if (msg.isFixedSize())
    return impl::call_for_array_message<true, true>(msg, std::forward<Callable>(f),
                                                    std::forward<Args>(args)...);
  if (msg.isBounded())
    return impl::call_for_array_message<true, false>(msg, std::forward<Callable>(f),
                                                     std::forward<Args>(args)...);
  return impl::call_for_array_message<false, false>(msg, std::forward<Callable>(f),
                                                    std::forward<Args>(args)...);
}

template <typename Callable, typename... Args>
auto invoke_for_array_message(const ArrayMessageBase& msg, Callable&& f, Args&&... args) {
  if (msg.isFixedSize())
    return impl::call_for_array_message<true, true>(msg, std::forward<Callable>(f),
                                                    std::forward<Args>(args)...);
  if (msg.isBounded())
    return impl::call_for_array_message<true, false>(msg, std::forward<Callable>(f),
                                                     std::forward<Args>(args)...);
  return impl::call_for_array_message<false, false>(msg, std::forward<Callable>(f),
                                                    std::forward<Args>(args)...);
}
}  // namespace ros2_babel_fish

#undef RBF2_INVOKE_FN

#endif  // ROS2_BABEL_FISH_METHOD_INVOKE_HELPERS_HPP
