// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_MACROS_HPP
#define ROS2_BABEL_FISH_MACROS_HPP

#include "ros2_babel_fish/messages/message_type_traits.hpp"

#define RBF2_TEMPLATE_CALL(function, type, ...)                                                    \
  do {                                                                                             \
    switch (type) {                                                                                \
      case MessageTypes::Compound:                                                                 \
        function<                                                                                  \
          ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Compound>::value>(      \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Array:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Array>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Bool:                                                                     \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Bool>::value>(   \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Octet:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Octet>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt8:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt8>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt16:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt16>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt32:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt32>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt64:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt64>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int8:                                                                     \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int8>::value>(   \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int16:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int16>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int32:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int32>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int64:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int64>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Float:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Float>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Double:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Double>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::LongDouble:                                                               \
        function<                                                                                  \
          ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::LongDouble>::value>(    \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Char:                                                                     \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Char>::value>(   \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::WChar:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::WChar>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::String:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::String>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::WString:                                                                  \
        function<                                                                                  \
          ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::WString>::value>(       \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::None:                                                                     \
        break; /* Do nothing except tell the compiler we know about this type. */                  \
    }                                                                                              \
  } while (false)

#define RBF2_TEMPLATE_CALL_VALUE_TYPES(function, type, ...)                                        \
  do {                                                                                             \
    switch (type) {                                                                                \
      case MessageTypes::Bool:                                                                     \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Bool>::value>(   \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Octet:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Octet>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt8:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt8>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt16:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt16>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt32:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt32>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::UInt64:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::UInt64>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int8:                                                                     \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int8>::value>(   \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int16:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int16>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int32:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int32>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int64:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int64>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Float:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Float>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Double:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Double>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::LongDouble:                                                               \
        function<                                                                                  \
          ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::LongDouble>::value>(    \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Char:                                                                     \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Char>::value>(   \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::WChar:                                                                    \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::WChar>::value>(  \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::String:                                                                   \
        function<::ros2_babel_fish::message_type_traits::value_type<MessageTypes::String>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::WString:                                                                  \
        function<                                                                                  \
          ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::WString>::value>(       \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Array:                                                                    \
      case MessageTypes::Compound:                                                                 \
      case MessageTypes::None:                                                                     \
        break; /* Do nothing except tell the compiler we know about these types. */                \
    }                                                                                              \
  } while (false)

#define RBF2_TEMPLATE_CALL_SIMPLE_VALUE_TYPES(function, type, ...)                                 \
  do {                                                                                             \
    switch (type) {                                                                                \
      case MessageTypes::Bool:                                                                     \
        function<                                                                                  \
          typename ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Bool>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Octet:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::Octet>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::UInt8:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::UInt8>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::UInt16:                                                                   \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::UInt16>::value>(__VA_ARGS__);                                              \
        break;                                                                                     \
      case MessageTypes::UInt32:                                                                   \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::UInt32>::value>(__VA_ARGS__);                                              \
        break;                                                                                     \
      case MessageTypes::UInt64:                                                                   \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::UInt64>::value>(__VA_ARGS__);                                              \
        break;                                                                                     \
      case MessageTypes::Int8:                                                                     \
        function<                                                                                  \
          typename ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int8>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::Int16:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::Int16>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::Int32:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::Int32>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::Int64:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::Int64>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::Float:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::Float>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::Double:                                                                   \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::Double>::value>(__VA_ARGS__);                                              \
        break;                                                                                     \
      case MessageTypes::LongDouble:                                                               \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::LongDouble>::value>(__VA_ARGS__);                                          \
        break;                                                                                     \
      case MessageTypes::Char:                                                                     \
        function<                                                                                  \
          typename ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Char>::value>( \
          __VA_ARGS__);                                                                            \
        break;                                                                                     \
      case MessageTypes::WChar:                                                                    \
        function<typename ::ros2_babel_fish::message_type_traits::value_type<                      \
          MessageTypes::WChar>::value>(__VA_ARGS__);                                               \
        break;                                                                                     \
      case MessageTypes::String:                                                                   \
      case MessageTypes::WString:                                                                  \
      case MessageTypes::Array:                                                                    \
      case MessageTypes::Compound:                                                                 \
      case MessageTypes::None:                                                                     \
        break; /* Do nothing except tell the compiler we know about these types. */                \
    }                                                                                              \
  } while (false)

#define _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(FUNCTION, ...) FUNCTION(__VA_ARGS__)

#define _RBF2_TEMPLATE_CALL_ARRAY_TYPES(FUNCTION, ARRAY, BOUNDED, FIXEDLENGTH, ...)                \
  switch (ARRAY.elementType()) {                                                                   \
    case MessageTypes::Bool:                                                                       \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION,                                                                                  \
        (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                                       \
           typename ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Bool>::value, \
           BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                                   \
      break;                                                                                       \
    case MessageTypes::Octet:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::Octet>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::UInt8:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::UInt8>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::UInt16:                                                                     \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::UInt16>::value,                                               \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::UInt32:                                                                     \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::UInt32>::value,                                               \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::UInt64:                                                                     \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::UInt64>::value,                                               \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Int8:                                                                       \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION,                                                                                  \
        (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                                       \
           typename ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Int8>::value, \
           BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                                   \
      break;                                                                                       \
    case MessageTypes::Int16:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::Int16>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Int32:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::Int32>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Int64:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::Int64>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Float:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::Float>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Double:                                                                     \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::Double>::value,                                               \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::LongDouble:                                                                 \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::LongDouble>::value,                                           \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Char:                                                                       \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION,                                                                                  \
        (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                                       \
           typename ::ros2_babel_fish::message_type_traits::value_type<MessageTypes::Char>::value, \
           BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                                   \
      break;                                                                                       \
    case MessageTypes::WChar:                                                                      \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::WChar>::value,                                                \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::String:                                                                     \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::String>::value,                                               \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::WString:                                                                    \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION, (ARRAY.as<typename ::ros2_babel_fish::ArrayMessage_<                             \
                     typename ::ros2_babel_fish::message_type_traits::value_type<                  \
                       MessageTypes::WString>::value,                                              \
                     BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) __VA_ARGS__);                         \
      break;                                                                                       \
    case MessageTypes::Compound:                                                                   \
      _RBF2_TEMPLATE_CALL_ARRAY_FUNCTION(                                                          \
        FUNCTION,                                                                                  \
        (ARRAY.as<::ros2_babel_fish::CompoundArrayMessage_<BOUNDED, FIXEDLENGTH>>())__VA_OPT__(, ) \
          __VA_ARGS__);                                                                            \
      break;                                                                                       \
    case MessageTypes::Array:                                                                      \
    case MessageTypes::None:                                                                       \
      break; /* Do nothing except tell the compiler we know about these types. */                  \
  }

/*!
 * Call a function with the signature:
 * fn( array, ... )
 * where array can be ::ros2_babel_fish::ArrayMessage_<T, BOUNDED, FIXED_LENGTH>
 *   or ::ros2_babel_fish::CompoundArrayMessage_<BOUNDED, FIXED_LENGTH>
 * and BOUNDED, FIXED_LENGTH are booleans indicating whether the array is bounded or fixed_length
 * (which implies bounded).
 */
#define RBF2_TEMPLATE_CALL_ARRAY_TYPES(FUNCTION, ARRAY, ...)                                       \
  do {                                                                                             \
    static_assert(::std::is_same<                                                                  \
                    ::ros2_babel_fish::ArrayMessageBase,                                           \
                    std::remove_const<std::remove_reference<decltype(ARRAY)>::type>::type>::value, \
                  "Second argument to macro needs to be of type ArrayMessageBase!");               \
    if ((ARRAY).isFixedSize()) {                                                                   \
      _RBF2_TEMPLATE_CALL_ARRAY_TYPES(FUNCTION, (ARRAY), true, true, __VA_ARGS__)                  \
    } else if ((ARRAY).isBounded()) {                                                              \
      _RBF2_TEMPLATE_CALL_ARRAY_TYPES(FUNCTION, (ARRAY), true, false, __VA_ARGS__)                 \
    } else {                                                                                       \
      _RBF2_TEMPLATE_CALL_ARRAY_TYPES(FUNCTION, (ARRAY), false, false, __VA_ARGS__)                \
    }                                                                                              \
  } while (false)

#endif  // ROS2_BABEL_FISH_MACROS_HPP
