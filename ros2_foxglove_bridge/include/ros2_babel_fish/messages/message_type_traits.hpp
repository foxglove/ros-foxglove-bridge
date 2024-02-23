//
// Created by stefan on 23.01.21.
//

#ifndef ROS2_BABEL_FISH_MESSAGE_TYPE_TRAITS_HPP
#define ROS2_BABEL_FISH_MESSAGE_TYPE_TRAITS_HPP

#include "ros2_babel_fish/messages/message_types.hpp"
#include <string>
#include <vector>

namespace ros2_babel_fish
{
// Predeclare message classes
class Message;

class ArrayMessageBase;

class CompoundMessage;

namespace message_type_traits
{
template<typename T>
struct message_type {
  static constexpr MessageType value = MessageTypes::None;
};

template<MessageType>
struct member_type {
  typedef void value;
};

template<MessageType>
struct value_type {
  typedef void value;
};

inline bool isValueType( MessageType type )
{
  return type != MessageTypes::Compound && type != MessageTypes::Array && type != MessageTypes::None;
}

#define DECLARE_MESSAGE_TYPE_FOR_TYPE( __message_type, __type )                                    \
  template<>                                                                                       \
  struct message_type<__type> {                                                                    \
    static constexpr MessageType value = __message_type;                                           \
  }
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Bool, bool );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt8, uint8_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt16, uint16_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt32, uint32_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::UInt64, uint64_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int8, int8_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int16, int16_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int32, int32_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Int64, int64_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Float, float );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::Double, double );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::LongDouble, long double );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::WChar, char16_t );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::String, std::string );
DECLARE_MESSAGE_TYPE_FOR_TYPE( MessageTypes::WString, std::wstring );
#undef DECLARE_MESSAGE_TYPE_FOR_TYPE

#define DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( __message_type, __type )                             \
  template<>                                                                                       \
  struct member_type<__message_type> {                                                             \
    typedef __type value;                                                                          \
  }
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Bool, bool );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Octet, unsigned char );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt8, uint8_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt16, uint16_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt32, uint32_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt64, uint64_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int8, int8_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int16, int16_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int32, int32_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int64, int64_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Float, float );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Double, double );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::LongDouble, long double );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Char, unsigned char );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::WChar, char16_t );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::String, std::string );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::WString, std::wstring );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Compound, CompoundMessage );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Array, ArrayMessageBase );
DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE( MessageTypes::None, void );
#undef DECLARE_MEMBER_TYPE_FOR_MESSAGE_TYPE
#define DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( __message_type, __type )                              \
  template<>                                                                                       \
  struct value_type<__message_type> {                                                              \
    typedef __type value;                                                                          \
  }
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Bool, bool );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Octet, unsigned char );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt8, uint8_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt16, uint16_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt32, uint32_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::UInt64, uint64_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int8, int8_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int16, int16_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int32, int32_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Int64, int64_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Float, float );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Double, double );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::LongDouble, long double );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Char, unsigned char );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::WChar, char16_t );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::String, std::string );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::WString, std::wstring );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Compound, CompoundMessage );
DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE( MessageTypes::Array, ArrayMessageBase );
#undef DECLARE_VALUE_TYPE_FOR_MESSAGE_TYPE

template<typename T, bool FIXED_LENGTH = false>
struct array_type {
  typedef T &Reference;
  typedef T ReturnType;
  typedef const T ConstReturnType;
  typedef const T &ArgumentType;
};
template<>
struct array_type<bool, false> {
  typedef std::vector<bool>::reference Reference;
  typedef bool ReturnType;
  typedef bool ConstReturnType;
  typedef bool ArgumentType;
};
template<>
struct array_type<Message, false> {
  typedef void Reference;
  typedef void ReturnType;
  typedef const void ConstReturnType;
  typedef void ArgumentType;
};
template<>
struct array_type<Message, true> {
  typedef void Reference;
  typedef void ReturnType;
  typedef const void ConstReturnType;
  typedef void ArgumentType;
};
} // namespace message_type_traits
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_MESSAGE_TYPE_TRAITS_HPP
