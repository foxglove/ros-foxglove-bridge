// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/messages/message.hpp"

#include <limits>

#include "../logging.hpp"
#include "ros2_babel_fish/babel_fish.hpp"
#include "ros2_babel_fish/macros.hpp"
#include "ros2_babel_fish/messages/array_message.hpp"
#include "ros2_babel_fish/messages/compound_message.hpp"
#include "ros2_babel_fish/messages/message_type_traits.hpp"
#include "ros2_babel_fish/messages/value_message.hpp"
#include "ros2_babel_fish/method_invoke_helpers.hpp"

namespace ros2_babel_fish {

Message::Message(MessageType type, std::shared_ptr<void> data)
    : data_(std::move(data))
    , type_(type) {}

Message& Message::operator[](const std::string&) {
  throw BabelFishException(
    "Tried to access child message on message object that does not support "
    "child access by key.");
}

const Message& Message::operator[](const std::string&) const {
  throw BabelFishException(
    "Tried to access child message on message object that does not support "
    "child access by key.");
}

Message::~Message() = default;

namespace {

template <typename T, typename U>
constexpr
  typename std::enable_if<std::numeric_limits<T>::is_signed != std::numeric_limits<U>::is_signed,
                          bool>::type
  inBounds() {
  return false;
}

template <typename T, typename U>
constexpr
  typename std::enable_if<std::numeric_limits<T>::is_signed == std::numeric_limits<U>::is_signed,
                          bool>::type
  inBounds() {
  return std::numeric_limits<U>::min() <= std::numeric_limits<T>::min() &&
         std::numeric_limits<T>::max() <= std::numeric_limits<U>::max();
}

template <typename T, typename U>
constexpr typename std::enable_if<!std::is_integral<U>::value && !std::is_floating_point<U>::value,
                                  bool>::type
isCompatible() {
  return false;
}

template <typename T, typename U>
constexpr typename std::enable_if<std::is_same<T, bool>::value, bool>::type isCompatible() {
  return std::is_same<T, U>::value;
}

template <typename T, typename U>
constexpr
  typename std::enable_if<std::is_integral<U>::value && !std::is_same<T, bool>::value, bool>::type
  isCompatible() {
  if (std::is_same<T, U>::value) return true;
  if (std::is_floating_point<T>::value) return false;
  if (std::numeric_limits<T>::is_signed && !std::numeric_limits<U>::is_signed) return false;
  return inBounds<T, U>();
}

template <typename T, typename U>
constexpr typename std::enable_if<std::is_floating_point<U>::value && !std::is_same<T, bool>::value,
                                  bool>::type
isCompatible() {
  return std::is_same<T, U>::value || !std::is_same<T, double>::value;
}

template <>
constexpr bool isCompatible<float, double>() {
  return true;
}

template <typename T, typename U>
constexpr typename std::enable_if<std::is_floating_point<T>::value, bool>::type inBounds(const T&) {
  return false;
}

// is_integral is necessary to avoid a CLang tidy warning
template <typename T, typename U>
typename std::enable_if<std::is_integral<T>::value && std::numeric_limits<T>::is_signed &&
                          !std::numeric_limits<U>::is_signed,
                        bool>::type
inBounds(const T& val) {
  return val >= 0 &&
         static_cast<typename std::make_unsigned<T>::type>(val) <= std::numeric_limits<U>::max();
}

template <typename T, typename U>
typename std::enable_if<std::is_integral<T>::value && !std::numeric_limits<T>::is_signed &&
                          std::numeric_limits<U>::is_signed,
                        bool>::type
inBounds(const T& val) {
  return val <= static_cast<typename std::make_unsigned<U>::type>(std::numeric_limits<U>::max());
}

template <typename T, typename U>
typename std::enable_if<std::is_integral<T>::value &&
                          std::numeric_limits<T>::is_signed == std::numeric_limits<U>::is_signed,
                        bool>::type
inBounds(const T& val) {
  return std::numeric_limits<U>::min() <= val && val <= std::numeric_limits<U>::max();
}

template <typename T, typename U>
typename std::enable_if<!std::is_integral<U>::value && !std::is_floating_point<U>::value,
                        bool>::type
isCompatible(const T&) {
  return false;
}

//! Booleans are only compatible with booleans as they are semantically distinct from numbers.
template <typename T, typename U>
typename std::enable_if<std::is_same<bool, T>::value, bool>::type isCompatible(const T&) {
  return std::is_same<T, U>::value;
}

template <typename T, typename U>
typename std::enable_if<std::is_integral<U>::value && !std::is_same<bool, T>::value, bool>::type
isCompatible(const T& val) {
  if (std::is_same<T, U>::value) return true;
  if (std::is_floating_point<T>::value) return false;
  return inBounds<T, U>(val);
}

template <typename T, typename U>
typename std::enable_if<std::is_floating_point<U>::value && !std::is_same<bool, T>::value,
                        bool>::type
isCompatible(const T&) {
  return std::is_integral<T>::value || sizeof(T) <= sizeof(U);
}

template <typename MessageType>
struct ValueAssigner {
  template <typename ValueType>
  static void assignValue(Message& m, const ValueType& value) {
    using namespace std::chrono_literals;
    using namespace message_type_traits;
    if (m.type() != message_type<ValueType>::value && !isCompatible<ValueType, MessageType>(value))
      throw BabelFishException(
        "Value does not fit into value message! Make sure you're using the correct type or at "
        "least stay within the range of values for the message type!");
#if RBF_WARN_ON_INCOMPATIBLE_TYPE
    if (m.type() != message_type<ValueType>::value && !isCompatible<ValueType, MessageType>()) {
      rclcpp::Clock clock;
      RBF2_WARN_THROTTLE(
        clock, 5000,
        "Assigned value fits but the type of the assignment can not be converted without loss of "
        "information in some cases! This message is throttled to once per 5 seconds!");
    }
#endif
    m.as<ValueMessage<MessageType>>().setValue(static_cast<MessageType>(value));
  }
};

template <>
struct ValueAssigner<bool> {
  template <typename ValueType>
  static void assignValue(Message&, const ValueType&) {
    throw BabelFishException("Can not assign non-boolean value to a boolean ValueMessage!");
  }
};

template <>
struct ValueAssigner<std::wstring> {
  template <typename ValueType>
  static void assignValue(Message&, const ValueType&) {
    throw BabelFishException("Can not assign non-wstring value to a wstring ValueMessage!");
  }
};

template <>
struct ValueAssigner<std::string> {
  template <typename ValueType>
  static void assignValue(Message&, const ValueType&) {
    throw BabelFishException("Can not assign non-string value to a string ValueMessage!");
  }
};

template <>
struct ValueAssigner<CompoundMessage> {
  template <typename ValueType>
  static void assignValue(Message&, const ValueType&) {
    throw BabelFishException("Can not assign value to a CompoundMessage!");
  }
};

template <>
struct ValueAssigner<ArrayMessageBase> {
  template <typename ValueType>
  static void assignValue(Message&, const ValueType&) {
    throw BabelFishException("Can not assign value to an array message!");
  }
};

// This is a workaround around the issue that GCC doesn't allow template specializations in a
// non-namespace scope and no template specializations for an unspecialized template
template <typename ValueType>
struct AssignerValue {
  template <typename MessageType>
  void operator()(ValueMessage<MessageType>& m, const ValueType& value) {
    ValueAssigner<MessageType>::template assignValue<ValueType>(m, value);
  }

  void operator()(CompoundMessage& m, const ValueType& value) {
    ValueAssigner<CompoundMessage>::template assignValue<ValueType>(m, value);
  }

  void operator()(ArrayMessageBase& m, const ValueType& value) {
    ValueAssigner<ArrayMessageBase>::template assignValue<ValueType>(m, value);
  }
};

template <typename T>
void assignToValueMessage(Message& m, const T& value) {
  invoke_for_message(m, AssignerValue<T>{}, value);
}
}  // namespace

Message& Message::operator=(bool value) {
  if (type() != MessageTypes::Bool)
    throw BabelFishException("Can not _assign a boolean to a non-boolean ValueMessage!");
  as<ValueMessage<bool>>() = value;
  return *this;
}

Message& Message::operator=(uint8_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(uint16_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(uint32_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(uint64_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(int8_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(int16_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(int32_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(int64_t value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(float value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(double value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(long double value) {
  assignToValueMessage(*this, value);
  return *this;
}

Message& Message::operator=(const char* value) {
  *this = std::string(value);
  return *this;
}

Message& Message::operator=(const std::string& value) {
  if (type() != MessageTypes::String)
    throw BabelFishException("Can not _assign a string to a non-string ValueMessage!");
  as<ValueMessage<std::string>>() = value;
  return *this;
}

Message& Message::operator=(const std::wstring& value) {
  if (type() != MessageTypes::WString)
    throw BabelFishException("Can not _assign a wstring to a non-wstring ValueMessage!");
  as<ValueMessage<std::wstring>>() = value;
  return *this;
}

Message& Message::operator=(const builtin_interfaces::msg::Time& value) {
  if (type() != MessageTypes::Compound)
    throw BabelFishException(
      "Tried to _assign rclcpp::Time to message that is not a compound message!");
  as<CompoundMessage>() = value;
  return *this;
}

Message& Message::operator=(const builtin_interfaces::msg::Duration& value) {
  if (type() != MessageTypes::Compound)
    throw BabelFishException(
      "Tried to _assign rclcpp::Duration to message that is not a compound message!");
  as<CompoundMessage>() = value;
  return *this;
}

Message& Message::operator=(const rclcpp::Time& value) {
  *this = (builtin_interfaces::msg::Time)value;
  return *this;
}

Message& Message::operator=(const rclcpp::Duration& value) {
  *this = (builtin_interfaces::msg::Duration)value;
  return *this;
}

Message& Message::operator=(const Message& other) {
  if (this == &other) return *this;
  _assign(other);
  return *this;
}

bool Message::isTime() const {
  return type_ == MessageTypes::Compound &&
         as<CompoundMessage>().datatype() == "builtin_interfaces::msg::Time";
}

bool Message::isDuration() const {
  return type_ == MessageTypes::Compound &&
         as<CompoundMessage>().datatype() == "builtin_interfaces::msg::Duration";
}

namespace {
template <typename U>
struct ValueExtractor {
  template <typename T>
  U operator()(const ValueMessage<T>& m) {
    using namespace message_type_traits;
    T val = m.getValue();
    if (m.type() != message_type<U>::value && !isCompatible<T, U>(val))
      throw BabelFishException("Value does not fit into casted type!");
#if RBF_WARN_ON_INCOMPATIBLE_TYPE
    if (m.type() != message_type<T>::value && !isCompatible<T, U>()) {
      rclcpp::Clock clock;
      RBF2_WARN_THROTTLE(
        clock, 5000,
        "Value fits into casted type but it is smaller than the message type which may lead to "
        "catastrophic failure in the future! This message is printed only once!");
    }
#endif
    return static_cast<U>(val);
  }

  U operator()(const ValueMessage<std::string>&) {
    throw BabelFishException("Tried to retrieve non-string ValueMessage as string!");
  }

  U operator()(const ValueMessage<std::wstring>&) {
    throw BabelFishException("Tried to retrieve non-wstring ValueMessage as wstring!");
  }
};

template <typename T>
T obtainValueAsType(const Message* m) {
  using namespace message_type_traits;
  if (m->type() == MessageTypes::Bool)
    throw BabelFishException("Can not return value of boolean ValueMessage as non-boolean!");
  return invoke_for_value_message(*m, ValueExtractor<T>{});
}
}  // namespace

template <>
bool Message::value() const {
  if (type() != MessageTypes::Bool)
    throw BabelFishException("Can not return value of non-boolean ValueMessage as boolean!");
  return as<ValueMessage<bool>>().getValue();
}

template <>
uint8_t Message::value() const {
  return obtainValueAsType<uint8_t>(this);
}

template <>
char16_t Message::value() const {
  return obtainValueAsType<char16_t>(this);
}

template <>
uint16_t Message::value() const {
  return obtainValueAsType<uint16_t>(this);
}

template <>
uint32_t Message::value() const {
  return obtainValueAsType<uint32_t>(this);
}

template <>
uint64_t Message::value() const {
  return obtainValueAsType<uint64_t>(this);
}

template <>
int8_t Message::value() const {
  return obtainValueAsType<int8_t>(this);
}

template <>
int16_t Message::value() const {
  return obtainValueAsType<int16_t>(this);
}

template <>
int32_t Message::value() const {
  return obtainValueAsType<int32_t>(this);
}

template <>
int64_t Message::value() const {
  return obtainValueAsType<int64_t>(this);
}

template <>
float Message::value() const {
  return obtainValueAsType<float>(this);
}

template <>
double Message::value() const {
  return obtainValueAsType<double>(this);
}

template <>
long double Message::value() const {
  return obtainValueAsType<long double>(this);
}

template <>
std::string Message::value() const {
  if (type() == MessageTypes::WString)
    throw BabelFishException("Can not return value of wstring ValueMessage as string!");
  if (type() != MessageTypes::String)
    throw BabelFishException("Can not return value of non-string ValueMessage as string!");
  return as<ValueMessage<std::string>>().getValue();
}

template <>
std::wstring Message::value() const {
  if (type() == MessageTypes::String)
    throw BabelFishException("Can not return value of string ValueMessage as wstring!");
  if (type() != MessageTypes::WString)
    throw BabelFishException("Can not return value of non-string ValueMessage as string!");
  return as<ValueMessage<std::wstring>>().getValue();
}

template <>
rclcpp::Time Message::value() const {
  if (type() != MessageTypes::Compound)
    throw BabelFishException(
      "Tried to obtain rclcpp::Time from message that is not a compound message!");
  const auto& compound = as<CompoundMessage>();
  if (compound.datatype() != "builtin_interfaces::msg::Time")
    throw BabelFishException("Tried to obtain rclcpp::Time from '" + compound.name() +
                             "' message which is not a 'builtin_interfaces/msg/Time' message!");
  return {
    *std::static_pointer_cast<const builtin_interfaces::msg::Time>(compound.type_erased_message())};
}

template <>
rclcpp::Duration Message::value() const {
  if (type() != MessageTypes::Compound)
    throw BabelFishException(
      "Tried to obtain rclcpp::Duration from message that is not a compound message!");
  const auto& compound = as<CompoundMessage>();
  if (compound.datatype() != "builtin_interfaces::msg::Duration")
    throw BabelFishException("Tried to obtain rclcpp::Duration from '" + compound.name() +
                             "' message which is not a 'builtin_interfaces/msg/Duration' message!");
  return {*std::static_pointer_cast<const builtin_interfaces::msg::Duration>(
    compound.type_erased_message())};
}

bool Message::operator==(const Message& other) const {
  if (this == &other) return true;
  if (type_ != other.type_) return false;
  return _isMessageEqual(other);
}

// Need to come after value specializations
bool Message::operator==(const char* c) const {
  return value<std::string>() == c;
}

bool Message::operator==(const wchar_t* c) const {
  return value<std::wstring>() == c;
}
}  // namespace ros2_babel_fish
