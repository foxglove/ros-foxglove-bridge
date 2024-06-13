// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_MESSAGE_HPP
#define ROS2_BABEL_FISH_MESSAGE_HPP

#include <memory>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>

#include "ros2_babel_fish/exceptions/babel_fish_exception.hpp"
#include "ros2_babel_fish/macros.hpp"

namespace ros2_babel_fish {

/*!
 * Message representation used by BabelFish.
 * Wraps the memory of an actual type erased ROS2 message.
 * Changes will affect the message that can be sent using the ROS runtime.
 */
class Message {
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Message)

  explicit Message(MessageType type, std::shared_ptr<void> data);

  virtual ~Message();

  MessageType type() const {
    return type_;
  }

  /**
   * Convenience method to obtain the content of a ValueMessage as the given type.
   * A type conversion is done if the type doesn't match exactly. If the target type can not fit the
   * source type, a warning is printed.
   *
   * @tparam T The type as which the value is retrieved
   * @return The value casted to the given type T
   *
   * @throws BabelFishException If the message is not a ValueMessage
   * @throws BabelFishException If the type of the ValueMessage can not be casted to a different
   * type which is the case for bool, std::string, ros::Time and ros::Duration
   */
  template <typename T>
  T value() const {
    auto result = std::dynamic_pointer_cast<T>(data_);
    if (!result) throw BabelFishException("Invalid cast!");
    return *result;
  }

  /*!
   * Convenience method that casts the message to the given type.
   * Example:
   * @code
   * Message &msg = getMessage();
   * CompoundMessage &compound = msg.as<CompoundMessage>();
   * @endcode
   * @tparam T Target type
   * @return Message casted to the target type as reference
   *
   * @throws BabelFishException If the message can not be casted to the target type
   */
  template <typename T>
  T& as() {
    T* result = dynamic_cast<T*>(this);
    if (result == nullptr) throw BabelFishException("Tried to cast message to incompatible type!");
    return *result;
  }

  //! @copydoc Message::as()
  template <typename T>
  const T& as() const {
    const T* result = dynamic_cast<const T*>(this);
    if (result == nullptr) throw BabelFishException("Tried to cast message to incompatible type!");
    return *result;
  }

  //! @return True if this message is a builtin_interfaces/Time message and can be cast to
  //! rclcpp::Time using the value method.
  bool isTime() const;

  //! @return True if this message is a builtin_interfaces/Duration message and can be cast to
  //! rclcpp::Duration using the value method.
  bool isDuration() const;

  /**
   * Convenience method to access the child with the given key  of a CompoundMessage.
   * @param key The name or path of the child
   * @return The child message accessed by the given key
   *
   * @throws BabelFishException If child access by key is not supported.
   */
  virtual Message& operator[](const std::string& key);

  //!@copydoc Message::operator[](const std::string&)
  virtual const Message& operator[](const std::string& key) const;

  /**
   * @defgroup Convenience methods for ValueMessage
   * @brief Will try to set the value of ValueMessage to the given value.
   * Incompatible types are checked at runtime. If the type of ValueMessage can not fit the passed
   * value prints an error.
   *
   * @throws BabelFishException If bool is assigned to non-boolean ValueMessage or non-boolean value
   * to bool ValueMessage
   * @throws BabelFishException If ros::Time / ros::Duration value is set to a different type of
   * ValueMessage.
   * @{
   */
  Message& operator=(bool value);

  Message& operator=(uint8_t value);

  Message& operator=(uint16_t value);

  Message& operator=(uint32_t value);

  Message& operator=(uint64_t value);

  Message& operator=(int8_t value);

  Message& operator=(int16_t value);

  Message& operator=(int32_t value);

  Message& operator=(int64_t value);

  Message& operator=(float value);

  Message& operator=(double value);

  Message& operator=(long double value);

  Message& operator=(const char* value);

  Message& operator=(const std::string& value);

  Message& operator=(const std::wstring& value);

  Message& operator=(const builtin_interfaces::msg::Time& value);

  Message& operator=(const builtin_interfaces::msg::Duration& value);

  Message& operator=(const rclcpp::Time& value);

  Message& operator=(const rclcpp::Duration& value);

  Message& operator=(const Message& other);

  /**@}*/

  template <typename T>
  bool operator==(const T& other) const;

  bool operator==(const Message& other) const;

  bool operator==(const char* c) const;

  bool operator==(const wchar_t* c) const;

  template <typename T>
  bool operator!=(const T& other) const;

protected:
  void move(std::shared_ptr<void> new_data) {
    data_ = std::move(new_data);
    onMoved();
  }

  virtual void onMoved() {}

  template <typename T, typename U>
  void checkValueEqual(const U& other, bool& result) const;

  // Disable copy construction except for subclasses
  Message(const Message& other)
      : data_(other.data_)
      , type_(other.type_) {}

  virtual bool _isMessageEqual(const Message& other) const = 0;

  virtual void _assign(const Message& other) = 0;

  unsigned char* data_ptr() {
    return reinterpret_cast<unsigned char*>(data_.get());
  }

  const unsigned char* data_ptr() const {
    return reinterpret_cast<const unsigned char*>(data_.get());
  }

  std::shared_ptr<void> data_;
  MessageType type_;

  friend class CompoundMessage;

  template <bool, bool>
  friend class CompoundArrayMessage_;
};

template <>
bool Message::value() const;

template <>
uint8_t Message::value() const;

template <>
uint16_t Message::value() const;

template <>
uint32_t Message::value() const;

template <>
uint64_t Message::value() const;

template <>
int8_t Message::value() const;

template <>
int16_t Message::value() const;

template <>
int32_t Message::value() const;

template <>
int64_t Message::value() const;

template <>
float Message::value() const;

template <>
double Message::value() const;

template <>
long double Message::value() const;

template <>
std::string Message::value() const;

template <>
char16_t Message::value() const;

template <>
std::wstring Message::value() const;

// These are now compound messages instead of value messages
template <>
rclcpp::Time Message::value() const;

template <>
rclcpp::Duration Message::value() const;

// Equality comparison needs to come after value specializations

namespace impl {
template <typename X>
struct PassThrough {
  typedef X type;
};

// Make sure we only compile possible comparisons.
// Looks more complicated than it is but the base implementation just makes sure we don't compare
// signed and unsigned
template <typename T>
struct EqHelper {
  template <typename U>
  using is_same_sign =
    std::integral_constant<bool, std::is_signed<T>::value == std::is_signed<U>::value>;

  template <typename U>
  static typename std::enable_if<std::is_arithmetic<U>::value && is_same_sign<U>::value, void>::type
  equal(const Message* m, const U& other, bool& result) {
    result = m->template value<T>() == other;
  }

  template <typename U>
  static typename std::enable_if<
    std::is_arithmetic<U>::value && !is_same_sign<U>::value && std::is_signed<U>::value, void>::type
  equal(const Message* m, const U& other, bool& result) {
    // Different sign and U signed so T is unsigned
    if (other < 0) {
      result = false;  // T can't be negative so no need to check
      return;
    }
    // We always cast unsigned to signed since the other way is not always possible (e.g.
    // float/double).
    using SignedT = typename std::common_type<U, typename std::make_signed<T>::type>::type;
    const T& val = m->template value<T>();
    // val has to be able to fit into the SignedT type, this may have inaccuracies with some
    // floating point types/values.
    result = val <= static_cast<T>(std::numeric_limits<SignedT>::max()) &&
             static_cast<SignedT>(val) == other;
  }

  template <typename U>
  static typename std::enable_if<std::is_arithmetic<U>::value && !is_same_sign<U>::value &&
                                   !std::is_signed<U>::value,
                                 void>::type
  equal(const Message* m, const U& other, bool& result) {
    // Different sign and U unsigned so T is signed
    using SignedU = typename std::make_signed<U>::type;
    if (other > static_cast<U>(std::numeric_limits<SignedU>::max()))
      result = false;
    else
      result = m->template value<T>() == static_cast<SignedU>(other);
  }

  template <typename U>
  static typename std::enable_if<!std::is_arithmetic<U>::value, void>::type equal(const Message*,
                                                                                  const U&,
                                                                                  bool& result) {
    result = false;
  }
};

template <>
struct EqHelper<bool> {
  template <typename U>
  static typename std::enable_if<std::is_convertible<U, bool>::value, void>::type equal(
    const Message* m, const U& other, bool& result) {
    result = m->template value<bool>() == other;
  }

  template <typename U>
  static typename std::enable_if<!std::is_convertible<U, bool>::value, void>::type equal(
    const Message*, const U&, bool& result) {
    result = false;
  }
};

template <>
struct EqHelper<std::string> {
  template <typename U>
  static typename std::enable_if<std::is_convertible<U, std::string>::value, void>::type equal(
    const Message* m, const U& other, bool& result) {
    result = m->template value<std::string>() == other;
  }

  template <typename U>
  static typename std::enable_if<!std::is_convertible<U, std::string>::value, void>::type equal(
    const Message*, const U&, bool& result) {
    result = false;
  }
};

template <>
struct EqHelper<std::wstring> {
  template <typename U>
  static typename std::enable_if<std::is_convertible<U, std::wstring>::value, void>::type equal(
    const Message* m, const U& other, bool& result) {
    result = m->template value<std::wstring>() == other;
  }

  template <typename U>
  static typename std::enable_if<!std::is_convertible<U, std::wstring>::value, void>::type equal(
    const Message*, const U&, bool& result) {
    result = false;
  }
};

template <>
struct EqHelper<CompoundMessage> {
  template <typename U>
  static typename std::enable_if<std::is_base_of<Message, U>::value, void>::type equal(
    const Message* m, const U& other, bool& result) {
    result = *m == other;
  }

  template <typename U>
  static typename std::enable_if<!std::is_base_of<Message, U>::value, void>::type equal(
    const Message* m, const U& other, bool& result) {
    result = m->template value<U>() == other;
  }
};

template <>
struct EqHelper<ArrayMessageBase> {
  template <typename U>
  static typename std::enable_if<std::is_base_of<Message, U>::value, void>::type equal(
    const Message* m, const U& other, bool& result) {
    result = *m == other;
  }

  template <typename U>
  static typename std::enable_if<!std::is_base_of<Message, U>::value, void>::type equal(
    const Message*, const U&, bool& result) {
    result = false;
  }
};
}  // namespace impl

template <typename T, typename U>
void Message::checkValueEqual(const U& other, bool& result) const {
  impl::EqHelper<T>::equal(this, other, result);
}

template <typename T>
bool Message::operator==(const T& other) const {
  bool result = false;
  RBF2_TEMPLATE_CALL(checkValueEqual, type_, other, result);
  return result;
}

template <typename T>
bool Message::operator!=(const T& other) const {
  return !(*this == other);
}
}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_MESSAGE_HPP
