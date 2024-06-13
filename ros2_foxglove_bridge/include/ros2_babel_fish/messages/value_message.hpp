// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_VALUE_MESSAGE_HPP
#define ROS2_BABEL_FISH_VALUE_MESSAGE_HPP

#include <rclcpp/time.hpp>

#include "message_type_traits.hpp"
#include "ros2_babel_fish/exceptions/babel_fish_exception.hpp"
#include "ros2_babel_fish/macros.hpp"
#include "ros2_babel_fish/messages/message.hpp"

namespace ros2_babel_fish {

template <typename T>
class ValueMessage final : public Message {
  static constexpr MessageType _type = message_type_traits::message_type<T>::value;
  static_assert(_type != MessageTypes::None, "Invalid type parameter for ValueMessage!");

public:
  RCLCPP_SMART_PTR_DEFINITIONS(ValueMessage<T>)

  explicit ValueMessage(MessageMemberIntrospection member, std::shared_ptr<void> data)
      : Message(_type, std::move(data))
      , member_(std::move(member)) {}

  T getValue() const {
    return *reinterpret_cast<const T*>(this->data_ptr() + member_->offset_);
  }

  void setValue(T value) {
    *reinterpret_cast<T*>(this->data_ptr() + member_->offset_) = value;
  }

  ValueMessage<T>& operator=(const T& value) {
    setValue(value);
    return *this;
  }

  ValueMessage<T>& operator=(const ValueMessage<T>& other) {
    if (this == &other) return *this;
    setValue(other.getValue());
    return *this;
  }

protected:
  template <typename OT>
  void assignValue(const Message& other) {
    Message::operator=(other.value<OT>());
  }

  void _assign(const Message& other) override {
    if (!message_type_traits::isValueType(other.type()))
      throw BabelFishException("Tried to assign non-value message to value message!");
    RBF2_TEMPLATE_CALL_VALUE_TYPES(assignValue, other.type(), other);
  }

  bool _isMessageEqual(const Message& o) const override {
    const auto& other = o.as<ValueMessage<T>>();
    return getValue() == other.getValue();
  }

  MessageMemberIntrospection member_;
};

template <>
inline void ValueMessage<std::string>::setValue(std::string value) {
  if (member_->string_upper_bound_ != 0 && value.length() > member_->string_upper_bound_)
    throw std::length_error("Exceeded string upper bound!");
  *reinterpret_cast<std::string*>(data_ptr() + member_->offset_) = std::move(value);
}

template <>
inline void ValueMessage<std::wstring>::setValue(std::wstring value) {
  if (member_->string_upper_bound_ != 0 && value.length() > member_->string_upper_bound_)
    throw std::length_error("Exceeded string upper bound!");
  *reinterpret_cast<std::wstring*>(data_ptr() + member_->offset_) = std::move(value);
}
}  // namespace ros2_babel_fish

#endif  // ROS2_BABEL_FISH_VALUE_MESSAGE_HPP
