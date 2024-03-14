// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/messages/compound_message.hpp"

#include <regex>

#include "ros2_babel_fish/exceptions/babel_fish_exception.hpp"
#include "ros2_babel_fish/idl/serialization.hpp"
#include "ros2_babel_fish/macros.hpp"
#include "ros2_babel_fish/messages/array_message.hpp"
#include "ros2_babel_fish/messages/value_message.hpp"

namespace ros2_babel_fish {

CompoundMessage::CompoundMessage()
    : Message(MessageTypes::Compound, nullptr)
    , members_(nullptr, nullptr) {}

CompoundMessage::CompoundMessage(MessageMembersIntrospection members, std::shared_ptr<void> data)
    : Message(MessageTypes::Compound, std::move(data))
    , members_(std::move(members))
    , values_(members_.value->member_count_) {}

CompoundMessage::CompoundMessage(MessageMembersIntrospection members,
                                 rosidl_runtime_cpp::MessageInitialization init)
    : CompoundMessage(members, createContainer(*members.value, init)) {}

CompoundMessage::CompoundMessage(const CompoundMessage& other)
    : CompoundMessage(other.members_, other.data_) {}

std::string CompoundMessage::datatype() const {
  return std::string(members_.value->message_namespace_) + "::" + members_.value->message_name_;
}

std::string CompoundMessage::name() const {
  static const std::regex namespace_regex("::");
  return std::regex_replace(members_.value->message_namespace_, namespace_regex, "/") + "/" +
         members_.value->message_name_;
}

bool CompoundMessage::containsKey(const std::string& key) const {
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) {
    if (members_.value->members_[i].name_ == key) return true;
  }
  return false;
}

std::vector<std::string> CompoundMessage::keys() const {
  std::vector<std::string> result;
  result.reserve(members_.value->member_count_);
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) {
    result.emplace_back(members_.value->members_[i].name_);
  }
  return result;
}

std::string CompoundMessage::keyAt(size_t index) {
  return index >= members_.value->member_count_ ? "" : members_.value->members_[index].name_;
}

CompoundMessage& CompoundMessage::operator=(const Message& other) {
  _assign(other);
  return *this;
}

CompoundMessage& CompoundMessage::operator=(const builtin_interfaces::msg::Time& value) {
  if (datatype() != "builtin_interfaces::msg::Time")
    throw BabelFishException("Tried to _assign rclcpp::Time to '" + name() +
                             "' message which is not a 'builtin_interfaces/msg/Time' message!");
  (*this)["sec"] = value.sec;
  (*this)["nanosec"] = value.nanosec;
  return *this;
}

CompoundMessage& CompoundMessage::operator=(const builtin_interfaces::msg::Duration& value) {
  if (datatype() != "builtin_interfaces::msg::Duration")
    throw BabelFishException("Tried to _assign rclcpp::Duration to '" + name() +
                             "' message which is not a 'builtin_interfaces/msg/Duration' message!");
  (*this)["sec"] = value.sec;
  (*this)["nanosec"] = value.nanosec;
  return *this;
}

CompoundMessage& CompoundMessage::operator=(const rclcpp::Time& value) {
  *this = (builtin_interfaces::msg::Time)value;
  return *this;
}

CompoundMessage& CompoundMessage::operator=(const rclcpp::Duration& value) {
  *this = (builtin_interfaces::msg::Duration)value;
  return *this;
}

CompoundMessage& CompoundMessage::operator=(const CompoundMessage& other) {
  if (this == &other) return *this;
  if (members_.value != other.members_.value) {
    if (other.members_.value->message_namespace_ != members_.value->message_namespace_ ||
        other.members_.value->message_name_ != members_.value->message_name_)
      throw BabelFishException("Tried to _assign compound of name '" + other.name() +
                               "' to compound of name '" + name() + "'!");
  }
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) *(*this)[i] = *other[i];
  return *this;
}

CompoundMessage& CompoundMessage::operator=(CompoundMessage&& other) noexcept {
  data_ = std::move(other.data_);
  type_ = other.type_;
  members_ = std::move(other.members_);
  values_ = std::move(other.values_);
  initialized_values_ = other.initialized_values_;
  return *this;
}

void CompoundMessage::_assign(const Message& other) {
  if (other.type() != MessageTypes::Compound)
    throw BabelFishException("Tried to assign non-compound to compound message!");
  *this = static_cast<const CompoundMessage&>(other);
}

bool CompoundMessage::_isMessageEqual(const Message& o) const {
  const auto& other = o.as<CompoundMessage>();
  if (other.members_.value != members_.value) return false;
  initValues();
  other.initValues();
  for (size_t i = 0; i < members_->member_count_; ++i) {
    if (values_[i] != other.values_[i]) return false;
  }
  return true;
}

namespace {

template <typename T>
void initValueMessage(Message::SharedPtr& message, const MessageMemberIntrospection& member,
                      const std::shared_ptr<void>& data) {
  message = ValueMessage<T>::template make_shared(member, data);
}

template <bool FIXED_LENGTH = false, bool BOUNDED = FIXED_LENGTH>
struct ArrayInit {
  template <typename T>
  static void initArrayMessage(Message::SharedPtr& message,
                               const MessageMemberIntrospection& member,
                               const std::shared_ptr<void>& data) {
    message = ArrayMessage_<T, BOUNDED, FIXED_LENGTH>::template make_shared(member, data);
  }
};

template <>
template <>
void ArrayInit<false, false>::initArrayMessage<ArrayMessageBase>(Message::SharedPtr&,
                                                                 const MessageMemberIntrospection&,
                                                                 const std::shared_ptr<void>&) {
  throw std::runtime_error(
    "Arrays of arrays are not supported by ROS2 (yet)! Please open an issue if this changed!");
}

template <>
template <>
void ArrayInit<false, true>::initArrayMessage<ArrayMessageBase>(Message::SharedPtr&,
                                                                const MessageMemberIntrospection&,
                                                                const std::shared_ptr<void>&) {
  throw std::runtime_error(
    "Arrays of arrays are not supported by ROS2 (yet)! Please open an issue if this changed!");
}

template <>
template <>
void ArrayInit<true, true>::initArrayMessage<ArrayMessageBase>(Message::SharedPtr&,
                                                               const MessageMemberIntrospection&,
                                                               const std::shared_ptr<void>&) {
  throw std::runtime_error(
    "Arrays of arrays are not supported by ROS2 (yet)! Please open an issue if this changed!");
}

template <>
template <>
void ArrayInit<false, false>::initArrayMessage<CompoundMessage>(
  Message::SharedPtr& message, const MessageMemberIntrospection& member,
  const std::shared_ptr<void>& data) {
  message = CompoundArrayMessage::make_shared(member, data);
}

template <>
template <>
void ArrayInit<false, true>::initArrayMessage<CompoundMessage>(
  Message::SharedPtr& message, const MessageMemberIntrospection& member,
  const std::shared_ptr<void>& data) {
  message = BoundedCompoundArrayMessage::make_shared(member, data);
}

template <>
template <>
void ArrayInit<true, true>::initArrayMessage<CompoundMessage>(
  Message::SharedPtr& message, const MessageMemberIntrospection& member,
  const std::shared_ptr<void>& data) {
  message = FixedLengthCompoundArrayMessage::make_shared(member, data);
}

void initValue(Message::SharedPtr& message, const MessageMemberIntrospection& member,
               const std::shared_ptr<void>& data) {
  if (member->is_array_) {
    // Empty deleter with copy to data to make sure the subarea of memory the shared_ptr is pointing
    // to isn't destroyed while this shared_ptr exists.
    std::shared_ptr<void> sub_data(static_cast<uint8_t*>(data.get()) + member->offset_,
                                   [data](void*) {
                                     (void)data;
                                   });
    if (member->is_upper_bound_) {
      using BoundedArrayInit = ArrayInit<false, true>;
      RBF2_TEMPLATE_CALL(BoundedArrayInit::initArrayMessage, member->type_id_, message, member,
                         sub_data);
    } else if (member->array_size_ == 0) {
      using NormalArrayInit = ArrayInit<false, false>;
      RBF2_TEMPLATE_CALL(NormalArrayInit::initArrayMessage, member->type_id_, message, member,
                         sub_data);
    } else {
      using FixedLengthArrayInit = ArrayInit<true, true>;
      RBF2_TEMPLATE_CALL(FixedLengthArrayInit::initArrayMessage, member->type_id_, message, member,
                         sub_data);
    }
  } else if (member->type_id_ == MessageTypes::Compound) {
    // Empty deleter with copy to data to make sure the subarea of memory the shared_ptr is pointing
    // to isn't destroyed while this shared_ptr exists.
    std::shared_ptr<void> sub_data(static_cast<uint8_t*>(data.get()) + member->offset_,
                                   [data](void*) {
                                     (void)data;
                                   });
    message = CompoundMessage::template make_shared(member, std::move(sub_data));
  } else {
    RBF2_TEMPLATE_CALL_VALUE_TYPES(initValueMessage, member->type_id_, message, member, data);
  }
}
}  // namespace

Message& CompoundMessage::operator[](const std::string& key) {
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) {
    if (members_.value->members_[i].name_ == key) {
      if (values_[i] == nullptr) {
        initValue(values_[i], members_.getMember(i), data_);
      }
      return *values_[i];
    }
  }
  throw std::out_of_range("Invalid key! '" + key + "' is not in " +
                          members_.value->message_namespace_ +
                          "::" + members_.value->message_name_ + ".");
}

const Message& CompoundMessage::operator[](const std::string& key) const {
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) {
    if (members_.value->members_[i].name_ == key) {
      if (values_[i] == nullptr) {
        initValue(values_[i], members_.getMember(i), data_);
      }
      return *values_[i];
    }
  }
  throw std::out_of_range("Invalid key! '" + key + "' is not in " +
                          members_.value->message_namespace_ +
                          "::" + members_.value->message_name_ + ".");
}

std::vector<Message::SharedPtr> CompoundMessage::values() {
  initValues();
  return values_;
}

std::vector<Message::ConstSharedPtr> CompoundMessage::values() const {
  initValues();
  return {values_.begin(), values_.end()};
}

void CompoundMessage::initValues() const {
  if (initialized_values_) return;
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) {
    if (values_[i] != nullptr) continue;
    initValue(values_[i], members_.getMember(i), data_);
  }
  initialized_values_ = true;
}

Message::SharedPtr CompoundMessage::operator[](size_t index) {
  if (values_[index] == nullptr) {
    initValue(values_[index], members_.getMember(index), data_);
  }
  return values_[index];
}

Message::ConstSharedPtr CompoundMessage::operator[](size_t index) const {
  if (values_[index] == nullptr) {
    initValue(values_[index], members_.getMember(index), data_);
  }
  return values_[index];
}

std::shared_ptr<const void> CompoundMessage::type_erased_message() const {
  return data_;
}

std::shared_ptr<void> CompoundMessage::type_erased_message() {
  return data_;
}

CompoundMessage CompoundMessage::clone() const {
  CompoundMessage result(
    members_, createContainer(members_, rosidl_runtime_cpp::MessageInitialization::SKIP));
  result = *this;
  return result;
}

bool CompoundMessage::isValid() const {
  return data_ != nullptr;
}

void CompoundMessage::onMoved() {
  for (uint32_t i = 0; i < members_.value->member_count_; ++i) {
    if (values_[i] == nullptr) continue;
    if (values_[i]->type() != MessageType::Compound && values_[i]->type() != MessageType::Array) {
      values_[i]->move(data_);
      continue;
    }
    // Compound and array messages get a pointer to their exact memory location
    std::shared_ptr<void> sub_data(
      static_cast<uint8_t*>(data_.get()) + members_->members_[i].offset_,
      [data = this->data_](void*) {
        (void)data;
      });
    values_[i]->move(sub_data);
  }
}
}  // namespace ros2_babel_fish
