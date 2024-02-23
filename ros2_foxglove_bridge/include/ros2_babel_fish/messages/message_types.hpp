// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS2_BABEL_FISH_MESSAGE_TYPES_HPP
#define ROS2_BABEL_FISH_MESSAGE_TYPES_HPP

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

namespace ros2_babel_fish
{

namespace MessageTypes
{
enum MessageType : uint8_t {
  None = 0x00000,
  Float = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,
  Double = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,
  LongDouble = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE,
  Char = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR,
  WChar = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR,
  Bool = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,
  Octet = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET,
  UInt8 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,
  Int8 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,
  UInt16 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,
  Int16 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,
  UInt32 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,
  Int32 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,
  UInt64 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,
  Int64 = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,
  String = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,
  WString = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING,
  Compound = ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,
  Array = 200,
};
}
typedef MessageTypes::MessageType MessageType;
} // namespace ros2_babel_fish

#endif // ROS2_BABEL_FISH_MESSAGE_TYPES_HPP
