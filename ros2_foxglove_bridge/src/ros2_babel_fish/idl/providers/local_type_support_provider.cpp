// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/idl/providers/local_type_support_provider.hpp"

#include <sstream>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <rcpputils/shared_library.hpp>
#include <rcutils/error_handling.h>
#include <rosidl_typesupport_c/identifier.h>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_cpp/service_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ros2_babel_fish/exceptions/babel_fish_exception.hpp"

namespace ros2_babel_fish {
namespace {
// ==== This block of code was taken from rosbag2_cpp and is under the following license ===
// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
std::string get_typesupport_library_path(const std::string& package_name,
                                         const std::string& typesupport_identifier) {
  const char* filename_prefix;
  const char* filename_extension;
  const char* dynamic_library_folder;
#ifdef _WIN32
  filename_prefix = "";
  filename_extension = ".dll";
  dynamic_library_folder = "/bin/";
#elif __APPLE__
  filename_prefix = "lib";
  filename_extension = ".dylib";
  dynamic_library_folder = "/lib/";
#else
  filename_prefix = "lib";
  filename_extension = ".so";
  dynamic_library_folder = "/lib/";
#endif

  std::string package_prefix;
  try {
    package_prefix = ament_index_cpp::get_package_prefix(package_name);
  } catch (ament_index_cpp::PackageNotFoundError& e) {
    throw BabelFishException(e.what());
  }

  auto library_path = package_prefix + dynamic_library_folder + filename_prefix + package_name +
                      "__" + typesupport_identifier + filename_extension;
  return library_path;
}

std::tuple<std::string, std::string, std::string> extract_type_identifier(
  const std::string& full_type) {
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (sep_position_back == std::string::npos || sep_position_back == 0 ||
      sep_position_back == full_type.length() - 1) {
    throw BabelFishException("Message type '" + full_type +
                             "' is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(
  const std::string& type, const std::string& typesupport_identifier) {
  auto package_name = std::get<0>(extract_type_identifier(type));
  auto library_path = get_typesupport_library_path(package_name, typesupport_identifier);
  return std::make_shared<rcpputils::SharedLibrary>(library_path);
}

const rosidl_message_type_support_t* get_typesupport_handle(
  const std::string& type, const std::string& typesupport_identifier,
  const std::shared_ptr<rcpputils::SharedLibrary>& library) {
  if (nullptr == library) {
    throw BabelFishException(
      "rcpputils::SharedLibrary not initialized. Call get_typesupport_library first.");
  }

  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);
  if (middle_module.empty()) middle_module = "msg";

  std::stringstream rcutils_dynamic_loading_error;
  rcutils_dynamic_loading_error
    << "Something went wrong loading the typesupport library for message type " << package_name
    << "/" << middle_module << "/" << type_name << ".";

  try {
    auto symbol_name = typesupport_identifier + "__get_message_type_support_handle__" +
                       package_name + "__" + middle_module + "__" + type_name;

    if (!library->has_symbol(symbol_name)) {
      throw std::runtime_error(
        std::string(
          " Could not find symbol for message type support handle getter. rcutils error: ") +
        rcutils_get_error_string().str +
        ". This is probably due to https://github.com/ros2/rosidl_typesupport/pull/114 not being "
        "merged yet.");
    }

    const rosidl_message_type_support_t* (*get_ts)() = nullptr;
    get_ts = (decltype(get_ts))library->get_symbol(symbol_name);

    if (!get_ts) {
      throw std::runtime_error{" Symbol of wrong type."};
    }
    auto type_support = get_ts();
    return type_support;
  } catch (std::runtime_error& e) {
    throw BabelFishException(rcutils_dynamic_loading_error.str() + e.what());
  }
}
// ==== End of block ===

const rosidl_service_type_support_t* get_service_typesupport_handle(
  const std::string& type, const std::string& typesupport_identifier,
  const std::shared_ptr<rcpputils::SharedLibrary>& library) {
  if (nullptr == library) {
    throw BabelFishException(
      "rcpputils::SharedLibrary not initialized. Call get_typesupport_library first.");
  }

  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);
  if (middle_module.empty()) middle_module = "srv";

  std::stringstream rcutils_dynamic_loading_error;
  rcutils_dynamic_loading_error
    << "Something went wrong loading the typesupport library for service type " << package_name
    << "/" << middle_module << "/" << type_name << " from library: " << library->get_library_path()
    << ".";

  try {
    auto symbol_name = typesupport_identifier + "__get_service_type_support_handle__" +
                       package_name + "__" + (middle_module.empty() ? "srv" : middle_module) +
                       "__" + type_name;
    if (!library->has_symbol(symbol_name)) {
      throw std::runtime_error(
        std::string(
          " Could not find symbol for message type support handle getter. rcutils error: ") +
        rcutils_get_error_string().str +
        ". This is probably due to https://github.com/ros2/rosidl_typesupport/pull/114 not being "
        "merged yet.");
    }

    const rosidl_service_type_support_t* (*get_ts)() = nullptr;
    get_ts = (decltype(get_ts))library->get_symbol(symbol_name);

    if (!get_ts) {
      throw std::runtime_error{" Symbol of wrong type."};
    }
    auto type_support = get_ts();
    return type_support;
  } catch (std::runtime_error& e) {
    throw BabelFishException(rcutils_dynamic_loading_error.str() + e.what());
  }
}

const rosidl_action_type_support_t* get_action_typesupport_handle(
  const std::string& type, const std::string& typesupport_identifier,
  const std::shared_ptr<rcpputils::SharedLibrary>& library) {
  if (nullptr == library) {
    throw std::runtime_error(
      "rcpputils::SharedLibrary not initialized. Call get_typesupport_library first.");
  }

  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);
  if (middle_module.empty()) middle_module = "action";

  std::stringstream rcutils_dynamic_loading_error;
  rcutils_dynamic_loading_error
    << "Something went wrong loading the typesupport library for action type " << package_name
    << "/" << middle_module << "/" << type_name << " from library: " << library->get_library_path()
    << ".";

  try {
    auto symbol_name = typesupport_identifier + "__get_action_type_support_handle__" +
                       package_name + "__" + (middle_module.empty() ? "action" : middle_module) +
                       "__" + type_name;

    if (!library->has_symbol(symbol_name)) {
      throw std::runtime_error(
        std::string(
          " Could not find symbol for message type support handle getter. rcutils error: ") +
        rcutils_get_error_string().str +
        ". This is probably due to https://github.com/ros2/rosidl_typesupport/pull/114 not being "
        "merged yet.");
    }

    const rosidl_action_type_support_t* (*get_ts)() = nullptr;
    get_ts = (decltype(get_ts))library->get_symbol(symbol_name);

    if (!get_ts) {
      throw std::runtime_error{" Symbol of wrong type."};
    }
    auto type_support = get_ts();
    return type_support;
  } catch (std::runtime_error& e) {
    throw BabelFishException(rcutils_dynamic_loading_error.str() + e.what());
  }
}
}  // namespace

LocalTypeSupportProvider::LocalTypeSupportProvider() = default;

MessageTypeSupport::ConstSharedPtr LocalTypeSupportProvider::getMessageTypeSupportImpl(
  const std::string& type) const {
  auto type_support_library =
    get_typesupport_library(type, rosidl_typesupport_cpp::typesupport_identifier);
  auto type_support_handle = get_typesupport_handle(
    type, rosidl_typesupport_cpp::typesupport_identifier, type_support_library);
  auto introspection_type_support_library =
    get_typesupport_library(type, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  auto introspection_type_support_handle =
    get_typesupport_handle(type, rosidl_typesupport_introspection_cpp::typesupport_identifier,
                           introspection_type_support_library);
  return registerMessage(type, type_support_library, *type_support_handle,
                         introspection_type_support_library, *introspection_type_support_handle);
}

ServiceTypeSupport::ConstSharedPtr LocalTypeSupportProvider::getServiceTypeSupportImpl(
  const std::string& type) const {
  auto type_support_library =
    get_typesupport_library(type, rosidl_typesupport_cpp::typesupport_identifier);
  auto type_support_handle = get_service_typesupport_handle(
    type, rosidl_typesupport_cpp::typesupport_identifier, type_support_library);
  // For introspection the cpp version is available
  auto introspection_type_support_library =
    get_typesupport_library(type, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  auto introspection_type_support_handle = get_service_typesupport_handle(
    type, rosidl_typesupport_introspection_cpp::typesupport_identifier,
    introspection_type_support_library);
  return registerService(type, type_support_library, *type_support_handle,
                         introspection_type_support_library, *introspection_type_support_handle);
}

ActionTypeSupport::ConstSharedPtr LocalTypeSupportProvider::getActionTypeSupportImpl(
  const std::string& type) const {
  auto type_support_library =
    get_typesupport_library(type, rosidl_typesupport_cpp::typesupport_identifier);
  auto type_support_handle = get_action_typesupport_handle(
    type, rosidl_typesupport_cpp::typesupport_identifier, type_support_library);
  auto introspection_type_support_library =
    get_typesupport_library(type, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  // There is no method to get the whole action introspection type support, hence, we need to get
  // each separately
  rosidl_action_type_support_t introspection;
  introspection.goal_service_type_support = get_service_typesupport_handle(
    type + "_SendGoal", rosidl_typesupport_introspection_cpp::typesupport_identifier,
    introspection_type_support_library);
  introspection.result_service_type_support = get_service_typesupport_handle(
    type + "_GetResult", rosidl_typesupport_introspection_cpp::typesupport_identifier,
    introspection_type_support_library);
  introspection.feedback_message_type_support = get_typesupport_handle(
    type + "_FeedbackMessage", rosidl_typesupport_introspection_cpp::typesupport_identifier,
    introspection_type_support_library);
  auto result = std::make_shared<ActionTypeSupport>();
  result->name = type;
  result->type_support_library = type_support_library;
  result->type_support_handle = *type_support_handle;
  result->introspection_type_support_library = introspection_type_support_library;

  result->goal_service_type_support = registerService(
    type + "_SendGoal", type_support_library, *type_support_handle->goal_service_type_support,
    introspection_type_support_library, *introspection.goal_service_type_support);
  result->result_service_type_support = registerService(
    type + "_GetResult", type_support_library, *type_support_handle->result_service_type_support,
    introspection_type_support_library, *introspection.result_service_type_support);
  result->feedback_message_type_support = registerMessage(
    type + "_FeedbackMessage", type_support_library,
    *type_support_handle->feedback_message_type_support, introspection_type_support_library,
    *introspection.feedback_message_type_support);

  // The other type supports are from a different package and we have to look them up
  result->cancel_service_type_support = getServiceTypeSupport("action_msgs/srv/CancelGoal");
  result->status_message_type_support = getMessageTypeSupport("action_msgs/msg/GoalStatusArray");
  introspection.cancel_service_type_support =
    &result->cancel_service_type_support->introspection_type_support_handle;
  introspection.status_message_type_support =
    &result->status_message_type_support->introspection_type_support_handle;
  result->introspection_type_support_handle = introspection;

  return registerAction(type, result);
}
}  // namespace ros2_babel_fish
