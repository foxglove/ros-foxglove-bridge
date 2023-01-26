#include <future>

#include <rclcpp/client.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <foxglove_bridge/service_utils.hpp>

namespace {
static std::tuple<std::string, std::string, std::string> extract_type_identifier(
  const std::string& full_type) {
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (sep_position_back == std::string::npos || sep_position_back == 0 ||
      sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
      "Message type is not of the form package/type and cannot be processed");
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
}  // namespace

namespace foxglove_bridge {

constexpr size_t MSG_MEM_BLOCK_SIZE = 1024;
constexpr char LIB_NAME[] = "rosidl_typesupport_introspection_cpp";
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ServiceMembers;

std::string get_type_from_message_members(const MessageMembers* members) {
  std::string s = members->message_namespace_;
  s.replace(s.find("::"), sizeof("::") - 1, "/");
  return s + "/" + members->message_name_;
}

std::shared_ptr<void> allocate_message(const MessageMembers* members) {
  std::string str;
  size_t str_capacity = str.capacity();
  for (size_t i = 0; i < str_capacity + 1; i++) {
    str += "a";
  }

  std::wstring wstr;
  size_t wstr_capacity = wstr.capacity();
  for (size_t i = 0; i < wstr_capacity + 1; i++) {
    wstr += L"a";
  }

  uint8_t* buf = static_cast<uint8_t*>(malloc(members->size_of_ + MSG_MEM_BLOCK_SIZE));
  memset(buf, 0, MSG_MEM_BLOCK_SIZE);

  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto member = members->members_ + i;

    if (member->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
      memcpy(buf + member->offset_, new std::string(str), sizeof(std::string));
    }
    if (member->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING) {
      memcpy(buf + member->offset_, new std::wstring(wstr), sizeof(std::wstring));
    }
  }

  return std::shared_ptr<void>(buf);
}

GenericClient::GenericClient(rclcpp::node_interfaces::NodeBaseInterface* node_base,
                             rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                             std::string service_name, std::string service_type,
                             rcl_client_options_t& client_options)
    : rclcpp::ClientBase(node_base, node_graph) {
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(service_type);

  srv_ts_lib_ = rclcpp::get_typesupport_library(service_type, LIB_NAME);

  const std::string symbol_name =
    std::string(LIB_NAME) + "__get_service_type_support_handle__" + package_name + "__" +
    (middle_module.empty() ? "srv" : middle_module) + "__" + type_name;
  const rosidl_service_type_support_t* (*get_ts)() = nullptr;
  // This will throw runtime_error if the symbol was not found.
  srv_ts_hdl_ = (reinterpret_cast<decltype(get_ts)>(srv_ts_lib_->get_symbol(symbol_name)))();

  auto srv_members = static_cast<const ServiceMembers*>(srv_ts_hdl_->data);
  auto response_members = srv_members->response_members_;
  auto response_type = get_type_from_message_members(response_members);
  res_ts_lib_ = rclcpp::get_typesupport_library(response_type, LIB_NAME);
  res_ts_hdl_ = rclcpp::get_typesupport_handle(response_type, LIB_NAME, *res_ts_lib_);

  rcl_ret_t ret = rcl_client_init(this->get_client_handle().get(), this->get_rcl_node_handle(),
                                  srv_ts_hdl_, service_name.c_str(), &client_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(service_name, rcl_node_get_name(rcl_node_handle),
                                           rcl_node_get_namespace(rcl_node_handle), true);
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
  }
}

std::shared_ptr<void> GenericClient::create_response() {
  auto srv_members = static_cast<const ServiceMembers*>(srv_ts_hdl_->data);
  return allocate_message(srv_members->response_members_);
}

std::shared_ptr<rmw_request_id_t> GenericClient::create_request_header() {
  return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
}

void GenericClient::handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<void> response) {
  std::unique_lock<std::mutex> lock(pending_requests_mutex_);
  int64_t sequence_number = request_header->sequence_number;

  auto ser_response = std::make_shared<rclcpp::SerializedMessage>();
  rmw_ret_t r =
    rmw_serialize(response.get(), res_ts_hdl_, &ser_response->get_rcl_serialized_message());
  if (r != RMW_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("foxglove_bridge", "Failed to serialize service response. Ignoring...");
    return;
  }

  // TODO(esteve) this should throw instead since it is not expected to happen in the first place
  if (this->pending_requests_.count(sequence_number) == 0) {
    RCUTILS_LOG_ERROR_NAMED("foxglove_bridge", "Received invalid sequence number. Ignoring...");
    return;
  }
  auto tuple = this->pending_requests_[sequence_number];
  auto call_promise = std::get<0>(tuple);
  auto callback = std::get<1>(tuple);
  auto future = std::get<2>(tuple);
  this->pending_requests_.erase(sequence_number);
  // Unlock here to allow the service to be called recursively from one of its callbacks.
  lock.unlock();

  call_promise->set_value(ser_response);
  callback(future);
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request) {
  return async_send_request(request, [](SharedFuture) {});
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request,
                                                              CallbackType&& cb) {
  std::lock_guard<std::mutex> lock(pending_requests_mutex_);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(get_client_handle().get(),
                                   request->get_rcl_serialized_message().buffer, &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
  }

  SharedPromise call_promise = std::make_shared<Promise>();
  SharedFuture f(call_promise->get_future());
  pending_requests_[sequence_number] =
    std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
  return f;
}

}  // namespace foxglove_bridge
