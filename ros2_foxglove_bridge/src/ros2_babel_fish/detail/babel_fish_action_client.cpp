// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#include "ros2_babel_fish/detail/babel_fish_action_client.hpp"

#include <cstring>

#include "ros2_babel_fish/idl/serialization.hpp"
#include "ros2_babel_fish/messages/array_message.hpp"
#include "ros2_babel_fish/messages/compound_message.hpp"

using namespace ros2_babel_fish;

namespace rclcpp_action {

Client<ros2_babel_fish::impl::BabelFishAction>::Client(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string& action_name, ros2_babel_fish::ActionTypeSupport::ConstSharedPtr type_support,
  const rcl_action_client_options_t& client_options)
    : ClientBase(node_base, node_graph, node_logging, action_name,
                 &type_support->type_support_handle, client_options)
    , type_support_(std::move(type_support)) {}

CompoundMessage Client<impl::BabelFishAction>::create_goal() const {
  MessageMembersIntrospection introspection = type_support_->goal_service_type_support->request();
  size_t index =
    std::find_if(introspection->members_, introspection->members_ + introspection->member_count_,
                 [](const auto& a) {
                   return std::strcmp(a.name_, "goal") == 0;
                 }) -
    introspection->members_;
  return CompoundMessage(type_support_->goal_service_type_support->request().getMember(index));
}

std::shared_future<
  rclcpp_action::ClientGoalHandle<ros2_babel_fish::impl::BabelFishAction>::SharedPtr>
Client<ros2_babel_fish::impl::BabelFishAction>::async_send_goal(const CompoundMessage& goal,
                                                                const SendGoalOptions& options) {
  auto promise = std::make_shared<std::promise<typename GoalHandle::SharedPtr>>();
  auto future = promise->get_future();
  CompoundMessage goal_request(type_support_->goal_service_type_support->request());
  auto uuid = this->generate_goal_id();
  auto& request_uuid = goal_request["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>();
  for (size_t i = 0; i < uuid.size(); ++i) request_uuid[i] = uuid[i];
  goal_request["goal"] = goal;
  this->send_goal_request(
    goal_request.type_erased_message(),
    [this, promise, options = options, uuid](std::shared_ptr<void> response) {
      CompoundMessage goal_response(type_support_->goal_service_type_support->response(), response);
      if (!goal_response["accepted"].value<bool>()) {
        promise->set_value(nullptr);
        if (options.goal_response_callback) options.goal_response_callback(nullptr);
        return;
      }
      rclcpp_action::GoalInfo goal_info;
      goal_info.goal_id.uuid = uuid;
      goal_info.stamp = goal_response["stamp"].value<rclcpp::Time>();
      std::shared_ptr<GoalHandle> goal_handle(
        new GoalHandle(goal_info, options.feedback_callback, options.result_callback));
      {
        std::lock_guard<std::mutex> guard(goal_handles_mutex_);
        goal_handles_[uuid] = goal_handle;
      }
      promise->set_value(goal_handle);
      if (options.goal_response_callback) options.goal_response_callback(goal_handle);

      if (options.result_callback) this->make_result_aware(goal_handle);
    });

  // Clean old goal handles
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    auto it = goal_handles_.begin();
    while (it != goal_handles_.end()) {
      if (!it->second.lock())
        it = goal_handles_.erase(it);
      else
        ++it;
    }
  }
  return future;
}

std::shared_future<
  rclcpp_action::ClientGoalHandle<ros2_babel_fish::impl::BabelFishAction>::WrappedResult>
Client<impl::BabelFishAction>::async_get_result(typename GoalHandle::SharedPtr goal_handle,
                                                ResultCallback result_callback) {
  std::lock_guard<std::mutex> lock(goal_handles_mutex_);
  if (goal_handles_.find(goal_handle->get_goal_id()) == goal_handles_.end())
    throw exceptions::UnknownGoalHandleError();

#if RCLCPP_VERSION_MAJOR >= 9
  if (goal_handle->is_invalidated()) {
    throw goal_handle->invalidate_exception_;
  }
#endif
  if (result_callback) goal_handle->set_result_callback(result_callback);
  this->make_result_aware(goal_handle);
  return goal_handle->async_get_result();
}

std::shared_future<ros2_babel_fish::CompoundMessage>
Client<impl::BabelFishAction>::async_cancel_goal(GoalHandle::SharedPtr goal_handle,
                                                 CancelCallback cancel_callback) {
  std::lock_guard<std::mutex> lock(goal_handles_mutex_);
  if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
    throw exceptions::UnknownGoalHandleError();
  }
  CompoundMessage cancel_request(type_support_->cancel_service_type_support->request());
  const auto& uuid = goal_handle->get_goal_id();
  auto& request_uuid =
    cancel_request["goal_info"]["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>();
  for (size_t i = 0; i < uuid.size(); ++i) request_uuid[i] = uuid[i];
  return async_cancel(cancel_request, cancel_callback);
}

std::shared_future<ros2_babel_fish::CompoundMessage>
Client<impl::BabelFishAction>::async_cancel_all_goals(CancelCallback cancel_callback) {
  CompoundMessage cancel_request(type_support_->cancel_service_type_support->request());
  cancel_request["goal_info"]["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>().fill(0u);
  return async_cancel(cancel_request, std::move(cancel_callback));
}

std::shared_future<ros2_babel_fish::CompoundMessage>
Client<impl::BabelFishAction>::async_cancel_goals_before(const rclcpp::Time& stamp,
                                                         CancelCallback cancel_callback) {
  CompoundMessage cancel_request(type_support_->cancel_service_type_support->request());
  cancel_request["goal_info"]["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>().fill(0u);
  cancel_request["goal_info"]["stamp"] = stamp;
  return async_cancel(cancel_request, cancel_callback);
}

std::shared_future<ros2_babel_fish::CompoundMessage> Client<impl::BabelFishAction>::async_cancel(
  CompoundMessage cancel_request, CancelCallback cancel_callback) {
  // Put promise in the heap to move it around.
  auto promise = std::make_shared<std::promise<CancelResponse>>();
  std::shared_future<CancelResponse> future(promise->get_future());
  this->send_cancel_request(
    cancel_request.type_erased_message(),
    [this, cancel_callback, promise](std::shared_ptr<void> response) mutable {
      CompoundMessage cancel_response(type_support_->cancel_service_type_support->response(),
                                      response);
      promise->set_value(cancel_response);
      if (cancel_callback) {
        cancel_callback(cancel_response);
      }
    });
  return future;
}

std::shared_ptr<void> Client<impl::BabelFishAction>::create_goal_response() const {
  return createContainer(type_support_->goal_service_type_support->response());
}

std::shared_ptr<void> Client<impl::BabelFishAction>::create_result_response() const {
  return createContainer(type_support_->result_service_type_support->response());
}

std::shared_ptr<void> Client<impl::BabelFishAction>::create_cancel_response() const {
  return createContainer(type_support_->cancel_service_type_support->response());
}

std::shared_ptr<void> Client<impl::BabelFishAction>::create_feedback_message() const {
  return createContainer(*type_support_->feedback_message_type_support);
}

void Client<impl::BabelFishAction>::handle_feedback_message(std::shared_ptr<void> message) {
  CompoundMessage feedback_message(*type_support_->feedback_message_type_support, message);
  GoalUUID goal_id;
  const auto& feedback_goal_id =
    feedback_message["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>();
  for (size_t i = 0; i < goal_id.size(); ++i) goal_id[i] = feedback_goal_id[i];

  std::lock_guard<std::mutex> guard(goal_handles_mutex_);
  // Feedback is not for one of our goal handles
  auto it = goal_handles_.find(goal_id);
  if (it == goal_handles_.end()) return;
  GoalHandle::SharedPtr goal_handle = it->second.lock();
  if (!goal_handle) {
    // No more references to goal handle => drop it
    goal_handles_.erase(it);
    return;
  }
  goal_handle->call_feedback_callback(
    goal_handle, CompoundMessage::make_shared(feedback_message["feedback"].as<CompoundMessage>()));
}

std::shared_ptr<void> Client<impl::BabelFishAction>::create_status_message() const {
  return createContainer(*type_support_->status_message_type_support);
}

void Client<impl::BabelFishAction>::handle_status_message(std::shared_ptr<void> message) {
  CompoundMessage status_message(*type_support_->status_message_type_support, message);

  std::lock_guard<std::mutex> guard(goal_handles_mutex_);
  const auto& status_list = status_message["status_list"].as<CompoundArrayMessage>();
  for (size_t i = 0; i < status_list.size(); ++i) {
    const auto& status = status_list[i];
    GoalUUID goal_id;
    const auto& status_goal_id =
      status["goal_info"]["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>();
    for (size_t k = 0; k < goal_id.size(); ++k) goal_id[k] = status_goal_id[k];

    // Status is not for one of our goal handles
    auto it = goal_handles_.find(goal_id);
    if (it == goal_handles_.end()) return;
    GoalHandle::SharedPtr goal_handle = it->second.lock();
    if (!goal_handle) {
      // No more references to goal handle => drop it
      goal_handles_.erase(it);
      return;
    }

    goal_handle->set_status(status["status"].value<int8_t>());
  }
}

void Client<ros2_babel_fish::impl::BabelFishAction>::make_result_aware(
  GoalHandle::SharedPtr goal_handle) {
  // Avoid making more than one request
  if (goal_handle->set_result_awareness(true)) return;
  CompoundMessage goal_result_request(type_support_->goal_service_type_support->request());
  const auto& uuid = goal_handle->get_goal_id();
  auto& request_uuid =
    goal_result_request["goal_id"]["uuid"].as<FixedLengthArrayMessage<uint8_t>>();
  for (size_t i = 0; i < uuid.size(); ++i) request_uuid[i] = uuid[i];
  try {
    this->send_result_request(
      goal_result_request.type_erased_message(), [goal_handle, this](std::shared_ptr<void> data) {
        CompoundMessage response(type_support_->result_service_type_support->response(), data);
        WrappedResult wrapped_result;
        wrapped_result.result =
          CompoundMessage::make_shared(response["result"].as<CompoundMessage>());
        wrapped_result.goal_id = goal_handle->get_goal_id();
        wrapped_result.code = static_cast<ResultCode>(response["status"].value<int8_t>());
        goal_handle->set_result(wrapped_result);
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        goal_handles_.erase(goal_handle->get_goal_id());
      });
  } catch (rclcpp::exceptions::RCLError& ex) {
    // Cause exception when user tries to access result
#if RCLCPP_VERSION_MAJOR >= 9
    goal_handle->invalidate(exceptions::UnawareGoalHandleError(ex.message));
#else
    goal_handle->invalidate();
#endif
  }
}
}  // namespace rclcpp_action
