// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license
// information.

#ifndef ROS2_BABEL_FISH_BABEL_FISH_ACTION_CLIENT_HPP
#define ROS2_BABEL_FISH_BABEL_FISH_ACTION_CLIENT_HPP

#include <rclcpp_action/client.hpp>

#include "ros2_babel_fish/messages/compound_message.hpp"

namespace ros2_babel_fish {
struct ActionTypeSupport;

namespace impl {
struct BabelFishAction {
  using Feedback = CompoundMessage;
  using Goal = CompoundMessage;
  using Result = CompoundMessage;

  typedef struct Impl {
    struct CancelGoalService {
      using Request = CompoundMessage;
      using Response = CompoundMessage;
    };
  } Impl;
};
}  // namespace impl
}  // namespace ros2_babel_fish

namespace rclcpp_action {
template <>
class Client<ros2_babel_fish::impl::BabelFishAction> : public rclcpp_action::ClientBase {
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(Client<ros2_babel_fish::impl::BabelFishAction>)

  using GoalHandle = rclcpp_action::ClientGoalHandle<ros2_babel_fish::impl::BabelFishAction>;
  using WrappedResult = GoalHandle::WrappedResult;
  using GoalResponseCallback = std::function<void(typename GoalHandle::SharedPtr)>;
  using FeedbackCallback = GoalHandle::FeedbackCallback;
  using ResultCallback = GoalHandle::ResultCallback;
  using CancelRequest = ros2_babel_fish::CompoundMessage;
  using CancelResponse = ros2_babel_fish::CompoundMessage;
  using CancelCallback = std::function<void(CancelResponse)>;

  /// Options for sending a goal.
  /**
   * This struct is used to pass parameters to the function `async_send_goal`.
   */
  struct SendGoalOptions {
    /// Function called when the goal is accepted or rejected.
    /**
     * Takes a single argument that is a goal handle shared pointer.
     * If the goal is accepted, then the pointer points to a valid goal handle.
     * If the goal is rejected, then pointer has the value `nullptr`.
     */
    GoalResponseCallback goal_response_callback;

    /// Function called whenever feedback is received for the goal.
    FeedbackCallback feedback_callback;

    /// Function called when the result for the goal is received.
    ResultCallback result_callback;
  };

  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string& action_name,
    std::shared_ptr<const ros2_babel_fish::ActionTypeSupport> type_support,
    const rcl_action_client_options_t& client_options = rcl_action_client_get_default_options());

  ros2_babel_fish::CompoundMessage create_goal() const;

  std::shared_future<GoalHandle::SharedPtr> async_send_goal(
    const ros2_babel_fish::CompoundMessage& goal, const SendGoalOptions& options = {});

  /// Asynchronously get the result for an active goal.
  /**
   * @throws exceptions::UnknownGoalHandleError If the goal unknown or already reached a terminal
   *   state, or if there was an error requesting the result.
   * @param[in] goal_handle The goal handle for which to get the result.
   * @param[in] result_callback Optional callback that is called when the result is received.
   *   Will overwrite any result callback specified in async_send_goal!
   * @return A future that is set to the goal result when the goal is finished.
   */
  std::shared_future<WrappedResult> async_get_result(typename GoalHandle::SharedPtr goal_handle,
                                                     ResultCallback result_callback = nullptr);

  /// Asynchronously request a goal be canceled.
  /**
   * \throws exceptions::UnknownGoalHandleError If the goal is unknown or already reached a
   *   terminal state.
   * \param[in] goal_handle The goal handle requesting to be canceled.
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  std::shared_future<CancelResponse> async_cancel_goal(GoalHandle::SharedPtr goal_handle,
                                                       CancelCallback cancel_callback = nullptr);

  /// Asynchronously request for all goals to be canceled.
  /**
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  std::shared_future<CancelResponse> async_cancel_all_goals(
    CancelCallback cancel_callback = nullptr);

  /// Asynchronously request all goals at or before a specified time be canceled.
  /**
   * \param[in] stamp The timestamp for the cancel goal request.
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  std::shared_future<CancelResponse> async_cancel_goals_before(
    const rclcpp::Time& stamp, CancelCallback cancel_callback = nullptr);

protected:
  std::shared_ptr<void> create_goal_response() const override;

  std::shared_ptr<void> create_result_response() const override;

  std::shared_ptr<void> create_cancel_response() const override;

  std::shared_ptr<void> create_feedback_message() const override;

  void handle_feedback_message(std::shared_ptr<void> message) override;

  std::shared_ptr<void> create_status_message() const override;

  void handle_status_message(std::shared_ptr<void> message) override;

  void make_result_aware(GoalHandle::SharedPtr goal_handle);

  std::shared_future<CancelResponse> async_cancel(CancelRequest cancel_request,
                                                  CancelCallback cancel_callback = nullptr);

private:
  std::shared_ptr<const ros2_babel_fish::ActionTypeSupport> type_support_;
  std::mutex goal_handles_mutex_;
  std::map<GoalUUID, typename GoalHandle::WeakPtr> goal_handles_;
};
}  // namespace rclcpp_action

namespace ros2_babel_fish {

using BabelFishActionClient = rclcpp_action::Client<impl::BabelFishAction>;

}

#endif  // ROS2_BABEL_FISH_BABEL_FISH_ACTION_CLIENT_HPP
