// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <future>

#include "approach_swath_action/approach_swath_action_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace approach_swath_action
{

ApproachSwathActionServer::ApproachSwathActionServer(const rclcpp::NodeOptions & options)
: Node("approach_swath_action_server", options)
{
  // Declare parameters
  this->declare_parameter("default_planner_id", "GridBased");
  this->declare_parameter("default_controller_id", "FollowPath");
  this->declare_parameter("default_path_pub_topic", "to_start_path");

  default_planner_id_ = this->get_parameter("default_planner_id").as_string();
  default_controller_id_ = this->get_parameter("default_controller_id").as_string();
  default_path_pub_topic_ = this->get_parameter("default_path_pub_topic").as_string();

  // Create callback group for action clients (allows concurrent callbacks)
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create action clients for Nav2 services
  compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this, "compute_path_to_pose", callback_group_);
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(
    this, "follow_path", callback_group_);

  // Create action server
  action_server_ = rclcpp_action::create_server<ApproachSwathAction>(
    this,
    "approach_swath",
    std::bind(&ApproachSwathActionServer::handle_goal, this, _1, _2),
    std::bind(&ApproachSwathActionServer::handle_cancel, this, _1),
    std::bind(&ApproachSwathActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "ApproachSwathActionServer initialized");
}

rclcpp_action::GoalResponse ApproachSwathActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ApproachSwathAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received approach swath goal with %zu map_points",
    goal->map_points.size());

  if (goal->map_points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No map_points provided, rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ApproachSwathActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleApproachSwath> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  is_cancelling_ = true;

  // Cancel follow_path if active (thread-safe)
  cancel_follow_path();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ApproachSwathActionServer::cancel_follow_path()
{
  std::lock_guard<std::mutex> lock(goal_handle_mutex_);
  if (follow_path_goal_handle_) {
    try {
      follow_path_client_->async_cancel_goal(follow_path_goal_handle_);
      RCLCPP_INFO(this->get_logger(), "Sent cancel request for follow_path");
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError & e) {
      RCLCPP_WARN(this->get_logger(), "Goal handle already invalid: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Error cancelling follow_path: %s", e.what());
    }
    follow_path_goal_handle_ = nullptr;
  }
}

void ApproachSwathActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleApproachSwath> goal_handle)
{
  std::thread{std::bind(&ApproachSwathActionServer::execute, this, goal_handle)}.detach();
}

void ApproachSwathActionServer::execute(
  const std::shared_ptr<GoalHandleApproachSwath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing approach swath goal");

  is_cancelling_ = false;
  follow_path_goal_handle_ = nullptr;
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ApproachSwathAction::Result>();

  // Get parameters from goal or use defaults
  std::string planner_id = goal->planner_id.empty() ? default_planner_id_ : goal->planner_id;
  std::string controller_id = goal->controller_id.empty() ? default_controller_id_ : goal->controller_id;
  std::string path_pub_topic = goal->path_pub_topic.empty() ? default_path_pub_topic_ : goal->path_pub_topic;

  // Create path publisher for this request
  if (current_path_pub_topic_ != path_pub_topic) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic, 10);
    current_path_pub_topic_ = path_pub_topic;
  }

  // Get goal pose (first point in map_points)
  geometry_msgs::msg::PoseStamped goal_pose = goal->map_points[0];

  RCLCPP_INFO(this->get_logger(), "Approaching goal [x: %.2f, y: %.2f] using planner: %s, controller: %s",
    goal_pose.pose.position.x, goal_pose.pose.position.y,
    planner_id.c_str(), controller_id.c_str());

  // Wait for action servers
  if (!compute_path_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose action server not available");
    result->error_code = ApproachSwathAction::Result::FAILED;
    goal_handle->abort(result);
    return;
  }

  if (!follow_path_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "follow_path action server not available");
    result->error_code = ApproachSwathAction::Result::FAILED;
    goal_handle->abort(result);
    return;
  }

  // Check for cancellation
  if (is_cancelling_) {
    result->error_code = ApproachSwathAction::Result::CANCELLED;
    goal_handle->canceled(result);
    return;
  }

  // Step 1: Compute path to goal
  nav_msgs::msg::Path path;
  if (!compute_path(goal_pose, planner_id, path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute path");
    result->error_code = ApproachSwathAction::Result::FAILED;
    goal_handle->abort(result);
    return;
  }

  // Publish path (even if empty, for visualization)
  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path.poses.size());

  // Check if already at goal (path is empty or very short)
  if (path.poses.size() <= 1) {
    RCLCPP_INFO(this->get_logger(), "Already at goal position, skipping path following");
    result->error_code = ApproachSwathAction::Result::NONE;
    goal_handle->succeed(result);
    return;
  }

  // Check for cancellation
  if (is_cancelling_) {
    result->error_code = ApproachSwathAction::Result::CANCELLED;
    goal_handle->canceled(result);
    return;
  }

  // Step 2: Follow path
  if (!follow_path(path, controller_id, goal_handle)) {
    if (is_cancelling_) {
      result->error_code = ApproachSwathAction::Result::CANCELLED;
      goal_handle->canceled(result);
    } else {
      result->error_code = ApproachSwathAction::Result::FAILED;
      goal_handle->abort(result);
    }
    return;
  }

  // Success
  result->error_code = ApproachSwathAction::Result::NONE;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Approach swath completed successfully");
}

bool ApproachSwathActionServer::compute_path(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & planner_id,
  nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(this->get_logger(), "Computing path to goal using planner: %s", planner_id.c_str());

  auto goal_msg = ComputePathToPose::Goal();
  goal_msg.goal = goal_pose;

  // Ensure frame_id is set
  if (goal_msg.goal.header.frame_id.empty()) {
    goal_msg.goal.header.frame_id = "map";
  }
  goal_msg.goal.header.stamp = this->now();

  goal_msg.planner_id = planner_id;
  goal_msg.use_start = false;  // Use current robot pose

  // Use promise/future for synchronization
  auto result_promise = std::make_shared<std::promise<ComputePathToPose::Result::SharedPtr>>();
  auto result_future = result_promise->get_future();

  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

  send_goal_options.result_callback =
    [this, result_promise](const GoalHandleComputePath::WrappedResult & wrapped_result)
    {
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "compute_path succeeded");
        result_promise->set_value(wrapped_result.result);
      } else {
        RCLCPP_ERROR(this->get_logger(), "compute_path result code: %d, error_code: %d",
          static_cast<int>(wrapped_result.code),
          wrapped_result.result ? wrapped_result.result->error_code : -1);
        result_promise->set_value(nullptr);
      }
    };

  auto goal_handle_future = compute_path_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal acceptance (with timeout)
  if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for compute_path goal acceptance");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "compute_path goal was rejected");
    return false;
  }

  // Wait for result (with timeout)
  if (result_future.wait_for(30s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for compute_path result");
    return false;
  }

  auto result = result_future.get();
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "compute_path failed");
    return false;
  }

  path = result->path;
  RCLCPP_INFO(this->get_logger(), "Path computed with %zu poses", path.poses.size());
  return true;
}

bool ApproachSwathActionServer::follow_path(
  const nav_msgs::msg::Path & path,
  const std::string & controller_id,
  const std::shared_ptr<GoalHandleApproachSwath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Following path with %zu poses", path.poses.size());

  auto goal_msg = FollowPath::Goal();
  goal_msg.path = path;
  // Update path timestamp to current time to avoid stale path rejection
  goal_msg.path.header.stamp = this->now();
  goal_msg.controller_id = controller_id;

  // Use promise/future for synchronization
  auto result_promise = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
  auto result_future = result_promise->get_future();

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

  // Feedback callback
  send_goal_options.feedback_callback =
    [this, goal_handle](
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
      distance_remaining_ = feedback->distance_to_goal;

      // Publish our own feedback
      auto approach_feedback = std::make_shared<ApproachSwathAction::Feedback>();
      approach_feedback->distance_remaining = distance_remaining_;
      goal_handle->publish_feedback(approach_feedback);
    };

  // Result callback
  send_goal_options.result_callback =
    [result_promise](const GoalHandleFollowPath::WrappedResult & wrapped_result)
    {
      result_promise->set_value(wrapped_result.code);
    };

  auto goal_handle_future = follow_path_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal acceptance
  if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for follow_path goal acceptance");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    follow_path_goal_handle_ = goal_handle_future.get();
    if (!follow_path_goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "follow_path goal was rejected");
      return false;
    }
  }

  // Wait for result (poll with cancellation check)
  while (rclcpp::ok()) {
    auto status = result_future.wait_for(100ms);

    if (is_cancelling_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling follow_path from execute loop");
      cancel_follow_path();
      // Wait a bit for cancellation to process
      result_future.wait_for(2s);
      return false;
    }

    if (status == std::future_status::ready) {
      break;
    }
  }

  auto result_code = result_future.get();
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    follow_path_goal_handle_ = nullptr;
  }

  if (result_code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "follow_path failed with code: %d",
      static_cast<int>(result_code));
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Path following completed");
  return true;
}

}  // namespace approach_swath_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(approach_swath_action::ApproachSwathActionServer)
