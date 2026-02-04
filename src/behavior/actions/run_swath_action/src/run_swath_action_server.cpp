// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "run_swath_action/run_swath_action_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace run_swath_action
{

RunSwathActionServer::RunSwathActionServer(const rclcpp::NodeOptions & options)
: Node("run_swath_action_server", options)
{
  // Declare parameters
  this->declare_parameter("default_planner_id", "StraightLine");
  this->declare_parameter("default_controller_id", "FollowPath");
  this->declare_parameter("default_path_pub_topic", "swath_path");

  default_planner_id_ = this->get_parameter("default_planner_id").as_string();
  default_controller_id_ = this->get_parameter("default_controller_id").as_string();
  default_path_pub_topic_ = this->get_parameter("default_path_pub_topic").as_string();

  // Create callback group for clients
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create action clients for Nav2 services
  compute_path_client_ = rclcpp_action::create_client<ComputePathThroughPoses>(
    this, "compute_path_through_poses", callback_group_);
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(
    this, "follow_path", callback_group_);

  // Create action server
  action_server_ = rclcpp_action::create_server<RunSwathAction>(
    this,
    "run_swath",
    std::bind(&RunSwathActionServer::handle_goal, this, _1, _2),
    std::bind(&RunSwathActionServer::handle_cancel, this, _1),
    std::bind(&RunSwathActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "RunSwathActionServer initialized");
}

rclcpp_action::GoalResponse RunSwathActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const RunSwathAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received run swath goal with %zu map_points",
    goal->map_points.size());

  if (goal->map_points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No map_points provided, rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RunSwathActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleRunSwath> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  is_cancelling_ = true;

  // Cancel follow_path if active
  if (follow_path_goal_handle_) {
    follow_path_client_->async_cancel_goal(follow_path_goal_handle_);
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RunSwathActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleRunSwath> goal_handle)
{
  std::thread{std::bind(&RunSwathActionServer::execute, this, goal_handle)}.detach();
}

void RunSwathActionServer::execute(
  const std::shared_ptr<GoalHandleRunSwath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing run swath goal");

  is_cancelling_ = false;
  follow_path_goal_handle_ = nullptr;
  distance_traveled_ = 0.0f;
  total_distance_ = 0.0f;
  current_waypoint_ = 0;

  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<RunSwathAction::Result>();

  // Get parameters from goal or use defaults
  std::string planner_id = goal->planner_id.empty() ? default_planner_id_ : goal->planner_id;
  std::string controller_id = goal->controller_id.empty() ? default_controller_id_ : goal->controller_id;
  std::string path_pub_topic = goal->path_pub_topic.empty() ? default_path_pub_topic_ : goal->path_pub_topic;

  // Create path publisher for this request
  if (current_path_pub_topic_ != path_pub_topic) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic, 10);
    current_path_pub_topic_ = path_pub_topic;
  }

  RCLCPP_INFO(this->get_logger(), "Running swath with %zu map_points using planner: %s, controller: %s",
    goal->map_points.size(), planner_id.c_str(), controller_id.c_str());

  // Wait for action servers
  if (!compute_path_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "compute_path_through_poses action server not available");
    result->error_code = RunSwathAction::Result::FAILED;
    goal_handle->abort(result);
    return;
  }

  if (!follow_path_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "follow_path action server not available");
    result->error_code = RunSwathAction::Result::FAILED;
    goal_handle->abort(result);
    return;
  }

  // Check for cancellation
  if (is_cancelling_) {
    result->error_code = RunSwathAction::Result::CANCELLED;
    goal_handle->canceled(result);
    return;
  }

  // Use map_points directly (already in map frame)
  std::vector<geometry_msgs::msg::PoseStamped> map_points = goal->map_points;

  // Ensure frame_id is set for all points
  for (auto & pose : map_points) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = "map";
    }
    pose.header.stamp = this->now();
  }

  RCLCPP_INFO(this->get_logger(), "Using %zu map_points", map_points.size());

  // Compute path through all poses
  nav_msgs::msg::Path path;
  if (!compute_path(map_points, planner_id, path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute path");
    result->error_code = RunSwathAction::Result::FAILED;
    goal_handle->abort(result);
    return;
  }

  // Calculate total distance
  total_distance_ = static_cast<float>(calculate_path_length(path));

  // Publish path
  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu poses, total distance: %.2f m",
    path.poses.size(), total_distance_);

  // Check for cancellation
  if (is_cancelling_) {
    result->error_code = RunSwathAction::Result::CANCELLED;
    goal_handle->canceled(result);
    return;
  }

  // Follow path
  if (!follow_path(path, controller_id, goal_handle)) {
    if (is_cancelling_) {
      result->error_code = RunSwathAction::Result::CANCELLED;
      goal_handle->canceled(result);
    } else {
      result->error_code = RunSwathAction::Result::FAILED;
      goal_handle->abort(result);
    }
    return;
  }

  // Success
  result->error_code = RunSwathAction::Result::NONE;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Run swath completed successfully");
}

bool RunSwathActionServer::compute_path(
  const std::vector<geometry_msgs::msg::PoseStamped> & goals,
  const std::string & planner_id,
  nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(this->get_logger(), "Computing path through %zu poses using planner: %s",
    goals.size(), planner_id.c_str());

  auto goal_msg = ComputePathThroughPoses::Goal();
  goal_msg.goals = goals;
  goal_msg.planner_id = planner_id;
  goal_msg.use_start = false;  // Use current robot pose

  // Use promise/future for synchronization
  auto result_promise = std::make_shared<std::promise<ComputePathThroughPoses::Result::SharedPtr>>();
  auto result_future = result_promise->get_future();

  auto send_goal_options = rclcpp_action::Client<ComputePathThroughPoses>::SendGoalOptions();

  send_goal_options.result_callback =
    [this, result_promise](const GoalHandleComputePath::WrappedResult & wrapped_result)
    {
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "compute_path_through_poses succeeded");
        result_promise->set_value(wrapped_result.result);
      } else {
        RCLCPP_ERROR(this->get_logger(), "compute_path_through_poses result code: %d, error_code: %d",
          static_cast<int>(wrapped_result.code),
          wrapped_result.result ? wrapped_result.result->error_code : -1);
        result_promise->set_value(nullptr);
      }
    };

  auto goal_handle_future = compute_path_client_->async_send_goal(goal_msg, send_goal_options);

  // Wait for goal acceptance
  if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for compute_path goal acceptance");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "compute_path goal was rejected");
    return false;
  }

  // Wait for result
  if (result_future.wait_for(60s) != std::future_status::ready) {
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

bool RunSwathActionServer::follow_path(
  const nav_msgs::msg::Path & path,
  const std::string & controller_id,
  const std::shared_ptr<GoalHandleRunSwath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Following path");

  auto goal_msg = FollowPath::Goal();
  goal_msg.path = path;
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
      distance_traveled_ = total_distance_ - feedback->distance_to_goal;

      // Publish our own feedback
      auto swath_feedback = std::make_shared<RunSwathAction::Feedback>();
      swath_feedback->distance_traveled = distance_traveled_;
      swath_feedback->total_distance = total_distance_;
      swath_feedback->current_waypoint = current_waypoint_;
      goal_handle->publish_feedback(swath_feedback);
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

  follow_path_goal_handle_ = goal_handle_future.get();
  if (!follow_path_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "follow_path goal was rejected");
    return false;
  }

  // Wait for result (poll with cancellation check)
  while (rclcpp::ok()) {
    auto status = result_future.wait_for(100ms);

    if (is_cancelling_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling follow_path");
      follow_path_client_->async_cancel_goal(follow_path_goal_handle_);
      result_future.wait_for(2s);
      return false;
    }

    if (status == std::future_status::ready) {
      break;
    }
  }

  auto result_code = result_future.get();
  follow_path_goal_handle_ = nullptr;

  if (result_code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "follow_path failed with code: %d",
      static_cast<int>(result_code));
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Path following completed");
  return true;
}

double RunSwathActionServer::calculate_path_length(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

}  // namespace run_swath_action

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(run_swath_action::RunSwathActionServer)
