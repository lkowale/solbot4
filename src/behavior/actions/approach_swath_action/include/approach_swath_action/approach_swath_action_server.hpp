// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef APPROACH_SWATH_ACTION__APPROACH_SWATH_ACTION_SERVER_HPP_
#define APPROACH_SWATH_ACTION__APPROACH_SWATH_ACTION_SERVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "solbot4_msgs/action/run_approach_swath.hpp"

namespace approach_swath_action
{

class ApproachSwathActionServer : public rclcpp::Node
{
public:
  using ApproachSwathAction = solbot4_msgs::action::RunApproachSwath;
  using GoalHandleApproachSwath = rclcpp_action::ServerGoalHandle<ApproachSwathAction>;

  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit ApproachSwathActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ApproachSwathActionServer() = default;

private:
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ApproachSwathAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleApproachSwath> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleApproachSwath> goal_handle);

  void execute(const std::shared_ptr<GoalHandleApproachSwath> goal_handle);

  // Helper methods
  bool compute_path(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & planner_id,
    nav_msgs::msg::Path & path);

  bool follow_path(
    const nav_msgs::msg::Path & path,
    const std::string & controller_id,
    const std::shared_ptr<GoalHandleApproachSwath> goal_handle);

  // Action server
  rclcpp_action::Server<ApproachSwathAction>::SharedPtr action_server_;

  // Callback group for action clients
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Action clients for Nav2 services
  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

  // Active goal handle for follow_path (for cancellation)
  GoalHandleFollowPath::SharedPtr follow_path_goal_handle_;
  std::mutex goal_handle_mutex_;

  // Safe cancellation helper
  void cancel_follow_path();

  // Path publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::string current_path_pub_topic_;

  // Parameters
  std::string default_planner_id_;
  std::string default_controller_id_;
  std::string default_path_pub_topic_;

  // State
  std::atomic<bool> is_cancelling_{false};
  float distance_remaining_{0.0f};
};

}  // namespace approach_swath_action

#endif  // APPROACH_SWATH_ACTION__APPROACH_SWATH_ACTION_SERVER_HPP_
