// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef RUN_SWATH_ACTION__RUN_SWATH_ACTION_SERVER_HPP_
#define RUN_SWATH_ACTION__RUN_SWATH_ACTION_SERVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "solbot4_msgs/action/run_swath.hpp"

namespace run_swath_action
{

class RunSwathActionServer : public rclcpp::Node
{
public:
  using RunSwathAction = solbot4_msgs::action::RunSwath;
  using GoalHandleRunSwath = rclcpp_action::ServerGoalHandle<RunSwathAction>;

  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;

  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit RunSwathActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~RunSwathActionServer() = default;

private:
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RunSwathAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRunSwath> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleRunSwath> goal_handle);

  void execute(const std::shared_ptr<GoalHandleRunSwath> goal_handle);

  // Helper methods
  bool compute_path(
    const std::vector<geometry_msgs::msg::PoseStamped> & goals,
    const std::string & planner_id,
    nav_msgs::msg::Path & path);

  bool follow_path(
    const nav_msgs::msg::Path & path,
    const std::string & controller_id,
    const std::shared_ptr<GoalHandleRunSwath> goal_handle);

  double calculate_path_length(const nav_msgs::msg::Path & path);

  // Action server
  rclcpp_action::Server<RunSwathAction>::SharedPtr action_server_;

  // Callback group for clients
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Action clients for Nav2 services
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr compute_path_client_;
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
  float total_distance_{0.0f};
  float distance_traveled_{0.0f};
  uint32_t current_waypoint_{0};
};

}  // namespace run_swath_action

#endif  // RUN_SWATH_ACTION__RUN_SWATH_ACTION_SERVER_HPP_
