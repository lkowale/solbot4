// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>

#include "move_seq_nav/move_seq_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace move_seq_nav
{

bool MoveSeqNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".sequence_file_blackboard_id",
    rclcpp::ParameterValue("sequence_file"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".field_name_blackboard_id",
    rclcpp::ParameterValue("field_name"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".progress_file_blackboard_id",
    rclcpp::ParameterValue("progress_file"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".fields_directory",
    rclcpp::ParameterValue("/home/aa/ros2_ws4/src/fields"));

  node->get_parameter(
    getName() + ".sequence_file_blackboard_id",
    sequence_file_blackboard_id_);
  node->get_parameter(
    getName() + ".field_name_blackboard_id",
    field_name_blackboard_id_);
  node->get_parameter(
    getName() + ".progress_file_blackboard_id",
    progress_file_blackboard_id_);
  node->get_parameter(
    getName() + ".fields_directory",
    fields_directory_);

  return true;
}

std::string MoveSeqNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr node)
{
  auto node_ptr = node.lock();
  std::string default_bt_xml_filename;

  if (!node_ptr) {
    return default_bt_xml_filename;
  }

  // Try to get from parameter first
  nav2_util::declare_parameter_if_not_declared(
    node_ptr, getName() + ".default_bt_xml_filename",
    rclcpp::ParameterValue(
      ament_index_cpp::get_package_share_directory("move_seq_nav") +
      "/behavior_trees/move_seq_loop.xml"));

  node_ptr->get_parameter(
    getName() + ".default_bt_xml_filename",
    default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool MoveSeqNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = bt_action_server_->getCurrentBTFilename();

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "Failed to load behavior tree: %s",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoal(goal);
  return true;
}

void MoveSeqNavigator::initializeGoal(ActionT::Goal::ConstSharedPtr goal)
{
  loop_count_ = 0;
  start_time_ = clock_->now();

  // Set sequence file and field name on blackboard for BT nodes to use
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set(sequence_file_blackboard_id_, goal->sequence_file);
  blackboard->set(field_name_blackboard_id_, goal->field_name);

  // Construct progress file path: /fields_directory/field_name/move_seq_progress.json
  std::string progress_file = fields_directory_ + "/" + goal->field_name + "/move_seq_progress.json";
  blackboard->set(progress_file_blackboard_id_, progress_file);

  RCLCPP_INFO(
    logger_, "Starting MoveSequence loop with sequence file: %s, field: %s, progress: %s",
    goal->sequence_file.c_str(), goal->field_name.c_str(), progress_file.c_str());
}

void MoveSeqNavigator::onLoop()
{
  // Publish feedback
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  // Get current segment and distance from blackboard if available
  auto blackboard = bt_action_server_->getBlackboard();

  feedback_msg->loop_count = loop_count_;

  uint16_t current_segment = 0;
  float distance_traveled = 0.0f;

  if (blackboard->get("current_segment", current_segment)) {
    feedback_msg->current_segment = current_segment;
  }
  if (blackboard->get("distance_traveled", distance_traveled)) {
    feedback_msg->distance_traveled = distance_traveled;
  }

  // Check if we completed a loop (segment 0 and new loop)
  static uint16_t last_segment = 0;
  if (current_segment == 0 && last_segment > 0) {
    loop_count_++;
    RCLCPP_INFO(logger_, "Completed loop %u", loop_count_);
  }
  last_segment = current_segment;

  feedback_msg->loop_count = loop_count_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void MoveSeqNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");
  initializeGoal(goal);
}

void MoveSeqNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  // Set result based on BT status
  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::NONE;
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELLED;
  } else {
    result->error_code = ActionT::Result::FAILED;
  }

  RCLCPP_INFO(
    logger_, "MoveSequence loop completed after %u loops with status: %d",
    loop_count_, static_cast<int>(final_bt_status));
}

}  // namespace move_seq_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(move_seq_nav::MoveSeqNavigator, nav2_core::NavigatorBase)
