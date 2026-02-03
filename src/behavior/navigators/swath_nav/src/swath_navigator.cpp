// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>

#include "swath_nav/swath_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace swath_nav
{

bool SwathNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".default_planner_id",
    rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".default_controller_id",
    rclcpp::ParameterValue("FollowPath"));
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".default_path_pub_topic",
    rclcpp::ParameterValue("swath_path"));

  node->get_parameter(getName() + ".default_planner_id", default_planner_id_);
  node->get_parameter(getName() + ".default_controller_id", default_controller_id_);
  node->get_parameter(getName() + ".default_path_pub_topic", default_path_pub_topic_);

  return true;
}

std::string SwathNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node) {
    return default_bt_xml_filename;
  }

  if (!node->has_parameter("default_swath_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("swath_nav");
    node->declare_parameter<std::string>(
      "default_swath_bt_xml",
      pkg_share_dir + "/behavior_trees/swath_nav.xml");
  }

  node->get_parameter("default_swath_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool SwathNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (bt_xml_filename.empty()) {
    bt_xml_filename = bt_action_server_->getCurrentBTFilename();
  }

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  if (goal->geo_points.empty()) {
    RCLCPP_ERROR(logger_, "No geo_points provided");
    return false;
  }

  auto blackboard = bt_action_server_->getBlackboard();

  blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", goal->geo_points);

  std::string planner_id = goal->planner_id.empty() ? default_planner_id_ : goal->planner_id;
  std::string controller_id = goal->controller_id.empty() ? default_controller_id_ : goal->controller_id;
  std::string path_pub_topic = goal->path_pub_topic.empty() ? default_path_pub_topic_ : goal->path_pub_topic;

  blackboard->set<std::string>("planner_id", planner_id);
  blackboard->set<std::string>("controller_id", controller_id);
  blackboard->set<std::string>("path_pub_topic", path_pub_topic);

  RCLCPP_INFO(logger_, "SwathNavigator received goal with %zu geo_points", goal->geo_points.size());
  RCLCPP_INFO(logger_, "Using planner_id: %s, controller_id: %s, path_pub_topic: %s",
    planner_id.c_str(), controller_id.c_str(), path_pub_topic.c_str());

  return true;
}

void SwathNavigator::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  float distance_traveled = 0.0f;
  float total_distance = 0.0f;
  uint32_t current_waypoint = 0;

  (void)blackboard->get("distance_traveled", distance_traveled);
  (void)blackboard->get("total_distance", total_distance);
  (void)blackboard->get("current_waypoint", current_waypoint);

  feedback_msg->distance_traveled = distance_traveled;
  feedback_msg->total_distance = total_distance;
  feedback_msg->current_waypoint = current_waypoint;

  bt_action_server_->publishFeedback(feedback_msg);
}

void SwathNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  auto blackboard = bt_action_server_->getBlackboard();

  if (!goal->geo_points.empty()) {
    blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", goal->geo_points);
  }

  std::string planner_id = goal->planner_id.empty() ? default_planner_id_ : goal->planner_id;
  std::string controller_id = goal->controller_id.empty() ? default_controller_id_ : goal->controller_id;
  std::string path_pub_topic = goal->path_pub_topic.empty() ? default_path_pub_topic_ : goal->path_pub_topic;

  blackboard->set<std::string>("planner_id", planner_id);
  blackboard->set<std::string>("controller_id", controller_id);
  blackboard->set<std::string>("path_pub_topic", path_pub_topic);
}

void SwathNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = ActionT::Result::NONE;
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::CANCELED) {
    result->error_code = ActionT::Result::CANCELLED;
  } else {
    result->error_code = ActionT::Result::FAILED;
  }

  RCLCPP_INFO(
    logger_, "SwathNavigator completed with status: %d",
    static_cast<int>(final_bt_status));
}

}  // namespace swath_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(swath_nav::SwathNavigator, nav2_core::NavigatorBase)
