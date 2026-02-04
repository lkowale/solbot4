// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <vector>
#include <fstream>

#include "one_line_nav/one_line_navigator.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <nlohmann/json.hpp>

namespace one_line_nav
{

bool OneLineNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".fields_directory",
    rclcpp::ParameterValue("/home/aa/ros2_ws4/src/fields"));

  node->get_parameter(getName() + ".fields_directory", fields_directory_);

  return true;
}

std::string OneLineNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node) {
    return default_bt_xml_filename;
  }

  if (!node->has_parameter("default_one_line_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("one_line_nav");
    node->declare_parameter<std::string>(
      "default_one_line_bt_xml",
      pkg_share_dir + "/behavior_trees/one_line_pausable.xml");
  }

  node->get_parameter("default_one_line_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool OneLineNavigator::loadLineFromFile(
  const std::string & field_name,
  geographic_msgs::msg::GeoPoint & start_point,
  geographic_msgs::msg::GeoPoint & end_point)
{
  std::string file_path = fields_directory_ + "/" + field_name + "/line.json";

  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Failed to open line file: %s", file_path.c_str());
    return false;
  }

  try {
    nlohmann::json j;
    file >> j;

    start_point.latitude = j["start"]["latitude"].get<double>();
    start_point.longitude = j["start"]["longitude"].get<double>();
    start_point.altitude = 0.0;

    end_point.latitude = j["end"]["latitude"].get<double>();
    end_point.longitude = j["end"]["longitude"].get<double>();
    end_point.altitude = 0.0;

    RCLCPP_INFO(logger_, "Loaded line from %s", file_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse line file %s: %s", file_path.c_str(), e.what());
    return false;
  }
}

bool OneLineNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  // Use default BT if none specified
  if (bt_xml_filename.empty()) {
    bt_xml_filename = bt_action_server_->getCurrentBTFilename();
  }

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  // Determine geo_points: either from goal or from file
  geographic_msgs::msg::GeoPoint start_point = goal->start_point;
  geographic_msgs::msg::GeoPoint end_point = goal->end_point;

  // Check if coordinates were provided (non-zero lat/lon)
  bool coords_provided =
    (start_point.latitude != 0.0 || start_point.longitude != 0.0) &&
    (end_point.latitude != 0.0 || end_point.longitude != 0.0);

  if (!coords_provided && !goal->field_name.empty()) {
    // Load from file
    if (!loadLineFromFile(goal->field_name, start_point, end_point)) {
      RCLCPP_ERROR(logger_, "Failed to load line from field: %s", goal->field_name.c_str());
      return false;
    }
  } else if (!coords_provided) {
    RCLCPP_ERROR(logger_, "No coordinates provided and no field_name specified");
    return false;
  }

  // Set blackboard variables for BT nodes
  auto blackboard = bt_action_server_->getBlackboard();
  std::vector<geographic_msgs::msg::GeoPoint> geo_points = {start_point, end_point};
  blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", geo_points);
  blackboard->set<std::string>("field_name", goal->field_name);
  blackboard->set<std::string>("fields_directory", fields_directory_);
  blackboard->set<int>("pass_number", 0);
  blackboard->set<std::string>("stage", "approach");

  RCLCPP_INFO(logger_, "OneLineNavigator received goal:");
  RCLCPP_INFO(logger_, "Start Point: [lat: %lf, lon: %lf, alt: %lf]",
    start_point.latitude, start_point.longitude, start_point.altitude);
  RCLCPP_INFO(logger_, "End Point: [lat: %lf, lon: %lf, alt: %lf]",
    end_point.latitude, end_point.longitude, end_point.altitude);

  return true;
}

void OneLineNavigator::onLoop()
{
  // Publish feedback if needed
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  float distance_traveled = 0.0f;
  float total_distance = 0.0f;

  (void)blackboard->get("distance_traveled", distance_traveled);
  (void)blackboard->get("total_distance", total_distance);

  feedback_msg->distance_traveled = distance_traveled;
  feedback_msg->total_distance = total_distance;

  bt_action_server_->publishFeedback(feedback_msg);
}

void OneLineNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  // Re-initialize with new goal
  geographic_msgs::msg::GeoPoint start_point = goal->start_point;
  geographic_msgs::msg::GeoPoint end_point = goal->end_point;

  bool coords_provided =
    (start_point.latitude != 0.0 || start_point.longitude != 0.0) &&
    (end_point.latitude != 0.0 || end_point.longitude != 0.0);

  if (!coords_provided && !goal->field_name.empty()) {
    loadLineFromFile(goal->field_name, start_point, end_point);
  }

  auto blackboard = bt_action_server_->getBlackboard();
  std::vector<geographic_msgs::msg::GeoPoint> geo_points = {start_point, end_point};
  blackboard->set<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points", geo_points);
}

void OneLineNavigator::goalCompleted(
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
    logger_, "OneLineNavigator completed with status: %d",
    static_cast<int>(final_bt_status));
}

}  // namespace one_line_nav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(one_line_nav::OneLineNavigator, nav2_core::NavigatorBase)
