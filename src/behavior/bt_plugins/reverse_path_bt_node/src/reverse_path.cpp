// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ReversePath : public BT::SyncActionNode
{
public:
  ReversePath(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("reverse_path_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "The path to reverse"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "The reversed path")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path input_path;
    if (!getInput("input_path", input_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [input_path]");
      return BT::NodeStatus::FAILURE;
    }

    if (input_path.poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Input path is empty");
      setOutput("output_path", input_path);
      return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path output_path;
    output_path.header = input_path.header;
    output_path.header.stamp = node_->now();

    // Reverse the order of poses
    output_path.poses.reserve(input_path.poses.size());
    for (auto it = input_path.poses.rbegin(); it != input_path.poses.rend(); ++it) {
      geometry_msgs::msg::PoseStamped pose = *it;

      // Rotate orientation by 180 degrees (flip direction)
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      tf2::Quaternion rotation;
      rotation.setRPY(0, 0, M_PI);
      q = q * rotation;
      q.normalize();
      pose.pose.orientation = tf2::toMsg(q);

      output_path.poses.push_back(pose);
    }

    setOutput("output_path", output_path);

    RCLCPP_INFO(node_->get_logger(),
      "Reversed path with %zu poses", output_path.poses.size());

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ReversePath>("ReversePath");
}
