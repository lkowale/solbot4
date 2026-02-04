// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ReversePoses : public BT::SyncActionNode
{
public:
  ReversePoses(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("reverse_poses_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "poses", "The poses to reverse (in-place)")
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    if (!getInput("poses", poses)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [poses]");
      return BT::NodeStatus::FAILURE;
    }

    if (poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Input poses vector is empty");
      return BT::NodeStatus::SUCCESS;
    }

    std::vector<geometry_msgs::msg::PoseStamped> reversed_poses;
    reversed_poses.reserve(poses.size());

    // Reverse the order of poses and flip orientations by 180 degrees
    for (auto it = poses.rbegin(); it != poses.rend(); ++it) {
      geometry_msgs::msg::PoseStamped pose = *it;

      // Rotate orientation by 180 degrees (flip direction)
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      tf2::Quaternion rotation;
      rotation.setRPY(0, 0, M_PI);
      q = q * rotation;
      q.normalize();
      pose.pose.orientation = tf2::toMsg(q);

      reversed_poses.push_back(pose);
    }

    setOutput("poses", reversed_poses);

    RCLCPP_INFO(node_->get_logger(),
      "Reversed %zu poses", reversed_poses.size());

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ReversePoses>("ReversePoses");
}
