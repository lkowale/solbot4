// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GetPoseFromPoses : public BT::SyncActionNode
{
public:
  GetPoseFromPoses(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses"),
      BT::InputPort<int>("index"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    int index = 0;

    if (!getInput("poses", poses)) {
      return BT::NodeStatus::FAILURE;
    }

    if (!getInput("index", index)) {
      return BT::NodeStatus::FAILURE;
    }

    if (index < 0 || index >= static_cast<int>(poses.size())) {
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped pose = poses[index];
    setOutput("pose", pose);

    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetPoseFromPoses>("GetPoseFromPoses");
}
