// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"

class PublishPath : public BT::SyncActionNode
{
public:
  PublishPath(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("publish_path_bt_node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "/path", "Topic name to publish the path"),
      BT::InputPort<nav_msgs::msg::Path>("path", "The path to publish")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string topic_name;
    if (!getInput("topic_name", topic_name)) {
      topic_name = "/path";
    }

    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [path]");
      return BT::NodeStatus::FAILURE;
    }

    // Get or create publisher for this topic
    auto publisher = getOrCreatePublisher(topic_name);

    // Update timestamp
    path.header.stamp = node_->now();

    publisher->publish(path);

    RCLCPP_INFO(node_->get_logger(),
      "Published path with %zu poses to topic '%s'",
      path.poses.size(), topic_name.c_str());

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr getOrCreatePublisher(
    const std::string & topic_name)
  {
    auto it = publishers_.find(topic_name);
    if (it != publishers_.end()) {
      return it->second;
    }

    auto publisher = node_->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
    publishers_[topic_name] = publisher;
    return publisher;
  }

  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> publishers_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<PublishPath>("PublishPath");
}
