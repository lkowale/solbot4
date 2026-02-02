// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "robot_localization/srv/from_ll.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class ConvertGeoPoints : public BT::SyncActionNode
{
public:
  ConvertGeoPoints(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("convert_geo_points_bt_node");
    client_ = node_->create_client<robot_localization::srv::FromLL>("fromLL");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geographic_msgs::msg::GeoPoint>>("geo_points"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("map_points")
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geographic_msgs::msg::GeoPoint> geo_points;
    if (!getInput("geo_points", geo_points)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input [geo_points]");
      return BT::NodeStatus::FAILURE;
    }

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Service fromLL not available");
      return BT::NodeStatus::FAILURE;
    }

    std::vector<geometry_msgs::msg::PoseStamped> map_points;

    for (auto & gp : geo_points) {
      auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
      req->ll_point = gp;

      auto future = client_->async_send_request(req);
      auto result = rclcpp::spin_until_future_complete(
        node_, future, std::chrono::seconds(2));

      if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call fromLL");
        return BT::NodeStatus::FAILURE;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = node_->now();
      pose.header.frame_id = "map";
      pose.pose.position = future.get()->map_point;
      pose.pose.orientation.w = 1.0;

      map_points.push_back(pose);

      RCLCPP_INFO(node_->get_logger(),
        "Converted geo point (%.6f, %.6f) -> map point (%.3f, %.3f)",
        gp.latitude, gp.longitude,
        pose.pose.position.x, pose.pose.position.y);
    }

    setOutput("map_points", map_points);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr client_;
};

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ConvertGeoPoints>("ConvertGeoPoints");
}
