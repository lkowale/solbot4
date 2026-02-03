// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef SWATH_NAV__SWATH_NAVIGATOR_HPP_
#define SWATH_NAV__SWATH_NAVIGATOR_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "solbot4_msgs/action/run_swath.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

namespace swath_nav
{

class SwathNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot4_msgs::action::RunSwath>
{
public:
  using ActionT = solbot4_msgs::action::RunSwath;

  SwathNavigator() = default;
  ~SwathNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("run_swath");}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  std::string default_planner_id_;
  std::string default_controller_id_;
  std::string default_path_pub_topic_;
};

}  // namespace swath_nav

#endif  // SWATH_NAV__SWATH_NAVIGATOR_HPP_
