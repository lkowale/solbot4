// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef MOVE_SEQ_NAV__MOVE_SEQ_NAVIGATOR_HPP_
#define MOVE_SEQ_NAV__MOVE_SEQ_NAVIGATOR_HPP_

#include <string>
#include <memory>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "solbot4_msgs/action/run_move_sequence_loop.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace move_seq_nav
{

class MoveSeqNavigator
  : public nav2_core::BehaviorTreeNavigator<solbot4_msgs::action::RunMoveSequenceLoop>
{
public:
  using ActionT = solbot4_msgs::action::RunMoveSequenceLoop;

  MoveSeqNavigator() = default;
  ~MoveSeqNavigator() override = default;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("run_move_sequence_loop");}

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  void onLoop() override;

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoal(ActionT::Goal::ConstSharedPtr goal);

  std::string sequence_file_blackboard_id_;
  std::string field_name_blackboard_id_;
  std::string progress_file_blackboard_id_;
  std::string fields_directory_;
  uint32_t loop_count_;
  rclcpp::Time start_time_;
};

}  // namespace move_seq_nav

#endif  // MOVE_SEQ_NAV__MOVE_SEQ_NAVIGATOR_HPP_
