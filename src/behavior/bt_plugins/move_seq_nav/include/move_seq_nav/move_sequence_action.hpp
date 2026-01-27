// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef MOVE_SEQ_NAV__MOVE_SEQUENCE_ACTION_HPP_
#define MOVE_SEQ_NAV__MOVE_SEQUENCE_ACTION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "solbot4_msgs/action/move_sequence.hpp"
#include "rclcpp/rclcpp.hpp"

namespace move_seq_nav
{

class MoveSequenceAction : public nav2_behavior_tree::BtActionNode<solbot4_msgs::action::MoveSequence>
{
public:
  MoveSequenceAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  void on_wait_for_result(
    std::shared_ptr<const solbot4_msgs::action::MoveSequence::Feedback> feedback) override;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("sequence_file", "long_sequence.json", "Path to sequence file"),
      BT::InputPort<uint16_t>("segment_idx", 0, "Starting segment index"),
      BT::InputPort<float>("segment_distance_traveled", 0.0f, "Distance already traveled in segment"),
      BT::InputPort<std::string>("progress_file", "/tmp/move_sequence_progress.json", "File to save progress"),
      BT::OutputPort<uint16_t>("error_code", "Error code from action result"),
    });
  }

private:
  void saveProgress();

  std::string sequence_file_;
  std::string progress_file_;
  uint16_t last_segment_idx_{0};
  float last_distance_traveled_{0.0f};
};

}  // namespace move_seq_nav

#endif  // MOVE_SEQ_NAV__MOVE_SEQUENCE_ACTION_HPP_
