// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef MOVE_SEQ_NAV__MOVE_SEQUENCE_ACTION_HPP_
#define MOVE_SEQ_NAV__MOVE_SEQUENCE_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "solbot4_msgs/action/move_sequence.hpp"

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

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("sequence_file", "long_sequence.json", "Path to sequence file"),
      BT::InputPort<uint16_t>("segment_idx", 0, "Starting segment index"),
      BT::InputPort<float>("segment_distance_traveled", 0.0f, "Distance already traveled in segment"),
      BT::OutputPort<uint16_t>("error_code", "Error code from action result"),
    });
  }
};

}  // namespace move_seq_nav

#endif  // MOVE_SEQ_NAV__MOVE_SEQUENCE_ACTION_HPP_
