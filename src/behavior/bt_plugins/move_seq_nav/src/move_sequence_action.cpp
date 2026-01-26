// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>

#include "move_seq_nav/move_sequence_action.hpp"

namespace move_seq_nav
{

MoveSequenceAction::MoveSequenceAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<solbot4_msgs::action::MoveSequence>(xml_tag_name, action_name, conf)
{
}

void MoveSequenceAction::on_tick()
{
  getInput("sequence_file", goal_.sequence_file);
  getInput("segment_idx", goal_.segment_idx);
  getInput("segment_distance_traveled", goal_.segment_distance_traveled);
}

BT::NodeStatus MoveSequenceAction::on_success()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveSequenceAction::on_aborted()
{
  setOutput("error_code", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveSequenceAction::on_cancelled()
{
  setOutput("error_code", static_cast<uint16_t>(0));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace move_seq_nav

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<move_seq_nav::MoveSequenceAction>(
        name, "move_sequence", config);
    };

  factory.registerBuilder<move_seq_nav::MoveSequenceAction>("MoveSequence", builder);
}
