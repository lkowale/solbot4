// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <fstream>

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
  getInput("progress_file", progress_file_);

  sequence_file_ = goal_.sequence_file;
  last_segment_idx_ = goal_.segment_idx;
  last_distance_traveled_ = goal_.segment_distance_traveled;
}

BT::NodeStatus MoveSequenceAction::on_success()
{
  setOutput("error_code", result_.result->error_code);
  // Clear progress file on successful completion
  std::remove(progress_file_.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveSequenceAction::on_aborted()
{
  setOutput("error_code", result_.result->error_code);
  // Save progress on abort so we can resume later
  saveProgress();
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveSequenceAction::on_cancelled()
{
  setOutput("error_code", static_cast<uint16_t>(0));
  // Save progress on cancel so we can resume later
  saveProgress();
  return BT::NodeStatus::SUCCESS;
}

void MoveSequenceAction::halt()
{
  // Save progress when halted
  saveProgress();
  BtActionNode::halt();
}

void MoveSequenceAction::feedbackCallback(
  typename rclcpp_action::ClientGoalHandle<solbot4_msgs::action::MoveSequence>::SharedPtr,
  const std::shared_ptr<const solbot4_msgs::action::MoveSequence::Feedback> feedback)
{
  // Store latest progress from feedback
  last_segment_idx_ = feedback->current_segment;
  last_distance_traveled_ = feedback->distance_traveled;
}

void MoveSequenceAction::saveProgress()
{
  std::ofstream file(progress_file_);
  if (file.is_open()) {
    file << "{\n";
    file << "  \"sequence_file\": \"" << sequence_file_ << "\",\n";
    file << "  \"segment_idx\": " << last_segment_idx_.load() << ",\n";
    file << "  \"distance_traveled\": " << last_distance_traveled_.load() << "\n";
    file << "}\n";
    file.close();
  }
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
