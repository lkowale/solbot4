// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <filesystem>

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

  action_completed_ = false;

  // If a progress file exists, resume from saved position
  if (loadProgress()) {
    goal_.segment_idx = last_segment_idx_;
    goal_.segment_distance_traveled = last_distance_traveled_;
  }

  sequence_file_ = goal_.sequence_file;
  last_segment_idx_ = goal_.segment_idx;
  last_distance_traveled_ = goal_.segment_distance_traveled;
}

BT::NodeStatus MoveSequenceAction::on_success()
{
  setOutput("error_code", result_.result->error_code);
  action_completed_ = true;
  // Clear progress file on successful completion
  std::remove(progress_file_.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveSequenceAction::on_aborted()
{
  setOutput("error_code", result_.result->error_code);
  action_completed_ = true;
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
  // Only save progress if the action didn't already complete
  // (halt is also called during tree cleanup after success/abort)
  if (!action_completed_) {
    saveProgress();
  }
  BtActionNode::halt();
}

void MoveSequenceAction::on_wait_for_result(
  std::shared_ptr<const solbot4_msgs::action::MoveSequence::Feedback> feedback)
{
  if (feedback) {
    last_segment_idx_ = feedback->current_segment;
    last_distance_traveled_ = feedback->distance_traveled;
  }
}

void MoveSequenceAction::saveProgress()
{
  std::ofstream file(progress_file_);
  if (file.is_open()) {
    file << "{\n";
    file << "  \"sequence_file\": \"" << sequence_file_ << "\",\n";
    file << "  \"segment_idx\": " << last_segment_idx_ << ",\n";
    file << "  \"distance_traveled\": " << last_distance_traveled_ << "\n";
    file << "}\n";
    file.close();
  }
}

bool MoveSequenceAction::loadProgress()
{
  if (!std::filesystem::exists(progress_file_)) {
    return false;
  }

  std::ifstream file(progress_file_);
  if (!file.is_open()) {
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string content = buffer.str();

  // Parse segment_idx
  auto idx_pos = content.find("\"segment_idx\":");
  if (idx_pos == std::string::npos) {
    return false;
  }
  idx_pos = content.find(':', idx_pos) + 1;
  last_segment_idx_ = static_cast<uint16_t>(std::stoi(content.substr(idx_pos)));

  // Parse distance_traveled
  auto dist_pos = content.find("\"distance_traveled\":");
  if (dist_pos == std::string::npos) {
    return false;
  }
  dist_pos = content.find(':', dist_pos) + 1;
  last_distance_traveled_ = std::stof(content.substr(dist_pos));

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(),
    "Loaded progress: segment_idx=%u, distance_traveled=%.2f",
    last_segment_idx_, last_distance_traveled_);

  return true;
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
