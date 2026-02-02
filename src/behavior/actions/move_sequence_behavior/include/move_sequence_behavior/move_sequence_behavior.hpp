// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOVE_SEQUENCE_BEHAVIOR_HPP_
#define MOVE_SEQUENCE_BEHAVIOR_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <nlohmann/json.hpp>
#include <fstream>

#include "nav2_behaviors/timed_behavior.hpp"
#include "solbot4_msgs/action/move_sequence.hpp"
#include "solbot4_msgs/msg/move_segment.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace move_sequence_behavior
{

using nav2_behaviors::TimedBehavior;
using nav2_behaviors::ResultStatus;
using nav2_behaviors::Status;
using ActionT = solbot4_msgs::action::MoveSequence;

class MoveSequence : public nav2_behaviors::TimedBehavior<ActionT>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  /**
   * @brief A constructor for nav2_behaviors::DriveOnHeading
   */
  MoveSequence()
  : nav2_behaviors::TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>())

  {
  }

  ~MoveSequence() = default;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
   ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    sequence_file_ = command->sequence_file;
    segment_idx_ = command->segment_idx;
    segment_distance_traveled_ = command->segment_distance_traveled;

    current_segment_ = segment_idx_;
    move_segments_ = loadMoveSegmentsFromJson(sequence_file_);
    initial_pose_set_ = false;

    return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
  }

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override
  {
    // Iterate through each segment in move_segments_

      const auto & seg = move_segments_[current_segment_];

      // Set velocities and distance from segment
      double cmd_vel_lin_x = seg.cmd_vel_lin_x;
      double cmd_vel_ang_z = seg.cmd_vel_ang_z;
      double distance = seg.distance - segment_distance_traveled_;

      rclcpp::Duration time_allowance(seg.time_allowance);

      // If this is the first cycle for this segment, store initial pose and end time
      if (!initial_pose_set_) {
        if (!nav2_util::getCurrentPose(
              initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
              this->transform_tolerance_))
        {
          RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
          return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
        }
        end_time_ = this->clock_->now() + time_allowance;
        initial_pose_set_ = true;
        RCLCPP_INFO(
          this->logger_, "Starting MoveSequence segment %d: "
          "cmd_vel_lin_x=%.2f, cmd_vel_ang_z=%.2f, distance=%.2f, time_allowance=%.2f",
          current_segment_, cmd_vel_lin_x, cmd_vel_ang_z, distance, time_allowance.seconds());
      }

      rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
      if (time_remaining.seconds() < 0.0 && time_allowance.seconds() > 0.0) {
        this->stopRobot();
        RCLCPP_WARN(
          this->logger_,
          "Exceeded time allowance before reaching the MoveSequence segment goal - Exiting MoveSequence");
        return ResultStatus{Status::FAILED, ActionT::Result::TIMEOUT};
      }

      geometry_msgs::msg::PoseStamped current_pose;
      if (!nav2_util::getCurrentPose(
            current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
            this->transform_tolerance_))
      {
        RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
        return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR};
      }

      double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
      double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
      double distance_traveled = hypot(diff_x, diff_y);

      feedback_->current_segment = current_segment_;
      feedback_->distance_traveled = distance_traveled;
      this->action_server_->publish_feedback(feedback_);

      // distance traveled for current segment
      if (distance_traveled >= std::fabs(distance)) 
      {
        initial_pose_set_ = false;
        // if all the segments are done, finish
        if( current_segment_ >= move_segments_.size() - 1) {
          this->stopRobot();
          return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE};
        }
        else {
          // Move to next segment
          current_segment_++;
          segment_distance_traveled_ = 0.0;
        }
      }

      auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
      cmd_vel->header.stamp = this->clock_->now();
      cmd_vel->header.frame_id = this->robot_base_frame_;
      cmd_vel->twist.linear.y = 0.0;
      cmd_vel->twist.angular.z = cmd_vel_ang_z;
      cmd_vel->twist.linear.x = cmd_vel_lin_x;

      this->vel_pub_->publish(std::move(cmd_vel));

      return ResultStatus{Status::RUNNING, ActionT::Result::NONE};
  }

  std::vector<solbot4_msgs::msg::MoveSegment> loadMoveSegmentsFromJson(const std::string & file_name)
  {
    std::vector<solbot4_msgs::msg::MoveSegment> segments;
    // Use configured directory or absolute path if provided
    std::string full_path;
    if (file_name[0] == '/') {
      full_path = file_name;  // Absolute path provided
    } else if (!sequence_files_directory_.empty()) {
      full_path = sequence_files_directory_ + "/" + file_name;
    } else {
      full_path = file_name;  // Relative to current working directory
    }
    std::ifstream file(full_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->logger_, "Could not open file: %s", full_path.c_str());
      return segments;
    }

    nlohmann::json j;
    file >> j;

    for (const auto & item : j) {
      solbot4_msgs::msg::MoveSegment seg;
      seg.cmd_vel_lin_x = item.value("cmd_vel_lin_x", 0.0);
      seg.cmd_vel_ang_z = item.value("cmd_vel_ang_z", 0.0);
      seg.distance = item.value("distance", 0.0);
  double time_allowance_sec = item.value("time_allowance", 0.0);
  seg.time_allowance.sec = static_cast<int32_t>(time_allowance_sec);
  seg.time_allowance.nanosec = static_cast<uint32_t>(0);

      segments.push_back(seg);
    }
    return segments;
  }
  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}
  std::vector<solbot4_msgs::msg::MoveSegment> move_segments_;
protected:


  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "sequence_files_directory", rclcpp::ParameterValue(std::string("")));
    node->get_parameter("sequence_files_directory", sequence_files_directory_);
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  std::string sequence_file_;
  std::string sequence_files_directory_;
  uint16_t segment_idx_,current_segment_;
  float segment_distance_traveled_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  bool initial_pose_set_ = false;
  rclcpp::Time end_time_;
};

}  // namespace move_sequence_behavior

#endif  // MOVE_SEQUENCE_BEHAVIOR_HPP_