// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef GPS_LINE_FOLLOWER_CONTROLLER__GPS_LINE_FOLLOWER_CONTROLLER_HPP_
#define GPS_LINE_FOLLOWER_CONTROLLER__GPS_LINE_FOLLOWER_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace gps_line_follower_controller
{

/**
 * @class GpsLineFollowerController
 * @brief Dual-mode controller for precision agriculture line following.
 *
 * Mode 1 (Approach): When GPS is far from the path, uses pure pursuit (RPP-like)
 *                    behavior to approach the line.
 * Mode 2 (On-Line):  When GPS is within switch_distance of the path, uses
 *                    cross-track error control to keep the GPS antenna on the line.
 *
 * The GPS frame is configurable (default: gps_link). The controller calculates
 * the perpendicular distance from the GPS position to the closest path segment.
 */
class GpsLineFollowerController : public nav2_core::Controller
{
public:
  GpsLineFollowerController() = default;
  ~GpsLineFollowerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Get the GPS position in the global frame
   * @param gps_pose Output GPS pose in global frame
   * @return True if transform succeeded
   */
  bool getGpsPose(geometry_msgs::msg::PoseStamped & gps_pose);

  /**
   * @brief Find the closest point on the path to a given position
   * @param position The position to find closest point for
   * @param closest_point Output closest point on path
   * @param closest_idx Output index of the closest path segment start
   * @return The perpendicular distance to the path
   */
  double findClosestPointOnPath(
    const geometry_msgs::msg::Point & position,
    geometry_msgs::msg::Point & closest_point,
    size_t & closest_idx);

  /**
   * @brief Calculate cross-track error (signed distance to path)
   * @param position Position to calculate error for
   * @param path_idx Index of path segment
   * @return Signed cross-track error (positive = left of path, negative = right)
   */
  double calculateCrossTrackError(
    const geometry_msgs::msg::Point & position,
    size_t path_idx);

  /**
   * @brief Get heading angle of path at given index
   * @param path_idx Index of path segment
   * @return Heading angle in radians
   */
  double getPathHeading(size_t path_idx);

  /**
   * @brief Compute velocity using pure pursuit approach (for approach mode)
   * @param robot_pose Current robot pose
   * @param lookahead_point The lookahead point on path
   * @return Velocity command
   */
  geometry_msgs::msg::TwistStamped computeApproachVelocity(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Point & lookahead_point);

  /**
   * @brief Compute velocity using cross-track error control (for on-line mode)
   * @param robot_pose Current robot pose
   * @param gps_pose GPS position in global frame
   * @param cross_track_error Signed cross-track error
   * @param path_heading Path heading at current segment
   * @return Velocity command
   */
  geometry_msgs::msg::TwistStamped computeOnLineVelocity(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & gps_pose,
    double cross_track_error,
    double path_heading);

  /**
   * @brief Find a lookahead point on the path
   * @param robot_position Current robot position
   * @param lookahead_dist Lookahead distance
   * @param lookahead_point Output lookahead point
   * @return True if valid lookahead point found
   */
  bool getLookaheadPoint(
    const geometry_msgs::msg::Point & robot_position,
    double lookahead_dist,
    geometry_msgs::msg::Point & lookahead_point);

  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalizeAngle(double angle);

  // Node and TF
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("GpsLineFollowerController")};
  rclcpp::Clock::SharedPtr clock_;

  // Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  // Path
  nav_msgs::msg::Path global_plan_;
  size_t current_path_idx_{0};

  // Parameters
  std::string gps_frame_id_;       // TF frame of GPS antenna
  double switch_distance_;          // Distance at which to switch to on-line mode
  double switch_hysteresis_;        // Hysteresis to prevent oscillation near threshold
  double wheelbase_;                // Robot wheelbase (for Ackermann)
  double desired_linear_vel_;       // Target linear velocity
  double max_linear_vel_;           // Maximum linear velocity
  double min_linear_vel_;           // Minimum linear velocity
  double max_angular_vel_;          // Maximum angular velocity
  double lookahead_dist_;           // Lookahead distance for approach mode
  double min_lookahead_dist_;       // Minimum lookahead distance
  double max_lookahead_dist_;       // Maximum lookahead distance
  double transform_tolerance_;      // TF transform timeout

  // Cross-track error PID gains
  double kp_cross_track_;           // Proportional gain for cross-track error
  double ki_cross_track_;           // Integral gain for cross-track error
  double kd_cross_track_;           // Derivative gain for cross-track error

  // Heading error gains
  double kp_heading_;               // Proportional gain for heading error

  // PID state
  double cross_track_error_integral_{0.0};
  double prev_cross_track_error_{0.0};
  rclcpp::Time prev_time_;
  bool pid_initialized_{false};

  // Speed limit
  double speed_limit_{1.0};
  bool speed_limit_is_percentage_{false};

  // Mode tracking
  enum class ControlMode { APPROACH, ON_LINE };
  ControlMode current_mode_{ControlMode::APPROACH};
};

}  // namespace gps_line_follower_controller

#endif  // GPS_LINE_FOLLOWER_CONTROLLER__GPS_LINE_FOLLOWER_CONTROLLER_HPP_
