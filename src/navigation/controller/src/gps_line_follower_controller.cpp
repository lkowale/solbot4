// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gps_line_follower_controller/gps_line_follower_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gps_line_follower_controller
{

void GpsLineFollowerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".gps_frame_id", rclcpp::ParameterValue("gps_link"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".switch_distance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_cross_track", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ki_cross_track", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kd_cross_track", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_heading", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".switch_hysteresis", rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".gps_frame_id", gps_frame_id_);
  node->get_parameter(plugin_name_ + ".switch_distance", switch_distance_);
  node->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".kp_cross_track", kp_cross_track_);
  node->get_parameter(plugin_name_ + ".ki_cross_track", ki_cross_track_);
  node->get_parameter(plugin_name_ + ".kd_cross_track", kd_cross_track_);
  node->get_parameter(plugin_name_ + ".kp_heading", kp_heading_);
  node->get_parameter(plugin_name_ + ".switch_hysteresis", switch_hysteresis_);

  // Default switch_distance to half wheelbase if not specified
  if (switch_distance_ <= 0.0) {
    switch_distance_ = wheelbase_ / 2.0;
  }

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

  RCLCPP_INFO(logger_,
    "GpsLineFollowerController configured. GPS frame: %s, switch_distance: %.3f m, wheelbase: %.3f m",
    gps_frame_id_.c_str(), switch_distance_, wheelbase_);
}

void GpsLineFollowerController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up GpsLineFollowerController plugin");
  global_path_pub_.reset();
}

void GpsLineFollowerController::activate()
{
  RCLCPP_INFO(logger_, "Activating GpsLineFollowerController plugin");
  global_path_pub_->on_activate();
  pid_initialized_ = false;
  cross_track_error_integral_ = 0.0;
  prev_cross_track_error_ = 0.0;
}

void GpsLineFollowerController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating GpsLineFollowerController plugin");
  global_path_pub_->on_deactivate();
}

void GpsLineFollowerController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  current_path_idx_ = 0;
  pid_initialized_ = false;
  cross_track_error_integral_ = 0.0;
  prev_cross_track_error_ = 0.0;
  current_mode_ = ControlMode::APPROACH;

  global_path_pub_->publish(path);
  RCLCPP_DEBUG(logger_, "Received new plan with %zu poses", path.poses.size());
}

void GpsLineFollowerController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped GpsLineFollowerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "No path received");
    return cmd_vel;
  }

  // Get GPS position in global frame
  geometry_msgs::msg::PoseStamped gps_pose;
  if (!getGpsPose(gps_pose)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "Could not get GPS pose, using robot pose for control");
    gps_pose = pose;
  }

  // Find closest point on path to GPS and calculate distance
  geometry_msgs::msg::Point closest_point;
  size_t closest_idx;
  double gps_to_path_distance = findClosestPointOnPath(
    gps_pose.pose.position, closest_point, closest_idx);

  // Update current path index
  if (closest_idx > current_path_idx_) {
    current_path_idx_ = closest_idx;
  }

  // Determine control mode based on GPS distance to path with hysteresis
  // - Switch to ON_LINE when GPS is closer than switch_distance
  // - Switch back to APPROACH only when GPS is farther than switch_distance + hysteresis
  // This prevents oscillation when GPS is near the threshold
  ControlMode new_mode = current_mode_;
  if (current_mode_ == ControlMode::APPROACH) {
    // Currently approaching - switch to ON_LINE when close enough
    if (gps_to_path_distance <= switch_distance_) {
      new_mode = ControlMode::ON_LINE;
    }
  } else {
    // Currently on line - only switch to APPROACH if significantly off the line
    if (gps_to_path_distance > switch_distance_ + switch_hysteresis_) {
      new_mode = ControlMode::APPROACH;
    }
  }

  if (new_mode != current_mode_) {
    current_mode_ = new_mode;
    if (current_mode_ == ControlMode::ON_LINE) {
      RCLCPP_INFO(logger_, "Switching to ON_LINE mode (GPS %.3f m from path)", gps_to_path_distance);
      // Reset PID when switching modes
      pid_initialized_ = false;
      cross_track_error_integral_ = 0.0;
    } else {
      RCLCPP_INFO(logger_, "Switching to APPROACH mode (GPS %.3f m from path, threshold %.3f m)",
        gps_to_path_distance, switch_distance_ + switch_hysteresis_);
    }
  }

  // Compute velocity based on current mode
  if (current_mode_ == ControlMode::APPROACH) {
    // Find lookahead point
    geometry_msgs::msg::Point lookahead_point;
    if (!getLookaheadPoint(pose.pose.position, lookahead_dist_, lookahead_point)) {
      // Use last path point if no valid lookahead
      lookahead_point = global_plan_.poses.back().pose.position;
    }
    cmd_vel = computeApproachVelocity(pose, lookahead_point);
  } else {
    // ON_LINE mode - use cross-track error control
    double cross_track_error = calculateCrossTrackError(gps_pose.pose.position, closest_idx);
    double path_heading = getPathHeading(closest_idx);
    cmd_vel = computeOnLineVelocity(pose, gps_pose, cross_track_error, path_heading);
  }

  // Apply speed limits
  double max_vel = desired_linear_vel_;
  if (speed_limit_is_percentage_) {
    max_vel *= speed_limit_;
  } else if (speed_limit_ > 0.0) {
    max_vel = std::min(max_vel, speed_limit_);
  }
  cmd_vel.twist.linear.x = std::clamp(cmd_vel.twist.linear.x, -max_linear_vel_, max_vel);
  cmd_vel.twist.angular.z = std::clamp(cmd_vel.twist.angular.z, -max_angular_vel_, max_angular_vel_);

  return cmd_vel;
}

bool GpsLineFollowerController::getGpsPose(geometry_msgs::msg::PoseStamped & gps_pose)
{
  geometry_msgs::msg::PoseStamped gps_origin;
  gps_origin.header.frame_id = gps_frame_id_;
  gps_origin.header.stamp = rclcpp::Time(0);
  gps_origin.pose.orientation.w = 1.0;

  try {
    tf_->transform(gps_origin, gps_pose, global_frame_,
      tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(logger_, "Could not transform GPS frame to global frame: %s", ex.what());
    return false;
  }
}

double GpsLineFollowerController::findClosestPointOnPath(
  const geometry_msgs::msg::Point & position,
  geometry_msgs::msg::Point & closest_point,
  size_t & closest_idx)
{
  double min_dist = std::numeric_limits<double>::max();
  closest_idx = 0;

  // Start search from current index to avoid going backwards
  size_t search_start = current_path_idx_;
  if (search_start >= global_plan_.poses.size()) {
    search_start = 0;
  }

  for (size_t i = search_start; i < global_plan_.poses.size() - 1; ++i) {
    const auto & p1 = global_plan_.poses[i].pose.position;
    const auto & p2 = global_plan_.poses[i + 1].pose.position;

    // Vector from p1 to p2
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double segment_length_sq = dx * dx + dy * dy;

    if (segment_length_sq < 1e-6) {
      continue;  // Skip very short segments
    }

    // Project position onto the line segment
    double t = ((position.x - p1.x) * dx + (position.y - p1.y) * dy) / segment_length_sq;
    t = std::clamp(t, 0.0, 1.0);

    // Closest point on segment
    geometry_msgs::msg::Point proj;
    proj.x = p1.x + t * dx;
    proj.y = p1.y + t * dy;
    proj.z = 0.0;

    // Distance from position to projected point
    double dist = std::hypot(position.x - proj.x, position.y - proj.y);

    if (dist < min_dist) {
      min_dist = dist;
      closest_point = proj;
      closest_idx = i;
    }
  }

  // Also check distance to last point
  if (!global_plan_.poses.empty()) {
    const auto & last_point = global_plan_.poses.back().pose.position;
    double dist_to_last = std::hypot(position.x - last_point.x, position.y - last_point.y);
    if (dist_to_last < min_dist) {
      min_dist = dist_to_last;
      closest_point = last_point;
      closest_idx = global_plan_.poses.size() - 1;
    }
  }

  return min_dist;
}

double GpsLineFollowerController::calculateCrossTrackError(
  const geometry_msgs::msg::Point & position,
  size_t path_idx)
{
  if (path_idx >= global_plan_.poses.size() - 1) {
    path_idx = global_plan_.poses.size() - 2;
  }

  const auto & p1 = global_plan_.poses[path_idx].pose.position;
  const auto & p2 = global_plan_.poses[path_idx + 1].pose.position;

  // Vector from p1 to p2 (path direction)
  double path_dx = p2.x - p1.x;
  double path_dy = p2.y - p1.y;
  double path_length = std::hypot(path_dx, path_dy);

  if (path_length < 1e-6) {
    return 0.0;
  }

  // Unit normal vector (perpendicular to path, pointing left)
  double nx = -path_dy / path_length;
  double ny = path_dx / path_length;

  // Vector from p1 to position
  double px = position.x - p1.x;
  double py = position.y - p1.y;

  // Signed cross-track error: positive = left of path, negative = right
  double cross_track = px * nx + py * ny;

  return cross_track;
}

double GpsLineFollowerController::getPathHeading(size_t path_idx)
{
  if (path_idx >= global_plan_.poses.size() - 1) {
    path_idx = global_plan_.poses.size() - 2;
  }

  const auto & p1 = global_plan_.poses[path_idx].pose.position;
  const auto & p2 = global_plan_.poses[path_idx + 1].pose.position;

  return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

geometry_msgs::msg::TwistStamped GpsLineFollowerController::computeApproachVelocity(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Point & lookahead_point)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = robot_pose.header;
  cmd_vel.header.stamp = clock_->now();

  // Get robot heading
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Calculate angle to lookahead point
  double dx = lookahead_point.x - robot_pose.pose.position.x;
  double dy = lookahead_point.y - robot_pose.pose.position.y;
  double dist_to_lookahead = std::hypot(dx, dy);

  if (dist_to_lookahead < 0.01) {
    // Very close to lookahead, minimal movement
    cmd_vel.twist.linear.x = min_linear_vel_;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  double angle_to_lookahead = std::atan2(dy, dx);
  double heading_error = normalizeAngle(angle_to_lookahead - robot_yaw);

  // Pure pursuit: curvature = 2 * sin(heading_error) / lookahead_distance
  double curvature = 2.0 * std::sin(heading_error) / dist_to_lookahead;

  // For Ackermann steering, angular velocity = linear_velocity * curvature
  // But we limit it to max_angular_vel
  cmd_vel.twist.linear.x = desired_linear_vel_;

  // Scale velocity based on curvature (slow down for tight turns)
  double curvature_factor = std::max(0.3, 1.0 - std::abs(curvature) * wheelbase_);
  cmd_vel.twist.linear.x *= curvature_factor;
  cmd_vel.twist.linear.x = std::max(cmd_vel.twist.linear.x, min_linear_vel_);

  cmd_vel.twist.angular.z = cmd_vel.twist.linear.x * curvature;

  return cmd_vel;
}

geometry_msgs::msg::TwistStamped GpsLineFollowerController::computeOnLineVelocity(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & /*gps_pose*/,
  double cross_track_error,
  double path_heading)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = robot_pose.header;
  cmd_vel.header.stamp = clock_->now();

  rclcpp::Time current_time = clock_->now();

  // Initialize PID timing
  if (!pid_initialized_) {
    prev_time_ = current_time;
    prev_cross_track_error_ = cross_track_error;
    pid_initialized_ = true;
  }

  double dt = (current_time - prev_time_).seconds();
  if (dt < 1e-6) {
    dt = 0.05;  // Default to 20Hz if time difference is too small
  }

  // PID control for cross-track error
  double p_term = kp_cross_track_ * cross_track_error;

  cross_track_error_integral_ += cross_track_error * dt;
  // Anti-windup: limit integral
  cross_track_error_integral_ = std::clamp(cross_track_error_integral_, -1.0, 1.0);
  double i_term = ki_cross_track_ * cross_track_error_integral_;

  double d_term = 0.0;
  if (dt > 0) {
    d_term = kd_cross_track_ * (cross_track_error - prev_cross_track_error_) / dt;
  }

  // Steering correction from cross-track error
  // Negate because: positive error (left of path) needs negative angular vel (steer right)
  double cross_track_correction = -(p_term + i_term + d_term);

  // Heading error correction
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double heading_error = normalizeAngle(path_heading - robot_yaw);
  double heading_correction = kp_heading_ * heading_error;

  // Combined angular velocity
  // Cross-track correction steers toward the line, heading correction aligns with the path
  double angular_vel = cross_track_correction + heading_correction;

  // Set velocities
  cmd_vel.twist.linear.x = desired_linear_vel_;
  cmd_vel.twist.angular.z = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  // Update state for next iteration
  prev_time_ = current_time;
  prev_cross_track_error_ = cross_track_error;

  RCLCPP_DEBUG(logger_,
    "ON_LINE: cross_track=%.3f, heading_err=%.3f, ang_vel=%.3f",
    cross_track_error, heading_error, cmd_vel.twist.angular.z);

  return cmd_vel;
}

bool GpsLineFollowerController::getLookaheadPoint(
  const geometry_msgs::msg::Point & robot_position,
  double lookahead_dist,
  geometry_msgs::msg::Point & lookahead_point)
{
  if (global_plan_.poses.size() < 2) {
    return false;
  }

  // Find the first point on path that is at least lookahead_dist away
  for (size_t i = current_path_idx_; i < global_plan_.poses.size(); ++i) {
    const auto & point = global_plan_.poses[i].pose.position;
    double dist = std::hypot(point.x - robot_position.x, point.y - robot_position.y);

    if (dist >= lookahead_dist) {
      lookahead_point = point;
      return true;
    }
  }

  // If no point is far enough, return the last point
  lookahead_point = global_plan_.poses.back().pose.position;
  return true;
}

double GpsLineFollowerController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace gps_line_follower_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gps_line_follower_controller::GpsLineFollowerController,
  nav2_core::Controller)
