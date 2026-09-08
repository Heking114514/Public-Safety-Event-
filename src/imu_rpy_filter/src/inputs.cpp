#include "imu_rpy_filter/imu_rpy_filter_node.hpp"

namespace imu_rpy_filter {

std::size_t
ImuRpyFilterNode::declare_positive_integer(const std::string &name,
                                           std::int64_t default_value) {
  const std::int64_t value =
      declare_parameter<std::int64_t>(name, default_value);
  if (value < 1) {
    throw std::invalid_argument(name + " must be at least 1");
  }
  return static_cast<std::size_t>(value);
}

double ImuRpyFilterNode::declare_positive(const std::string &name,
                                          double default_value) {
  const double value = declare_parameter<double>(name, default_value);
  if (value <= 0.0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return value;
}

double ImuRpyFilterNode::declare_nonnegative(const std::string &name,
                                             double default_value) {
  const double value = declare_parameter<double>(name, default_value);
  if (value < 0.0) {
    throw std::invalid_argument(name + " cannot be negative");
  }
  return value;
}

void ImuRpyFilterNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  latest_reference_yaw_ = quaternion_yaw(message->pose.pose.orientation);
  ++reference_sequence_;
  reference_received_at_ = std::chrono::steady_clock::now();
  have_reference_ = true;
}

void ImuRpyFilterNode::tracking_callback(
    const std_msgs::msg::Int32::SharedPtr message) {
  tracking_state_ = message->data;
  tracking_received_at_ = std::chrono::steady_clock::now();
  have_tracking_state_ = true;
}

void ImuRpyFilterNode::command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message) {
  latest_command_linear_ = message->linear.x;
  latest_command_yaw_rate_ = message->angular.z;
  command_received_at_ = std::chrono::steady_clock::now();
  have_command_ = std::isfinite(latest_command_linear_) &&
                  std::isfinite(latest_command_yaw_rate_);
}

void ImuRpyFilterNode::wheel_callback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  latest_wheel_linear_ = message->twist.twist.linear.x;
  latest_wheel_yaw_rate_ = message->twist.twist.angular.z;
  wheel_received_at_ = std::chrono::steady_clock::now();
  have_wheel_ = std::isfinite(latest_wheel_linear_) &&
                std::isfinite(latest_wheel_yaw_rate_);
}

void ImuRpyFilterNode::update_external_motion_diagnostics() {
  const auto current = std::chrono::steady_clock::now();
  const auto observation = collect_external_motion_diagnostics(
      command_fresh(current), wheel_fresh(current), latest_command_linear_,
      latest_command_yaw_rate_, latest_wheel_linear_, latest_wheel_yaw_rate_,
      command_linear_stationary_threshold_, command_yaw_stationary_threshold_,
      wheel_linear_stationary_threshold_, wheel_yaw_stationary_threshold_);
  external_stationary_ = observation.stationary;
  external_moving_ = observation.moving;
}

bool ImuRpyFilterNode::command_fresh(
    const std::chrono::steady_clock::time_point &current) const {
  return have_command_ &&
         std::chrono::duration<double>(current - command_received_at_)
                 .count() <= external_motion_timeout_;
}

bool ImuRpyFilterNode::wheel_fresh(
    const std::chrono::steady_clock::time_point &current) const {
  return have_wheel_ &&
         std::chrono::duration<double>(current - wheel_received_at_).count() <=
             external_motion_timeout_;
}

} // namespace imu_rpy_filter
