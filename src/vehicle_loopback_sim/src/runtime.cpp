#include "vehicle_loopback_sim/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace vehicle_loopback_sim
{
namespace
{

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(0.5 * yaw);
  result.w = std::cos(0.5 * yaw);
  return result;
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace

void VehicleLoopbackSim::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  desired_twist_ = {message->linear.x, message->angular.z};
  last_command_time_ = now();
}

void VehicleLoopbackSim::on_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  pose_ = {
    message->pose.pose.position.x,
    message->pose.pose.position.y,
    quaternion_yaw(message->pose.pose.orientation)};
  applied_twist_ = {};
  desired_twist_ = {};
  last_update_time_ = now();
  reset_obstacle_reporting();
  RCLCPP_INFO(
    get_logger(), "loopback pose reset to x=%.3f y=%.3f yaw=%.3f",
    pose_.x, pose_.y, pose_.yaw);
}

void VehicleLoopbackSim::update()
{
  const rclcpp::Time stamp = now();
  const double elapsed_s = std::clamp(
    (stamp - last_update_time_).seconds() * playback_speed_, 0.0, 0.20);
  last_update_time_ = stamp;
  const double command_age_s = (stamp - last_command_time_).seconds();
  const Twist2d desired = command_age_s <= command_timeout_s_ ? desired_twist_ : Twist2d{};
  applied_twist_ = limited_twist(
    desired, applied_twist_, elapsed_s,
    max_linear_speed_mps_, max_angular_speed_rps_,
    max_linear_accel_mps2_, max_angular_accel_rps2_);
  const Pose2d candidate = integrate_base_link(
    pose_, applied_twist_, elapsed_s, base_offset_x_m_, base_offset_y_m_);
  navigation_safety::Report collision_report;
  if (!pose_is_safe(candidate, &collision_report)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "static collision guard blocked simulated motion at candidate=(%.6f, %.6f, %.3fdeg) failure=%d corner=%d overlap=%.6fm",
      candidate.x, candidate.y, candidate.yaw * 180.0 / 3.14159265358979323846,
      static_cast<int>(collision_report.failure), collision_report.footprint_corner,
      collision_report.overlap);
    applied_twist_ = {};
  } else {
    pose_ = candidate;
  }
  publish_odometry(stamp);
  publish_imu(stamp);
  if (publish_tf_) {
    publish_transform(stamp);
  }
  if (publish_control_telemetry_) {
    publish_telemetry(stamp, command_age_s);
  }
}

void VehicleLoopbackSim::publish_odometry(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = map_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = pose_.x;
  message.pose.pose.position.y = pose_.y;
  message.pose.pose.orientation = yaw_quaternion(pose_.yaw);
  message.twist.twist.linear.x = applied_twist_.linear_x;
  message.twist.twist.angular.z = applied_twist_.angular_z;
  message.pose.covariance[0] = 1.0e-6;
  message.pose.covariance[7] = 1.0e-6;
  message.pose.covariance[35] = 1.0e-6;
  message.twist.covariance[0] = 1.0e-6;
  message.twist.covariance[35] = 1.0e-6;
  odom_publisher_->publish(message);
}

void VehicleLoopbackSim::publish_imu(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = base_frame_;
  message.orientation = yaw_quaternion(pose_.yaw);
  message.angular_velocity.z = applied_twist_.angular_z;
  message.orientation_covariance[8] = 1.0e-6;
  message.angular_velocity_covariance[8] = 1.0e-6;
  message.linear_acceleration_covariance[0] = -1.0;
  imu_publisher_->publish(message);
}

void VehicleLoopbackSim::publish_transform(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = pose_.x;
  transform.transform.translation.y = pose_.y;
  transform.transform.rotation = yaw_quaternion(pose_.yaw);
  tf_broadcaster_->sendTransform(transform);
}

void VehicleLoopbackSim::publish_telemetry(
  const rclcpp::Time & stamp, double command_age_s)
{
  const auto target = wheel_speeds(desired_twist_, wheel_track_m_);
  const auto measured = wheel_speeds(applied_twist_, wheel_track_m_);
  ++telemetry_sequence_;
  mission_control_interfaces::msg::ControlTelemetry message;
  message.header.stamp = stamp;
  const double elapsed_ms = std::max(0.0, (stamp - started_time_).seconds() * 1000.0);
  message.mcu_time_ms = static_cast<uint32_t>(static_cast<uint64_t>(elapsed_ms));
  message.sample_sequence = telemetry_sequence_;
  message.mode = mission_control_interfaces::msg::ControlTelemetry::MODE_NAVIGATION;
  message.emergency_stop = false;
  message.command_valid = command_age_s <= command_timeout_s_;
  message.command_age_ms = static_cast<uint32_t>(std::max(0.0, command_age_s) * 1000.0);
  message.received_linear_velocity_mps = static_cast<float>(desired_twist_.linear_x);
  message.received_angular_velocity_radps = static_cast<float>(desired_twist_.angular_z);
  message.target_left_velocity_mps = static_cast<float>(target.first);
  message.target_right_velocity_mps = static_cast<float>(target.second);
  message.measured_left_velocity_mps = static_cast<float>(measured.first);
  message.measured_right_velocity_mps = static_cast<float>(measured.second);
  telemetry_publisher_->publish(message);
}

void VehicleLoopbackSim::publish_status_bundle()
{
  std_msgs::msg::String fusion_status;
  fusion_status.data = "FULL";
  fusion_status_publisher_->publish(fusion_status);
  if (publish_actuator_health_) {
    std_msgs::msg::Bool health;
    health.data = true;
    actuator_health_publisher_->publish(health);
  }
  const rclcpp::Time stamp = now();
  publish_obstacle_report(stamp);

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4)
         << "{\"pose\":{\"x_m\":" << pose_.x
         << ",\"y_m\":" << pose_.y
         << ",\"yaw_rad\":" << pose_.yaw
         << "},\"twist\":{\"linear_x_mps\":" << applied_twist_.linear_x
         << ",\"angular_z_rps\":" << applied_twist_.angular_z
         << "},\"fusion_status\":\"FULL\"}";
  const std::string text = stream.str();
  if (text == last_status_text_) {
    return;
  }
  last_status_text_ = text;
  std_msgs::msg::String status;
  status.data = text;
  status_publisher_->publish(status);
}

}  // namespace vehicle_loopback_sim
