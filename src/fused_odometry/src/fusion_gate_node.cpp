#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "fused_odometry/fusion_logic.hpp"

namespace fused_odometry
{
namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

double age_seconds(const SteadyTime & stamp)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool finite(double value)
{
  return std::isfinite(value);
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double inverse_norm = 1.0 / std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x * inverse_norm;
  const double y = quaternion.y * inverse_norm;
  const double z = quaternion.z * inverse_norm;
  const double w = quaternion.w * inverse_norm;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

bool valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return finite(norm_squared) && norm_squared > 0.25 && norm_squared < 2.25;
}

double quaternion_tilt(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double inverse_norm = 1.0 / std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x * inverse_norm;
  const double y = quaternion.y * inverse_norm;
  const double body_z_in_world_z = std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0);
  return std::acos(body_z_in_world_z);
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

diagnostic_msgs::msg::KeyValue value(const std::string & key, const std::string & data)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = data;
  return result;
}

}  // namespace

class FusionGateNode : public rclcpp::Node
{
public:
  FusionGateNode()
  : Node("fused_odometry_gate"),
    wheel_gate_(
      positive("wheel_visual_reject_mps", 0.45),
      positive_count("residual_bad_samples", 3),
      positive_count("residual_recovery_samples", 10)),
    imu_gate_(
      positive("imu_visual_reject_radps", 0.8),
      static_cast<std::size_t>(get_parameter("residual_bad_samples").as_int()),
      static_cast<std::size_t>(get_parameter("residual_recovery_samples").as_int())),
    visual_vx_window_(positive("consistency_window_s", 0.75)),
    visual_wz_window_(get_parameter("consistency_window_s").as_double()),
    wheel_vx_window_(get_parameter("consistency_window_s").as_double()),
    wheel_visual_residual_window_(get_parameter("consistency_window_s").as_double()),
    wheel_imu_yaw_residual_window_(get_parameter("consistency_window_s").as_double()),
    imu_visual_residual_window_(get_parameter("consistency_window_s").as_double()),
    motion_classifier_(MotionClassifierConfig{
      positive("command_motion_threshold_mps", 0.08),
      positive("motion_moving_threshold_mps", 0.04),
      positive("motion_stationary_threshold_mps", 0.02),
      positive("motion_fault_dwell_s", 0.6),
      positive("motion_recovery_dwell_s", 1.0)}),
    angular_stall_detector_(AngularStallConfig{
      positive("angular_stall_command_threshold_radps", 0.30),
      positive("angular_stall_stationary_threshold_radps", 0.10),
      positive("angular_stall_dwell_s", 0.8),
      positive("angular_stall_recovery_dwell_s", 1.0)})
  {
    visual_topic_ = declare_parameter<std::string>("visual_topic", "/odom");
    raw_visual_topic_ = declare_parameter<std::string>("raw_visual_topic", "/odom/orb_raw");
    tracking_topic_ = declare_parameter<std::string>("tracking_topic", "/tracking_state");
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/wheel/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/filtered");
    fused_topic_ = declare_parameter<std::string>("fused_topic", "/odometry/fused");
    command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel_nav");
    visual_output_topic_ = declare_parameter<std::string>(
      "visual_output_topic", "/fusion/input/visual_odom");
    wheel_output_topic_ = declare_parameter<std::string>(
      "wheel_output_topic", "/fusion/input/wheel_odom");
    imu_output_topic_ = declare_parameter<std::string>(
      "imu_output_topic", "/fusion/input/imu");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/odometry/fusion_status");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "camera_link");
    visual_expected_frame_ = declare_parameter<std::string>("visual_expected_frame", "map");
    visual_expected_child_frame_ = declare_parameter<std::string>(
      "visual_expected_child_frame", "camera_link");
    raw_visual_expected_frame_ = declare_parameter<std::string>(
      "raw_visual_expected_frame", "map");
    wheel_expected_child_frame_ = declare_parameter<std::string>(
      "wheel_expected_child_frame", "base_link");
    imu_expected_frame_ = declare_parameter<std::string>("imu_expected_frame", "camera_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    visual_timeout_ = positive("visual_timeout_s", 0.4);
    tracking_timeout_ = positive("tracking_timeout_s", 0.6);
    wheel_timeout_ = positive("wheel_timeout_s", 0.35);
    imu_timeout_ = positive("imu_timeout_s", 0.15);
    fused_timeout_ = positive("fused_timeout_s", 0.5);
    command_timeout_ = positive("command_timeout_s", 0.4);
    raw_visual_timeout_ = positive("raw_visual_timeout_s", 0.3);
    raw_sync_tolerance_ = positive("raw_sync_tolerance_s", 0.08);
    raw_visual_max_tilt_ = positive("raw_visual_max_tilt_rad", 0.50);
    max_dead_reckoning_time_ = positive("max_dead_reckoning_time_s", 2.0);
    max_dead_reckoning_distance_ = positive("max_dead_reckoning_distance_m", 0.30);
    max_wheel_only_time_ = positive("max_wheel_only_time_s", 2.0);
    max_wheel_only_distance_ = positive("max_wheel_only_distance_m", 0.30);
    max_wheel_only_speed_ = positive("max_wheel_only_speed_mps", 0.30);
    max_wheel_speed_ = positive("max_wheel_speed_mps", 1.5);
    max_wheel_yaw_rate_ = positive("max_wheel_yaw_rate_radps", 4.0);
    wheel_soft_residual_ = positive("wheel_visual_soft_mps", 0.15);
    imu_soft_residual_ = positive("imu_visual_soft_radps", 0.20);
    visual_position_gate_ = positive("visual_fused_position_gate_m", 0.8);
    visual_yaw_gate_ = positive("visual_fused_yaw_gate_rad", 0.7);
    visual_hard_position_gate_ = positive("visual_recovery_hard_position_m", 1.5);
    visual_hard_yaw_gate_ = positive("visual_recovery_hard_yaw_rad", 1.0);
    visual_ramp_duration_ = positive("visual_recovery_ramp_s", 0.75);
    visual_ramp_initial_scale_ = positive("visual_recovery_initial_covariance_scale", 50.0);
    wheel_imu_yaw_soft_ = positive("wheel_imu_yaw_soft_radps", 0.15);
    wheel_imu_yaw_reject_ = positive("wheel_imu_yaw_reject_radps", 0.45);
    wheel_yaw_validation_time_ = positive("wheel_yaw_validation_s", 2.0);
    wheel_yaw_backup_time_ = positive("wheel_yaw_backup_s", 2.0);
    wheel_vx_variance_ = positive("wheel_vx_variance", 0.08);
    wheel_wz_variance_ = positive("wheel_wz_variance", 0.50);
    imu_wz_variance_ = positive("imu_wz_variance", 0.015);
    visual_xy_variance_ = positive("visual_xy_variance", 0.02);
    visual_yaw_variance_ = positive("visual_yaw_variance", 0.04);
    recovery_samples_ = positive_count("visual_recovery_samples", 5);
    robust_min_samples_ = positive_count("robust_min_samples", 5);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    visual_publisher_ = create_publisher<nav_msgs::msg::Odometry>(visual_output_topic_, sensor_qos);
    wheel_publisher_ = create_publisher<nav_msgs::msg::Odometry>(wheel_output_topic_, sensor_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_output_topic_, sensor_qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    visual_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      visual_topic_, sensor_qos, std::bind(&FusionGateNode::visual_callback, this, std::placeholders::_1));
    raw_visual_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_visual_topic_, sensor_qos,
      std::bind(&FusionGateNode::raw_visual_callback, this, std::placeholders::_1));
    tracking_subscription_ = create_subscription<std_msgs::msg::Int32>(
      tracking_topic_, sensor_qos, std::bind(&FusionGateNode::tracking_callback, this, std::placeholders::_1));
    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, sensor_qos, std::bind(&FusionGateNode::wheel_callback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&FusionGateNode::imu_callback, this, std::placeholders::_1));
    fused_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      fused_topic_, 10, std::bind(&FusionGateNode::fused_callback, this, std::placeholders::_1));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10, std::bind(&FusionGateNode::command_callback, this, std::placeholders::_1));

    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&FusionGateNode::publish_status, this));
    RCLCPP_INFO(
      get_logger(), "Fusion gate ready: visual=%s wheel=%s imu=%s output=%s (%s/%s)",
      visual_topic_.c_str(), wheel_topic_.c_str(), imu_topic_.c_str(), fused_topic_.c_str(),
      world_frame_.c_str(), base_frame_.c_str());
  }

private:
  double positive(const std::string & name, double default_value)
  {
    const double result = declare_parameter<double>(name, default_value);
    if (!finite(result) || result <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return result;
  }

  std::size_t positive_count(const std::string & name, std::int64_t default_value)
  {
    const auto result = declare_parameter<std::int64_t>(name, default_value);
    if (result < 1) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(result);
  }

  bool tracking_fresh_and_good() const
  {
    return tracking_received_ && tracking_good_ && age_seconds(tracking_received_at_) <= tracking_timeout_;
  }

  bool fused_fresh() const
  {
    return fused_received_ && age_seconds(fused_received_at_) <= fused_timeout_;
  }

  bool vision_healthy() const
  {
    return visual_accepted_ && tracking_fresh_and_good() &&
           age_seconds(visual_received_at_) <= visual_timeout_;
  }

  bool wheel_healthy() const
  {
    const bool classified_bad = motion_fault_ == MotionFault::kSlip ||
      motion_fault_ == MotionFault::kEncoderFailure;
    return wheel_received_ && !wheel_gate_.rejected() &&
           !classified_bad &&
           age_seconds(wheel_received_at_) <= wheel_timeout_;
  }

  bool imu_healthy() const
  {
    return imu_received_ && !imu_gate_.rejected() &&
           age_seconds(imu_received_at_) <= imu_timeout_;
  }

  void tracking_callback(const std_msgs::msg::Int32::SharedPtr message)
  {
    tracking_received_ = true;
    tracking_received_at_ = std::chrono::steady_clock::now();
    tracking_good_ = message->data == 2 || message->data == 5;
    tracking_state_ = message->data;
    if (!tracking_good_) {
      mark_visual_interrupted();
    }
  }

  void mark_visual_interrupted()
  {
    if (!visual_interrupted_) {
      visual_interrupted_ = true;
      visual_accepted_ = false;
      visual_recovery_count_ = 0;
      dead_reckoning_distance_ = 0.0;
      dead_reckoning_since_ = std::chrono::steady_clock::now();
      dead_reckoning_active_ = true;
    }
  }

  void raw_visual_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const bool orientation_valid = valid_quaternion(message->pose.pose.orientation);
    const double tilt = orientation_valid ?
      quaternion_tilt(message->pose.pose.orientation) : std::numeric_limits<double>::infinity();
    if (message->header.frame_id != raw_visual_expected_frame_ ||
      message->child_frame_id != visual_expected_child_frame_ ||
      !finite(position.x) || !finite(position.y) ||
      !orientation_valid || tilt > raw_visual_max_tilt_)
    {
      raw_visual_received_ = false;
      ++invalid_raw_visual_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting raw visual odometry with invalid frame/value or %.3f rad body tilt",
        tilt);
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (raw_visual_stamp_valid_ && stamp <= last_raw_visual_stamp_) {
      ++invalid_raw_visual_count_;
      return;
    }
    raw_visual_stamp_valid_ = true;
    last_raw_visual_stamp_ = stamp;
    latest_raw_pose_ = {
      position.x, position.y, quaternion_yaw(message->pose.pose.orientation)};
    raw_visual_received_ = true;
    raw_visual_received_at_ = std::chrono::steady_clock::now();
  }

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!finite(message->linear.x) || !finite(message->angular.z)) {
      return;
    }
    command_velocity_ = message->linear.x;
    command_yaw_rate_ = message->angular.z;
    command_received_ = true;
    command_received_at_ = std::chrono::steady_clock::now();
  }

  void visual_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    if (message->header.frame_id != visual_expected_frame_ ||
      message->child_frame_id != visual_expected_child_frame_ ||
      !finite(position.x) || !finite(position.y) || !valid_quaternion(orientation))
    {
      ++invalid_visual_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting visual odometry with invalid value/frame (%s -> %s; expected %s -> %s)",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        visual_expected_frame_.c_str(), visual_expected_child_frame_.c_str());
      return;
    }
    const rclcpp::Time message_stamp(message->header.stamp);
    if (visual_stamp_valid_ && message_stamp <= last_visual_message_stamp_) {
      ++invalid_visual_count_;
      return;
    }
    visual_stamp_valid_ = true;
    last_visual_message_stamp_ = message_stamp;
    if (!tracking_fresh_and_good()) {
      return;
    }

    const Pose2d continuous_raw{position.x, position.y, quaternion_yaw(orientation)};
    const bool recovering = !ever_accepted_visual_ || visual_interrupted_;
    if (recovering) {
      if (++visual_recovery_count_ < recovery_samples_) {
        return;
      }
      if (!ever_accepted_visual_) {
        aligner_.clear();
      }
    }

    const bool raw_synchronized = raw_visual_received_ &&
      age_seconds(raw_visual_received_at_) <= raw_visual_timeout_ &&
      std::abs((message_stamp - last_raw_visual_stamp_).seconds()) <= raw_sync_tolerance_;
    Pose2d aligned = aligner_.apply(continuous_raw);
    using_raw_visual_ = false;
    if (raw_synchronized) {
      if (!raw_aligner_.initialized()) {
        raw_aligner_.align_to(latest_raw_pose_, aligned);
      }
      aligned = raw_aligner_.apply(latest_raw_pose_);
      using_raw_visual_ = true;
    }

    // Preserve the established visual map transform across an outage. A plausible
    // recovery can then correct wheel drift; an implausible jump is not disguised
    // by moving the visual origin onto the dead-reckoned pose.
    if (recovering && ever_accepted_visual_ && fused_fresh() &&
      !pose_residual_within(
        aligned, fused_pose_, visual_hard_position_gate_, visual_hard_yaw_gate_))
    {
      ++visual_recovery_rejected_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejecting visual recovery inconsistent with prediction (position=%.3f m yaw=%.3f rad)",
        std::hypot(aligned.x - fused_pose_.x, aligned.y - fused_pose_.y),
        std::abs(wrap_angle(aligned.yaw - fused_pose_.yaw)));
      return;
    }

    if (recovering) {
      visual_interrupted_ = false;
      dead_reckoning_active_ = false;
      dead_reckoning_distance_ = 0.0;
      ever_accepted_visual_ = true;
      visual_ramp_started_at_ = steady_seconds();
      visual_ramp_active_ = true;
      visual_realigned_until_ = visual_ramp_started_at_ + visual_ramp_duration_;
      ++visual_realign_count_;
      RCLCPP_INFO(
        get_logger(),
        "Visual odometry accepted after %zu healthy samples; correcting fused drift gradually",
        visual_recovery_count_);
    }

    double position_residual = 0.0;
    double yaw_residual = 0.0;
    if (fused_fresh()) {
      position_residual = std::hypot(aligned.x - fused_pose_.x, aligned.y - fused_pose_.y);
      yaw_residual = std::abs(wrap_angle(aligned.yaw - fused_pose_.yaw));
    }
    double position_scale = std::clamp(
      1.0 + std::pow(position_residual / visual_position_gate_, 2), 1.0, 25.0);
    double yaw_scale = std::clamp(
      1.0 + std::pow(yaw_residual / visual_yaw_gate_, 2), 1.0, 25.0);
    if (!recovering && fused_fresh() &&
      !pose_residual_within(
        aligned, fused_pose_, visual_hard_position_gate_, visual_hard_yaw_gate_))
    {
      ++visual_recovery_rejected_count_;
      mark_visual_interrupted();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Visual pose jumped outside the fusion gate; waiting for stable recovery");
      return;
    }
    if (visual_ramp_active_) {
      const double progress = std::clamp(
        (steady_seconds() - visual_ramp_started_at_) / visual_ramp_duration_, 0.0, 1.0);
      const double ramp_scale = 1.0 + (visual_ramp_initial_scale_ - 1.0) * (1.0 - progress);
      position_scale = std::max(position_scale, ramp_scale);
      yaw_scale = std::max(yaw_scale, ramp_scale);
      if (progress >= 1.0) {
        visual_ramp_active_ = false;
      }
    }

    nav_msgs::msg::Odometry output = *message;
    output.header.frame_id = world_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = aligned.x;
    output.pose.pose.position.y = aligned.y;
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation = yaw_quaternion(aligned.yaw);
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = visual_xy_variance_ * position_scale;
    output.pose.covariance[7] = visual_xy_variance_ * position_scale;
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = visual_yaw_variance_ * yaw_scale;
    output.twist.covariance.fill(0.0);
    visual_publisher_->publish(output);

    const double stamp = message_stamp.seconds();
    if (last_visual_pose_valid_) {
      const double dt = stamp - last_visual_stamp_;
      if (dt > 0.01 && dt < 0.5) {
        const double dx = aligned.x - last_visual_pose_.x;
        const double dy = aligned.y - last_visual_pose_.y;
        visual_forward_velocity_ =
          (std::cos(last_visual_pose_.yaw) * dx + std::sin(last_visual_pose_.yaw) * dy) / dt;
        visual_yaw_rate_ = wrap_angle(aligned.yaw - last_visual_pose_.yaw) / dt;
        visual_velocity_valid_ = finite(visual_forward_velocity_) && finite(visual_yaw_rate_);
        if (visual_velocity_valid_) {
          const double sample_time = steady_seconds();
          visual_vx_window_.add(sample_time, visual_forward_velocity_);
          visual_wz_window_.add(sample_time, visual_yaw_rate_);
        }
      }
    }
    last_visual_pose_ = aligned;
    last_visual_stamp_ = stamp;
    last_visual_pose_valid_ = true;
    visual_accepted_ = true;
    visual_received_at_ = std::chrono::steady_clock::now();
  }

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double velocity = message->twist.twist.linear.x;
    const double wheel_yaw_rate = message->twist.twist.angular.z;
    if (message->child_frame_id != wheel_expected_child_frame_ ||
      !finite(velocity) || std::abs(velocity) > max_wheel_speed_)
    {
      ++invalid_wheel_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting wheel odometry with invalid vx/child frame (%s; expected %s)",
        message->child_frame_id.c_str(), wheel_expected_child_frame_.c_str());
      return;
    }
    const rclcpp::Time message_stamp(message->header.stamp);
    if (wheel_stamp_valid_ && message_stamp <= last_wheel_message_stamp_) {
      ++invalid_wheel_count_;
      return;
    }
    wheel_stamp_valid_ = true;
    last_wheel_message_stamp_ = message_stamp;

    const double sample_time = steady_seconds();
    wheel_vx_window_.add(sample_time, velocity);
    latest_wheel_velocity_ = velocity;
    latest_wheel_yaw_rate_ = wheel_yaw_rate;
    wheel_yaw_rate_finite_ = finite(wheel_yaw_rate) &&
      std::abs(wheel_yaw_rate) <= max_wheel_yaw_rate_;

    double residual = 0.0;
    bool compared = false;
    if (visual_vx_window_.ready(robust_min_samples_) && vision_healthy()) {
      wheel_visual_residual_window_.add(
        sample_time, std::abs(velocity - visual_vx_window_.median()));
      residual = wheel_visual_residual_window_.median();
      wheel_gate_.update(residual + 1.4826 * wheel_visual_residual_window_.mad());
      compared = true;
    }
    if (wheel_yaw_rate_finite_ && imu_received_ &&
      age_seconds(imu_received_at_) <= imu_timeout_)
    {
      wheel_imu_yaw_residual_window_.add(
        sample_time, std::abs(wheel_yaw_rate - latest_imu_yaw_rate_));
      if (wheel_imu_yaw_residual_window_.ready(robust_min_samples_)) {
        const double robust_residual = wheel_imu_yaw_residual_window_.median() +
          1.4826 * wheel_imu_yaw_residual_window_.mad();
        if (robust_residual <= wheel_imu_yaw_reject_) {
          if (!wheel_yaw_validation_active_) {
            wheel_yaw_validation_active_ = true;
            wheel_yaw_validation_started_at_ = sample_time;
          }
          if (sample_time - wheel_yaw_validation_started_at_ >= wheel_yaw_validation_time_) {
            wheel_yaw_validated_ = true;
            wheel_yaw_last_validated_at_ = sample_time;
          }
        } else {
          wheel_yaw_validation_active_ = false;
          wheel_yaw_validated_ = false;
        }
      }
    }
    wheel_received_ = true;
    wheel_received_at_ = std::chrono::steady_clock::now();
    wheel_residual_ = compared ? residual : 0.0;
    if (wheel_gate_.rejected() || !ever_accepted_visual_ ||
      motion_fault_ != MotionFault::kNone)
    {
      return;
    }

    nav_msgs::msg::Odometry output;
    output.header = message->header;
    output.header.frame_id = world_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.orientation.w = 1.0;
    output.twist.twist.linear.x = velocity;
    output.pose.covariance.fill(0.0);
    output.twist.covariance.fill(0.0);
    for (std::size_t index = 0; index < 6; ++index) {
      output.pose.covariance[index * 6 + index] = 1.0e6;
      output.twist.covariance[index * 6 + index] = 1.0e6;
    }
    const double scale = compared ?
      wheel_gate_.covariance_scale(residual, wheel_soft_residual_) : 2.0;
    const double input_vx_variance = message->twist.covariance[0];
    const double base_vx_variance = finite(input_vx_variance) && input_vx_variance > 0.0 ?
      std::max(wheel_vx_variance_, input_vx_variance) : wheel_vx_variance_;
    output.twist.covariance[0] = base_vx_variance * scale;
    const bool wheel_yaw_backup = wheel_yaw_validated_ &&
      sample_time - wheel_yaw_last_validated_at_ <= wheel_yaw_backup_time_ &&
      std::abs(velocity) <= max_wheel_only_speed_;
    if (wheel_yaw_rate_finite_ && wheel_yaw_validated_ &&
      (imu_healthy() || wheel_yaw_backup))
    {
      output.twist.twist.angular.z = wheel_yaw_rate;
      const double input_wz_variance = message->twist.covariance[35];
      const double base_wz_variance = finite(input_wz_variance) && input_wz_variance > 0.0 ?
        std::max(wheel_wz_variance_, input_wz_variance) : wheel_wz_variance_;
      const double yaw_residual = wheel_imu_yaw_residual_window_.ready(robust_min_samples_) ?
        wheel_imu_yaw_residual_window_.median() : wheel_imu_yaw_reject_;
      output.twist.covariance[35] = base_wz_variance * std::clamp(
        1.0 + std::pow(yaw_residual / wheel_imu_yaw_soft_, 2), 1.0, 100.0);
    }
    wheel_publisher_->publish(output);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const double yaw_rate = message->angular_velocity.z;
    if (message->header.frame_id != imu_expected_frame_ || !finite(yaw_rate)) {
      ++invalid_imu_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting IMU with invalid wz/frame (%s; expected %s)",
        message->header.frame_id.c_str(), imu_expected_frame_.c_str());
      return;
    }
    const rclcpp::Time message_stamp(message->header.stamp);
    if (imu_stamp_valid_ && message_stamp <= last_imu_message_stamp_) {
      ++invalid_imu_count_;
      return;
    }
    imu_stamp_valid_ = true;
    last_imu_message_stamp_ = message_stamp;
    latest_imu_yaw_rate_ = yaw_rate;
    double residual = 0.0;
    bool compared = false;
    if (visual_velocity_valid_ && vision_healthy()) {
      imu_visual_residual_window_.add(
        steady_seconds(), std::abs(yaw_rate - visual_yaw_rate_));
      residual = imu_visual_residual_window_.median();
      imu_gate_.update(residual + 1.4826 * imu_visual_residual_window_.mad());
      compared = true;
    }
    imu_received_ = true;
    imu_received_at_ = std::chrono::steady_clock::now();
    imu_residual_ = compared ? residual : 0.0;
    if (imu_gate_.rejected() || !ever_accepted_visual_) {
      return;
    }

    sensor_msgs::msg::Imu output;
    output.header = message->header;
    output.header.frame_id = base_frame_;
    output.orientation_covariance[0] = -1.0;
    output.linear_acceleration_covariance[0] = -1.0;
    output.angular_velocity.z = yaw_rate;
    output.angular_velocity_covariance.fill(0.0);
    output.angular_velocity_covariance[0] = 1.0e6;
    output.angular_velocity_covariance[4] = 1.0e6;
    const double scale = compared ?
      imu_gate_.covariance_scale(residual, imu_soft_residual_) : 2.0;
    const double source_variance = message->angular_velocity_covariance[8];
    const double base_variance = finite(source_variance) && source_variance > 0.0 ?
      std::max(imu_wz_variance_, source_variance) : imu_wz_variance_;
    output.angular_velocity_covariance[8] = base_variance * scale;
    imu_publisher_->publish(output);
  }

  void fused_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    if (message->header.frame_id != world_frame_ || message->child_frame_id != base_frame_ ||
      !finite(position.x) || !finite(position.y) ||
      !valid_quaternion(message->pose.pose.orientation))
    {
      return;
    }
    const Pose2d current{position.x, position.y, quaternion_yaw(message->pose.pose.orientation)};
    if (dead_reckoning_active_ && fused_received_) {
      dead_reckoning_distance_ += std::hypot(
        current.x - fused_pose_.x, current.y - fused_pose_.y);
    }
    fused_pose_ = current;
    fused_received_ = true;
    fused_received_at_ = std::chrono::steady_clock::now();
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = message->header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = position.x;
      transform.transform.translation.y = position.y;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = yaw_quaternion(current.yaw);
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_status()
  {
    if (visual_accepted_ && !vision_healthy()) {
      mark_visual_interrupted();
    }
    const bool vision = vision_healthy();
    const bool command_fresh = command_received_ &&
      age_seconds(command_received_at_) <= command_timeout_;
    const bool wheel_measurement_fresh = wheel_received_ &&
      age_seconds(wheel_received_at_) <= wheel_timeout_;
    motion_fault_ = motion_classifier_.update(
      steady_seconds(), command_velocity_,
      wheel_vx_window_.ready(robust_min_samples_) ? wheel_vx_window_.median() : latest_wheel_velocity_,
      visual_vx_window_.ready(robust_min_samples_) ? visual_vx_window_.median() : visual_forward_velocity_,
      command_fresh, wheel_measurement_fresh,
      vision && visual_vx_window_.ready(robust_min_samples_));
    const bool wheel = wheel_healthy();
    const bool imu = imu_healthy();
    std::size_t angular_source_count = 0;
    double measured_yaw_rate = 0.0;
    if (vision && visual_velocity_valid_) {
      measured_yaw_rate = std::max(measured_yaw_rate, std::abs(visual_yaw_rate_));
      ++angular_source_count;
    }
    if (imu) {
      measured_yaw_rate = std::max(measured_yaw_rate, std::abs(latest_imu_yaw_rate_));
      ++angular_source_count;
    }
    if (wheel && wheel_yaw_rate_finite_) {
      measured_yaw_rate = std::max(measured_yaw_rate, std::abs(latest_wheel_yaw_rate_));
      ++angular_source_count;
    }
    angular_stalled_ = angular_stall_detector_.update(
      steady_seconds(), command_yaw_rate_, measured_yaw_rate,
      command_fresh, angular_source_count >= 2);
    const bool wheel_yaw_backup = wheel_yaw_validated_ &&
      steady_seconds() - wheel_yaw_last_validated_at_ <= wheel_yaw_backup_time_ &&
      std::abs(latest_wheel_velocity_) <= max_wheel_only_speed_;
    const bool visual_realigned = steady_seconds() <= visual_realigned_until_;
    const double outage_time = dead_reckoning_active_ ? age_seconds(dead_reckoning_since_) : 0.0;
    const FusionMode mode = select_mode(
      ever_accepted_visual_, vision, wheel, imu, wheel_yaw_backup, visual_realigned,
      motion_fault_ == MotionFault::kStalled || angular_stalled_,
      outage_time, dead_reckoning_distance_,
      max_dead_reckoning_time_, max_dead_reckoning_distance_,
      max_wheel_only_time_, max_wheel_only_distance_);

    std_msgs::msg::String status;
    status.data = mode_name(mode);
    status_publisher_->publish(status);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = "fused_odometry/health";
    item.hardware_id = "odometry_fusion";
    item.level = (mode == FusionMode::kFault || mode == FusionMode::kFaultStalled) ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (mode == FusionMode::kFull ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    item.message = mode_name(mode);
    item.values.push_back(value("vision", vision ? "healthy" : "unavailable"));
    item.values.push_back(value("wheel", wheel ? "healthy" : "unavailable"));
    item.values.push_back(value("imu", imu ? "healthy" : "unavailable"));
    item.values.push_back(value("tracking_state", std::to_string(tracking_state_)));
    item.values.push_back(value("visual_recovery_samples", std::to_string(visual_recovery_count_)));
    item.values.push_back(value("wheel_visual_residual_mps", std::to_string(wheel_residual_)));
    item.values.push_back(value("imu_visual_residual_radps", std::to_string(imu_residual_)));
    item.values.push_back(value(
      "wheel_imu_yaw_residual_radps",
      std::to_string(wheel_imu_yaw_residual_window_.median())));
    item.values.push_back(value(
      "wheel_visual_residual_mad_mps",
      std::to_string(wheel_visual_residual_window_.mad())));
    item.values.push_back(value("motion_fault", motion_fault_name(motion_fault_)));
    item.values.push_back(value("command_vx_mps", std::to_string(command_velocity_)));
    item.values.push_back(value("command_wz_radps", std::to_string(command_yaw_rate_)));
    item.values.push_back(value("measured_wz_radps", std::to_string(measured_yaw_rate)));
    item.values.push_back(value("angular_sources", std::to_string(angular_source_count)));
    item.values.push_back(value("angular_stalled", angular_stalled_ ? "true" : "false"));
    item.values.push_back(value("wheel_yaw_validated", wheel_yaw_validated_ ? "true" : "false"));
    item.values.push_back(value("wheel_yaw_backup", wheel_yaw_backup ? "true" : "false"));
    item.values.push_back(value("raw_visual_used", using_raw_visual_ ? "true" : "false"));
    item.values.push_back(value("visual_realign_count", std::to_string(visual_realign_count_)));
    item.values.push_back(value(
      "visual_recovery_rejected", std::to_string(visual_recovery_rejected_count_)));
    item.values.push_back(value("vision_outage_s", std::to_string(outage_time)));
    item.values.push_back(value("dead_reckoning_distance_m", std::to_string(dead_reckoning_distance_)));
    item.values.push_back(value(
      "wheel_frame_approximation",
      wheel_expected_child_frame_ == base_frame_ ? "false" : "true"));
    item.values.push_back(value("invalid_visual", std::to_string(invalid_visual_count_)));
    item.values.push_back(value("invalid_raw_visual", std::to_string(invalid_raw_visual_count_)));
    item.values.push_back(value("invalid_wheel", std::to_string(invalid_wheel_count_)));
    item.values.push_back(value("invalid_imu", std::to_string(invalid_imu_count_)));
    diagnostics.status.push_back(item);
    diagnostics_publisher_->publish(diagnostics);

    if (mode != last_mode_) {
      RCLCPP_WARN(get_logger(), "Fusion mode: %s", mode_name(mode));
      last_mode_ = mode;
    }
  }

  ResidualGate wheel_gate_;
  ResidualGate imu_gate_;
  PoseAligner aligner_;
  PoseAligner raw_aligner_;
  RobustWindow visual_vx_window_;
  RobustWindow visual_wz_window_;
  RobustWindow wheel_vx_window_;
  RobustWindow wheel_visual_residual_window_;
  RobustWindow wheel_imu_yaw_residual_window_;
  RobustWindow imu_visual_residual_window_;
  MotionClassifier motion_classifier_;
  AngularStallDetector angular_stall_detector_;

  std::string visual_topic_;
  std::string raw_visual_topic_;
  std::string tracking_topic_;
  std::string wheel_topic_;
  std::string imu_topic_;
  std::string fused_topic_;
  std::string command_topic_;
  std::string visual_output_topic_;
  std::string wheel_output_topic_;
  std::string imu_output_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string world_frame_;
  std::string base_frame_;
  std::string visual_expected_frame_;
  std::string visual_expected_child_frame_;
  std::string raw_visual_expected_frame_;
  std::string wheel_expected_child_frame_;
  std::string imu_expected_frame_;
  double visual_timeout_{0.4};
  bool publish_tf_{true};
  double tracking_timeout_{0.6};
  double wheel_timeout_{0.35};
  double imu_timeout_{0.15};
  double fused_timeout_{0.5};
  double command_timeout_{0.4};
  double raw_visual_timeout_{0.3};
  double raw_sync_tolerance_{0.08};
  double raw_visual_max_tilt_{0.5};
  double max_dead_reckoning_time_{2.0};
  double max_dead_reckoning_distance_{0.3};
  double max_wheel_only_time_{2.0};
  double max_wheel_only_distance_{0.3};
  double max_wheel_only_speed_{0.3};
  double max_wheel_speed_{1.5};
  double max_wheel_yaw_rate_{4.0};
  double wheel_soft_residual_{0.15};
  double imu_soft_residual_{0.2};
  double visual_position_gate_{0.8};
  double visual_yaw_gate_{0.7};
  double visual_hard_position_gate_{1.5};
  double visual_hard_yaw_gate_{1.0};
  double visual_ramp_duration_{0.75};
  double visual_ramp_initial_scale_{50.0};
  double wheel_imu_yaw_soft_{0.15};
  double wheel_imu_yaw_reject_{0.45};
  double wheel_yaw_validation_time_{2.0};
  double wheel_yaw_backup_time_{2.0};
  double wheel_vx_variance_{0.08};
  double wheel_wz_variance_{0.5};
  double imu_wz_variance_{0.015};
  double visual_xy_variance_{0.02};
  double visual_yaw_variance_{0.04};
  std::size_t recovery_samples_{5};
  std::size_t robust_min_samples_{5};

  bool tracking_received_{false};
  bool tracking_good_{false};
  bool visual_accepted_{false};
  bool ever_accepted_visual_{false};
  bool visual_interrupted_{true};
  bool wheel_received_{false};
  bool imu_received_{false};
  bool fused_received_{false};
  bool last_visual_pose_valid_{false};
  bool visual_velocity_valid_{false};
  bool dead_reckoning_active_{false};
  bool raw_visual_received_{false};
  bool raw_visual_stamp_valid_{false};
  bool command_received_{false};
  bool using_raw_visual_{false};
  bool visual_ramp_active_{false};
  bool wheel_yaw_rate_finite_{false};
  bool wheel_yaw_validation_active_{false};
  bool wheel_yaw_validated_{false};
  bool angular_stalled_{false};
  bool visual_stamp_valid_{false};
  bool wheel_stamp_valid_{false};
  bool imu_stamp_valid_{false};
  int tracking_state_{-1};
  std::size_t visual_recovery_count_{0};
  std::size_t invalid_visual_count_{0};
  std::size_t invalid_wheel_count_{0};
  std::size_t invalid_imu_count_{0};
  std::size_t invalid_raw_visual_count_{0};
  std::size_t visual_realign_count_{0};
  std::size_t visual_recovery_rejected_count_{0};
  double last_visual_stamp_{0.0};
  double visual_forward_velocity_{0.0};
  double visual_yaw_rate_{0.0};
  double wheel_residual_{0.0};
  double imu_residual_{0.0};
  double dead_reckoning_distance_{0.0};
  double command_velocity_{0.0};
  double command_yaw_rate_{0.0};
  double latest_wheel_velocity_{0.0};
  double latest_wheel_yaw_rate_{0.0};
  double latest_imu_yaw_rate_{0.0};
  double wheel_yaw_validation_started_at_{0.0};
  double wheel_yaw_last_validated_at_{0.0};
  double visual_ramp_started_at_{0.0};
  double visual_realigned_until_{0.0};
  Pose2d last_visual_pose_{};
  Pose2d fused_pose_{};
  Pose2d latest_raw_pose_{};
  MotionFault motion_fault_{MotionFault::kNone};
  FusionMode last_mode_{FusionMode::kFull};
  SteadyTime tracking_received_at_{};
  SteadyTime visual_received_at_{};
  SteadyTime wheel_received_at_{};
  SteadyTime imu_received_at_{};
  SteadyTime fused_received_at_{};
  SteadyTime dead_reckoning_since_{};
  SteadyTime raw_visual_received_at_{};
  SteadyTime command_received_at_{};
  rclcpp::Time last_visual_message_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_message_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_message_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_raw_visual_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr visual_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_visual_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracking_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace fused_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fused_odometry::FusionGateNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fused_odometry_gate"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
