#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "std_msgs/msg/u_int64.hpp"
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
    visual_vx_window_(positive("consistency_window_s", 0.75)),
    visual_wz_window_(get_parameter("consistency_window_s").as_double()),
    raw_visual_vx_window_(get_parameter("consistency_window_s").as_double()),
    raw_visual_wz_window_(get_parameter("consistency_window_s").as_double()),
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
    raw_visual_topic_ = declare_parameter<std::string>("raw_visual_topic", "/odom/orb_raw");
    tracking_topic_ = declare_parameter<std::string>("tracking_topic", "/tracking_state");
    map_change_topic_ = declare_parameter<std::string>(
      "map_change_topic", "/orbslam3/map_change");
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
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    visual_expected_child_frame_ = declare_parameter<std::string>(
      "visual_expected_child_frame", "base_link");
    raw_visual_expected_frame_ = declare_parameter<std::string>(
      "raw_visual_expected_frame", "map");
    wheel_expected_child_frame_ = declare_parameter<std::string>(
      "wheel_expected_child_frame", "base_link");
    imu_expected_frame_ = declare_parameter<std::string>("imu_expected_frame", "camera_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    visual_timeout_ = positive("visual_timeout_s", 0.4);
    tracking_timeout_ = positive("tracking_timeout_s", 0.6);
    wheel_timeout_ = positive("wheel_timeout_s", 0.35);
    imu_timeout_ = positive("imu_timeout_s", 0.15);
    command_timeout_ = positive("command_timeout_s", 0.4);
    raw_visual_timeout_ = positive("raw_visual_timeout_s", 0.3);
    raw_visual_max_tilt_ = positive("raw_visual_max_tilt_rad", 0.50);
    map_change_grace_ = positive("map_change_grace_s", 1.0);
    max_dead_reckoning_time_ = positive("max_dead_reckoning_time_s", 2.0);
    max_dead_reckoning_distance_ = positive("max_dead_reckoning_distance_m", 0.30);
    max_wheel_only_time_ = positive("max_wheel_only_time_s", 2.0);
    max_wheel_only_distance_ = positive("max_wheel_only_distance_m", 0.30);
    max_wheel_only_speed_ = positive("max_wheel_only_speed_mps", 0.30);
    max_wheel_speed_ = positive("max_wheel_speed_mps", 1.5);
    max_wheel_yaw_rate_ = positive("max_wheel_yaw_rate_radps", 4.0);
    max_imu_yaw_rate_ = positive("max_imu_yaw_rate_radps", 4.0);
    wheel_soft_residual_ = positive("wheel_visual_soft_mps", 0.15);
    visual_stationary_speed_ = positive("visual_stationary_speed_mps", 0.025);
    stationary_wheel_reject_speed_ = positive("stationary_wheel_reject_speed_mps", 0.03);
    imu_soft_residual_ = positive("imu_visual_soft_radps", 0.20);
    imu_visual_covariance_cap_ = positive("imu_visual_covariance_cap_radps", 0.80);
    visual_ramp_duration_ = positive("visual_recovery_ramp_s", 0.75);
    visual_ramp_initial_scale_ = positive("visual_recovery_initial_covariance_scale", 50.0);
    wheel_imu_yaw_soft_ = positive("wheel_imu_yaw_soft_radps", 0.15);
    wheel_imu_yaw_reject_ = positive("wheel_imu_yaw_reject_radps", 0.45);
    wheel_yaw_validation_min_rate_ = positive(
      "wheel_yaw_validation_min_rate_radps", 0.08);
    wheel_yaw_validation_time_ = positive("wheel_yaw_validation_s", 2.0);
    wheel_yaw_backup_time_ = positive("wheel_yaw_backup_s", 2.0);
    fuse_wheel_yaw_ = declare_parameter<bool>("fuse_wheel_yaw", false);
    wheel_vx_variance_ = positive("wheel_vx_variance", 0.08);
    wheel_turn_downweight_start_ = positive(
      "wheel_turn_downweight_start_radps", 0.15);
    wheel_turn_full_downweight_ = positive(
      "wheel_turn_full_downweight_radps", 0.60);
    if (wheel_turn_full_downweight_ <= wheel_turn_downweight_start_) {
      throw std::invalid_argument(
              "wheel_turn_full_downweight_radps must exceed "
              "wheel_turn_downweight_start_radps");
    }
    wheel_turn_covariance_scale_ = positive("wheel_turn_covariance_scale", 100.0);
    wheel_in_place_max_linear_speed_ = positive(
      "wheel_in_place_max_linear_speed_mps", 0.10);
    wheel_in_place_min_yaw_rate_ = positive(
      "wheel_in_place_min_yaw_rate_radps", 0.30);
    wheel_wz_variance_ = positive("wheel_wz_variance", 0.50);
    imu_wz_variance_ = positive("imu_wz_variance", 0.015);
    imu_bias_time_constant_ = positive("imu_bias_time_constant_s", 3.0);
    imu_bias_maximum_ = positive("imu_bias_max_radps", 0.08);
    imu_bias_learning_command_rate_ = positive(
      "imu_bias_learning_max_command_radps", 0.08);
    imu_bias_learning_visual_rate_ = positive(
      "imu_bias_learning_max_visual_rate_radps", 0.12);
    imu_bias_estimator_ = YawBiasEstimator(imu_bias_time_constant_, imu_bias_maximum_);
    visual_xy_variance_ = positive("visual_xy_variance", 0.02);
    visual_yaw_variance_ = positive("visual_yaw_variance", 0.04);
    visual_vx_variance_ = positive("visual_vx_variance", 0.01);
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

    raw_visual_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_visual_topic_, sensor_qos,
      std::bind(&FusionGateNode::raw_visual_callback, this, std::placeholders::_1));
    tracking_subscription_ = create_subscription<std_msgs::msg::Int32>(
      tracking_topic_, sensor_qos, std::bind(&FusionGateNode::tracking_callback, this, std::placeholders::_1));
    map_change_subscription_ = create_subscription<std_msgs::msg::UInt64>(
      map_change_topic_, 10,
      std::bind(&FusionGateNode::map_change_callback, this, std::placeholders::_1));
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
      raw_visual_topic_.c_str(), wheel_topic_.c_str(), imu_topic_.c_str(), fused_topic_.c_str(),
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
    return imu_received_ && age_seconds(imu_received_at_) <= imu_timeout_;
  }

  bool raw_visual_increment_healthy() const
  {
    return raw_visual_received_ && raw_visual_velocity_valid_ &&
      tracking_fresh_and_good() &&
      age_seconds(raw_visual_received_at_) <= raw_visual_timeout_ &&
      raw_visual_vx_window_.ready(robust_min_samples_);
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

  void map_change_callback(const std_msgs::msg::UInt64::SharedPtr message)
  {
    map_change_sequence_ = message->data;
    map_change_grace_until_ = steady_seconds() + map_change_grace_;
    RCLCPP_INFO(
      get_logger(), "Accepting gated ORB map correction sequence %lu",
      static_cast<unsigned long>(map_change_sequence_));
  }

  void mark_visual_interrupted()
  {
    if (!visual_interrupted_) {
      visual_interrupted_ = true;
      visual_accepted_ = false;
      visual_velocity_valid_ = false;
      visual_forward_velocity_ = 0.0;
      visual_yaw_rate_ = 0.0;
      imu_residual_ = 0.0;
      imu_visual_robust_residual_ = 0.0;
      visual_yaw_disagreement_scale_ = 1.0;
      visual_vx_window_.clear();
      visual_wz_window_.clear();
      raw_visual_vx_window_.clear();
      raw_visual_wz_window_.clear();
      wheel_visual_residual_window_.clear();
      imu_visual_residual_window_.clear();
      raw_visual_velocity_valid_ = false;
      last_raw_increment_pose_valid_ = false;
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
    const Pose2d current_raw_pose{
      position.x, position.y, quaternion_yaw(message->pose.pose.orientation)};
    if (last_raw_increment_pose_valid_) {
      const double dt = (stamp - last_raw_increment_stamp_).seconds();
      if (dt > 0.01 && dt < 0.5) {
        const double dx = current_raw_pose.x - last_raw_increment_pose_.x;
        const double dy = current_raw_pose.y - last_raw_increment_pose_.y;
        const double forward_velocity =
          (std::cos(last_raw_increment_pose_.yaw) * dx +
          std::sin(last_raw_increment_pose_.yaw) * dy) / dt;
        const double yaw_rate =
          wrap_angle(current_raw_pose.yaw - last_raw_increment_pose_.yaw) / dt;
        raw_visual_velocity_valid_ = finite(forward_velocity) && finite(yaw_rate) &&
          std::abs(forward_velocity) <= max_wheel_speed_ &&
          std::abs(yaw_rate) <= max_imu_yaw_rate_;
        if (raw_visual_velocity_valid_) {
          const double sample_time = steady_seconds();
          raw_visual_vx_window_.add(sample_time, forward_velocity);
          raw_visual_wz_window_.add(sample_time, yaw_rate);
          raw_visual_forward_velocity_ = forward_velocity;
          raw_visual_yaw_rate_ = yaw_rate;
        } else {
          raw_visual_vx_window_.clear();
          raw_visual_wz_window_.clear();
        }
      } else {
        raw_visual_velocity_valid_ = false;
        raw_visual_vx_window_.clear();
        raw_visual_wz_window_.clear();
      }
    }
    last_raw_increment_pose_ = current_raw_pose;
    last_raw_increment_stamp_ = stamp;
    last_raw_increment_pose_valid_ = true;
    raw_visual_received_ = true;
    raw_visual_received_at_ = std::chrono::steady_clock::now();
    publish_raw_visual(*message, current_raw_pose);
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

  void publish_raw_visual(
    const nav_msgs::msg::Odometry & message, const Pose2d & raw_pose)
  {
    if (!tracking_fresh_and_good()) {
      return;
    }

    const bool recovering = !ever_accepted_visual_ || visual_interrupted_;
    const bool map_change_grace = steady_seconds() <= map_change_grace_until_;
    // A normal frame must have a physically plausible raw increment. On ORB
    // loop closure MapChanged() is published before the corrected pose, so the
    // one discontinuous observation is allowed and smoothed by map->odom.
    if (!raw_visual_velocity_valid_ && !map_change_grace) {
      if (!recovering) {
        ++visual_recovery_rejected_count_;
        mark_visual_interrupted();
      }
      return;
    }
    if (recovering) {
      if (!raw_visual_increment_healthy()) {
        return;
      }
      if (++visual_recovery_count_ < recovery_samples_) {
        return;
      }
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
        "Raw visual odometry accepted after %zu coherent samples; correcting global drift",
        visual_recovery_count_);
    }

    double position_scale = 1.0;
    double yaw_scale = 1.0;
    visual_yaw_disagreement_scale_ = 1.0;
    if (imu_healthy() && imu_visual_residual_window_.ready(robust_min_samples_)) {
      visual_yaw_disagreement_scale_ = disagreement_covariance_scale(
        imu_visual_robust_residual_, imu_soft_residual_, imu_visual_covariance_cap_);
      yaw_scale = std::max(yaw_scale, visual_yaw_disagreement_scale_);
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

    nav_msgs::msg::Odometry output = message;
    output.header.frame_id = world_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = raw_pose.x;
    output.pose.pose.position.y = raw_pose.y;
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation = yaw_quaternion(raw_pose.yaw);
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = visual_xy_variance_ * position_scale;
    output.pose.covariance[7] = visual_xy_variance_ * position_scale;
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = visual_yaw_variance_ * yaw_scale;
    output.twist.covariance.fill(0.0);
    for (std::size_t index = 0; index < 6; ++index) {
      output.twist.covariance[index * 6 + index] = 1.0e6;
    }
    visual_forward_velocity_ = raw_visual_vx_window_.median();
    visual_yaw_rate_ = raw_visual_wz_window_.median();
    visual_velocity_valid_ = true;
    visual_vx_window_.add(steady_seconds(), visual_forward_velocity_);
    visual_wz_window_.add(steady_seconds(), visual_yaw_rate_);
    output.twist.twist.linear.x = visual_forward_velocity_;
    output.twist.twist.linear.y = 0.0;
    output.twist.twist.linear.z = 0.0;
    output.twist.covariance[0] = visual_vx_variance_ * position_scale;
    visual_publisher_->publish(output);

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
    const bool raw_visual_reference = raw_visual_increment_healthy();
    const bool accepted_visual_reference =
      visual_vx_window_.ready(robust_min_samples_) && vision_healthy();
    if (raw_visual_reference || accepted_visual_reference) {
      const double visual_velocity = raw_visual_reference ?
        raw_visual_vx_window_.median() : visual_vx_window_.median();
      wheel_visual_residual_window_.add(
        sample_time, std::abs(velocity - visual_velocity));
      residual = wheel_visual_residual_window_.median();
      const double rejection_residual = wheel_visual_rejection_residual(
        velocity, visual_velocity,
        visual_stationary_speed_, stationary_wheel_reject_speed_);
      wheel_gate_.update(std::max(
        rejection_residual,
        residual + 1.4826 * wheel_visual_residual_window_.mad()));
      compared = true;
    }
    wheel_yaw_excited_ = false;
    wheel_yaw_instantly_consistent_ = false;
    if (wheel_yaw_rate_finite_ && imu_received_ &&
      age_seconds(imu_received_at_) <= imu_timeout_)
    {
      wheel_yaw_excited_ = yaw_rates_excited(
        wheel_yaw_rate, latest_imu_yaw_rate_, wheel_yaw_validation_min_rate_);
      wheel_yaw_instantly_consistent_ = yaw_rates_consistent(
        wheel_yaw_rate, latest_imu_yaw_rate_, wheel_imu_yaw_reject_);
      const bool either_yaw_excited =
        std::abs(wheel_yaw_rate) >= wheel_yaw_validation_min_rate_ ||
        std::abs(latest_imu_yaw_rate_) >= wheel_yaw_validation_min_rate_;
      if (wheel_yaw_excited_) {
        wheel_imu_yaw_residual_window_.add(
          sample_time, std::abs(wheel_yaw_rate - latest_imu_yaw_rate_));
      }
      if (wheel_yaw_excited_ && wheel_yaw_instantly_consistent_ &&
        wheel_imu_yaw_residual_window_.ready(robust_min_samples_))
      {
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
      } else if (either_yaw_excited) {
        wheel_yaw_validation_active_ = false;
        wheel_yaw_validated_ = false;
      } else if (!wheel_yaw_validated_) {
        // Zero-rate agreement is not evidence that the encoder yaw scale and
        // sign are valid. Require an actual turn before starting validation.
        wheel_yaw_validation_active_ = false;
      }
    }
    wheel_received_ = true;
    wheel_received_at_ = std::chrono::steady_clock::now();
    wheel_residual_ = compared ? residual : 0.0;
    const bool stationary_visual_conflict = raw_visual_reference &&
      std::abs(raw_visual_vx_window_.median()) <= visual_stationary_speed_ &&
      std::abs(velocity) > stationary_wheel_reject_speed_;
    if (wheel_gate_.rejected() || stationary_visual_conflict || !ever_accepted_visual_ ||
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
    const bool imu_fresh = imu_received_ && age_seconds(imu_received_at_) <= imu_timeout_;
    const bool command_fresh = command_received_ &&
      age_seconds(command_received_at_) <= command_timeout_;
    const double residual_scale = compared ?
      wheel_gate_.covariance_scale(residual, wheel_soft_residual_) : 2.0;
    const double turn_scale = wheel_vx_turn_covariance_scale(
      latest_imu_yaw_rate_, command_yaw_rate_, imu_fresh, command_fresh,
      wheel_turn_downweight_start_, wheel_turn_full_downweight_,
      wheel_turn_covariance_scale_);
    wheel_vx_turn_scale_ = turn_scale;
    const double input_vx_variance = message->twist.covariance[0];
    const double base_vx_variance = finite(input_vx_variance) && input_vx_variance > 0.0 ?
      std::max(wheel_vx_variance_, input_vx_variance) : wheel_vx_variance_;
    output.twist.covariance[0] = base_vx_variance * residual_scale * turn_scale;
    const bool wheel_yaw_backup = fuse_wheel_yaw_ && wheel_yaw_validated_ &&
      sample_time - wheel_yaw_last_validated_at_ <= wheel_yaw_backup_time_ &&
      std::abs(velocity) <= max_wheel_only_speed_;
    const bool wheel_yaw_matches_live_imu = imu_healthy() && wheel_yaw_instantly_consistent_;
    if (fuse_wheel_yaw_ && wheel_yaw_rate_finite_ && wheel_yaw_validated_ &&
      (wheel_yaw_matches_live_imu || wheel_yaw_backup))
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
    wheel_vx_zeroed_ = zero_wheel_vx_during_in_place_turn(
      command_velocity_, command_yaw_rate_, command_fresh,
      wheel_in_place_max_linear_speed_, wheel_in_place_min_yaw_rate_);
    if (wheel_vx_zeroed_) {
      output.twist.twist.linear.x = 0.0;
      output.twist.covariance[0] = base_vx_variance * residual_scale;
    }
    wheel_publisher_->publish(output);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const double yaw_rate = message->angular_velocity.z;
    if (message->header.frame_id != imu_expected_frame_ || !finite(yaw_rate) ||
      std::abs(yaw_rate) > max_imu_yaw_rate_)
    {
      ++invalid_imu_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting IMU with invalid wz/frame (wz=%.3f, %s; expected %s)",
        yaw_rate, message->header.frame_id.c_str(), imu_expected_frame_.c_str());
      return;
    }
    const rclcpp::Time message_stamp(message->header.stamp);
    if (imu_stamp_valid_ && message_stamp <= last_imu_message_stamp_) {
      ++invalid_imu_count_;
      return;
    }
    imu_stamp_valid_ = true;
    last_imu_message_stamp_ = message_stamp;
    const bool command_fresh = command_received_ &&
      age_seconds(command_received_at_) <= command_timeout_;
    const bool raw_visual_rate_ready = raw_visual_increment_healthy() &&
      raw_visual_wz_window_.ready(robust_min_samples_);
    const bool accepted_visual_rate_ready = visual_velocity_valid_ && vision_healthy() &&
      visual_wz_window_.ready(robust_min_samples_);
    const bool visual_rate_ready = raw_visual_rate_ready || accepted_visual_rate_ready;
    const double visual_rate_reference = visual_rate_ready ?
      (raw_visual_rate_ready ? raw_visual_wz_window_.median() : visual_wz_window_.median()) :
      visual_yaw_rate_;
    const bool learn_bias = visual_rate_ready && command_fresh &&
      std::abs(command_yaw_rate_) <= imu_bias_learning_command_rate_ &&
      std::abs(visual_rate_reference) <= imu_bias_learning_visual_rate_;
    latest_imu_yaw_rate_ = imu_bias_estimator_.correct(
      steady_seconds(), yaw_rate, visual_rate_reference, visual_rate_ready, learn_bias);
    double residual = 0.0;
    bool compared = false;
    if (visual_velocity_valid_ && vision_healthy()) {
      imu_visual_residual_window_.add(
        steady_seconds(), std::abs(latest_imu_yaw_rate_ - visual_yaw_rate_));
      residual = imu_visual_residual_window_.median();
      imu_visual_robust_residual_ =
        residual + 1.4826 * imu_visual_residual_window_.mad();
      compared = true;
    }
    imu_received_ = true;
    imu_received_at_ = std::chrono::steady_clock::now();
    imu_residual_ = compared ? residual : 0.0;
    if (!ever_accepted_visual_) {
      return;
    }

    sensor_msgs::msg::Imu output;
    output.header = message->header;
    output.header.frame_id = base_frame_;
    output.orientation_covariance[0] = -1.0;
    output.linear_acceleration_covariance[0] = -1.0;
    output.angular_velocity.z = latest_imu_yaw_rate_;
    output.angular_velocity_covariance.fill(0.0);
    output.angular_velocity_covariance[0] = 1.0e6;
    output.angular_velocity_covariance[4] = 1.0e6;
    const double source_variance = message->angular_velocity_covariance[8];
    const double base_variance = finite(source_variance) && source_variance > 0.0 ?
      std::max(imu_wz_variance_, source_variance) : imu_wz_variance_;
    output.angular_velocity_covariance[8] = base_variance;
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
    const bool raw_visual_motion_valid = raw_visual_increment_healthy();
    const bool accepted_visual_motion_valid =
      vision && visual_vx_window_.ready(robust_min_samples_);
    const bool visual_motion_valid = raw_visual_motion_valid || accepted_visual_motion_valid;
    const double visual_motion_velocity = raw_visual_motion_valid ?
      raw_visual_vx_window_.median() :
      (accepted_visual_motion_valid ? visual_vx_window_.median() : visual_forward_velocity_);
    motion_fault_ = motion_classifier_.update(
      steady_seconds(), command_velocity_,
      wheel_vx_window_.ready(robust_min_samples_) ? wheel_vx_window_.median() : latest_wheel_velocity_,
      visual_motion_velocity,
      command_fresh, wheel_measurement_fresh,
      visual_motion_valid);
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
    if (wheel && fuse_wheel_yaw_ && wheel_yaw_rate_finite_ && wheel_yaw_validated_) {
      measured_yaw_rate = std::max(measured_yaw_rate, std::abs(latest_wheel_yaw_rate_));
      ++angular_source_count;
    }
    angular_stalled_ = angular_stall_detector_.update(
      steady_seconds(), command_yaw_rate_, measured_yaw_rate,
      command_fresh, angular_source_count >= 2);
    const bool wheel_yaw_backup = fuse_wheel_yaw_ && wheel_yaw_validated_ &&
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
      "imu_yaw_bias_radps", std::to_string(imu_bias_estimator_.bias())));
    item.values.push_back(value(
      "visual_yaw_disagreement_scale", std::to_string(visual_yaw_disagreement_scale_)));
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
    item.values.push_back(value("wheel_yaw_excited", wheel_yaw_excited_ ? "true" : "false"));
    item.values.push_back(value(
      "wheel_yaw_instantly_consistent", wheel_yaw_instantly_consistent_ ? "true" : "false"));
    item.values.push_back(value("wheel_yaw_fused", fuse_wheel_yaw_ ? "true" : "false"));
    item.values.push_back(value("wheel_yaw_backup", wheel_yaw_backup ? "true" : "false"));
    item.values.push_back(value(
      "wheel_vx_turn_covariance_scale", std::to_string(wheel_vx_turn_scale_)));
    item.values.push_back(value(
      "wheel_vx_zeroed", wheel_vx_zeroed_ ? "true" : "false"));
    item.values.push_back(value(
      "raw_visual_increment", raw_visual_increment_healthy() ? "healthy" : "unavailable"));
    item.values.push_back(value(
      "raw_visual_vx_mps", std::to_string(raw_visual_forward_velocity_)));
    item.values.push_back(value("raw_visual_used", "true"));
    item.values.push_back(value("visual_realign_count", std::to_string(visual_realign_count_)));
    item.values.push_back(value(
      "visual_recovery_rejected", std::to_string(visual_recovery_rejected_count_)));
    item.values.push_back(value("map_change_sequence", std::to_string(map_change_sequence_)));
    item.values.push_back(value(
      "map_change_grace", steady_seconds() <= map_change_grace_until_ ? "true" : "false"));
    item.values.push_back(value("vision_outage_s", std::to_string(outage_time)));
    item.values.push_back(value("dead_reckoning_distance_m", std::to_string(dead_reckoning_distance_)));
    item.values.push_back(value(
      "wheel_frame_approximation",
      wheel_expected_child_frame_ == base_frame_ ? "false" : "true"));
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
  RobustWindow visual_vx_window_;
  RobustWindow visual_wz_window_;
  RobustWindow raw_visual_vx_window_;
  RobustWindow raw_visual_wz_window_;
  RobustWindow wheel_vx_window_;
  RobustWindow wheel_visual_residual_window_;
  RobustWindow wheel_imu_yaw_residual_window_;
  RobustWindow imu_visual_residual_window_;
  MotionClassifier motion_classifier_;
  AngularStallDetector angular_stall_detector_;

  std::string raw_visual_topic_;
  std::string tracking_topic_;
  std::string map_change_topic_;
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
  std::string visual_expected_child_frame_;
  std::string raw_visual_expected_frame_;
  std::string wheel_expected_child_frame_;
  std::string imu_expected_frame_;
  double visual_timeout_{0.4};
  bool publish_tf_{false};
  double tracking_timeout_{0.6};
  double wheel_timeout_{0.35};
  double imu_timeout_{0.15};
  double command_timeout_{0.4};
  double raw_visual_timeout_{0.3};
  double raw_visual_max_tilt_{0.5};
  double map_change_grace_{1.0};
  double max_dead_reckoning_time_{2.0};
  double max_dead_reckoning_distance_{0.3};
  double max_wheel_only_time_{2.0};
  double max_wheel_only_distance_{0.3};
  double max_wheel_only_speed_{0.3};
  double max_wheel_speed_{1.5};
  double max_wheel_yaw_rate_{4.0};
  double max_imu_yaw_rate_{4.0};
  double wheel_soft_residual_{0.15};
  double visual_stationary_speed_{0.025};
  double stationary_wheel_reject_speed_{0.03};
  double imu_soft_residual_{0.2};
  double imu_visual_covariance_cap_{0.8};
  double visual_ramp_duration_{0.75};
  double visual_ramp_initial_scale_{50.0};
  double wheel_imu_yaw_soft_{0.15};
  double wheel_imu_yaw_reject_{0.45};
  double wheel_yaw_validation_min_rate_{0.08};
  double wheel_yaw_validation_time_{2.0};
  double wheel_yaw_backup_time_{2.0};
  double wheel_vx_variance_{0.08};
  double wheel_turn_downweight_start_{0.15};
  double wheel_turn_full_downweight_{0.60};
  double wheel_turn_covariance_scale_{100.0};
  double wheel_in_place_max_linear_speed_{0.10};
  double wheel_in_place_min_yaw_rate_{0.30};
  double wheel_wz_variance_{0.5};
  double imu_wz_variance_{0.015};
  double imu_bias_time_constant_{3.0};
  double imu_bias_maximum_{0.08};
  double imu_bias_learning_command_rate_{0.08};
  double imu_bias_learning_visual_rate_{0.12};
  double visual_xy_variance_{0.02};
  double visual_yaw_variance_{0.04};
  double visual_vx_variance_{0.01};
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
  bool visual_velocity_valid_{false};
  bool dead_reckoning_active_{false};
  bool raw_visual_received_{false};
  bool raw_visual_stamp_valid_{false};
  bool command_received_{false};
  bool visual_ramp_active_{false};
  bool wheel_yaw_rate_finite_{false};
  bool wheel_yaw_validation_active_{false};
  bool wheel_yaw_validated_{false};
  bool wheel_yaw_excited_{false};
  bool wheel_yaw_instantly_consistent_{false};
  bool fuse_wheel_yaw_{false};
  bool wheel_vx_zeroed_{false};
  bool raw_visual_velocity_valid_{false};
  bool last_raw_increment_pose_valid_{false};
  bool angular_stalled_{false};
  bool wheel_stamp_valid_{false};
  bool imu_stamp_valid_{false};
  int tracking_state_{-1};
  std::size_t visual_recovery_count_{0};
  std::size_t invalid_wheel_count_{0};
  std::size_t invalid_imu_count_{0};
  std::size_t invalid_raw_visual_count_{0};
  std::size_t visual_realign_count_{0};
  std::size_t visual_recovery_rejected_count_{0};
  double visual_forward_velocity_{0.0};
  double visual_yaw_rate_{0.0};
  double wheel_residual_{0.0};
  double imu_residual_{0.0};
  double imu_visual_robust_residual_{0.0};
  double visual_yaw_disagreement_scale_{1.0};
  double dead_reckoning_distance_{0.0};
  double command_velocity_{0.0};
  double command_yaw_rate_{0.0};
  double latest_wheel_velocity_{0.0};
  double latest_wheel_yaw_rate_{0.0};
  double latest_imu_yaw_rate_{0.0};
  double raw_visual_forward_velocity_{0.0};
  double raw_visual_yaw_rate_{0.0};
  YawBiasEstimator imu_bias_estimator_{};
  double wheel_vx_turn_scale_{1.0};
  double wheel_yaw_validation_started_at_{0.0};
  double wheel_yaw_last_validated_at_{0.0};
  double visual_ramp_started_at_{0.0};
  double visual_realigned_until_{0.0};
  double map_change_grace_until_{0.0};
  uint64_t map_change_sequence_{0};
  Pose2d fused_pose_{};
  Pose2d last_raw_increment_pose_{};
  MotionFault motion_fault_{MotionFault::kNone};
  FusionMode last_mode_{FusionMode::kFull};
  SteadyTime tracking_received_at_{};
  SteadyTime visual_received_at_{};
  SteadyTime wheel_received_at_{};
  SteadyTime imu_received_at_{};
  SteadyTime dead_reckoning_since_{};
  SteadyTime raw_visual_received_at_{};
  SteadyTime command_received_at_{};
  rclcpp::Time last_wheel_message_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_message_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_raw_visual_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_raw_increment_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr visual_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_visual_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracking_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr map_change_subscription_;
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
