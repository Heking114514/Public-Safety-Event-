#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

double FusionGateNode::positive(const std::string &name, double default_value) {
  const double result = declare_parameter<double>(name, default_value);
  if (!finite(result) || result <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
  return result;
}

double FusionGateNode::nonnegative(const std::string &name,
                                   double default_value) {
  const double result = declare_parameter<double>(name, default_value);
  if (!finite(result) || result < 0.0) {
    throw std::invalid_argument(name + " must be finite and non-negative");
  }
  return result;
}

bool FusionGateNode::accept_measurement_stamp(
    const builtin_interfaces::msg::Time &stamp_message, StampState &stamp_state,
    double max_age, const char *source) {
  const rclcpp::Time stamp(stamp_message);
  const rclcpp::Time current = now();
  const auto result = ValidateMeasurementStamp(
      stamp.nanoseconds(), stamp_state.valid, stamp_state.last.nanoseconds(),
      current.nanoseconds(), max_age, stamp_future_tolerance_);
  if (result != StampValidation::kAccepted) {
    if (current.nanoseconds() > 0 && stamp.nanoseconds() > 0) {
      const double age = (current - stamp).seconds();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting %s measurement with age %.3fs (validation=%d)", source,
          age, static_cast<int>(result));
    }
    return false;
  }
  stamp_state.last = stamp;
  stamp_state.valid = true;
  return true;
}

std::size_t FusionGateNode::positive_count(const std::string &name,
                                           std::int64_t default_value) {
  const auto result = declare_parameter<std::int64_t>(name, default_value);
  if (result < 1) {
    throw std::invalid_argument(name + " must be positive");
  }
  return static_cast<std::size_t>(result);
}

FusionGateNode::FusionGateNode()
    : Node("fused_odometry_gate"),
      wheel_gate_(positive("wheel_visual_reject_mps", 0.45),
                  positive_count("residual_bad_samples", 3),
                  positive_count("residual_recovery_samples", 10)),
      visual_vx_window_(positive("consistency_window_s", 0.75)),
      visual_wz_window_(get_parameter("consistency_window_s").as_double()),
      raw_visual_vx_window_(get_parameter("consistency_window_s").as_double()),
      raw_visual_wz_window_(get_parameter("consistency_window_s").as_double()),
      wheel_vx_window_(get_parameter("consistency_window_s").as_double()),
      wheel_visual_residual_window_(
          get_parameter("consistency_window_s").as_double()),
      wheel_imu_yaw_residual_window_(
          get_parameter("consistency_window_s").as_double()),
      imu_visual_residual_window_(
          get_parameter("consistency_window_s").as_double()),
      health_monitor_(
          MotionClassifierConfig{
              positive("command_motion_threshold_mps", 0.08),
              positive("motion_moving_threshold_mps", 0.04),
              positive("motion_stationary_threshold_mps", 0.02),
              positive("motion_fault_dwell_s", 0.6),
              positive("motion_recovery_dwell_s", 1.0)},
          AngularStallConfig{
              positive("angular_stall_command_threshold_radps", 0.30),
              positive("angular_stall_stationary_threshold_radps", 0.10),
              positive("angular_stall_dwell_s", 0.8),
              positive("angular_stall_recovery_dwell_s", 1.0)}) {
  raw_visual_topic_ = declare_parameter<std::string>("raw_visual_topic",
                                                     "/odometry/visual_raw");
  tracking_topic_ =
      declare_parameter<std::string>("tracking_topic", "/tracking_state");
  map_change_topic_ = declare_parameter<std::string>("map_change_topic",
                                                     "/orbslam3/map_change");
  wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/wheel/odom");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/filtered");
  command_topic_ =
      declare_parameter<std::string>("command_topic", "/cmd_vel_nav");
  visual_output_topic_ = declare_parameter<std::string>(
      "visual_output_topic", "/fusion/input/visual_odom");
  wheel_output_topic_ = declare_parameter<std::string>(
      "wheel_output_topic", "/fusion/input/wheel_odom");
  // Public control feedback: both EKF and navigation must consume the same
  // bias-corrected yaw-rate sample produced below.
  imu_output_topic_ =
      declare_parameter<std::string>("imu_output_topic", "/imu/control");
  status_topic_ =
      declare_parameter<std::string>("status_topic", "/odometry/fusion_status");
  diagnostics_topic_ =
      declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  world_frame_ = declare_parameter<std::string>("world_frame", "map");
  // Wheel velocity is retained for supervision and bag diagnostics only.
  // Preserve its native frame rather than relabeling the recorded stream.
  wheel_frame_ = declare_parameter<std::string>("wheel_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  visual_expected_child_frame_ = declare_parameter<std::string>(
      "visual_expected_child_frame", "base_link");
  raw_visual_expected_frame_ =
      declare_parameter<std::string>("raw_visual_expected_frame", "map");
  wheel_expected_child_frame_ =
      declare_parameter<std::string>("wheel_expected_child_frame", "base_link");
  wheel_expected_frame_ =
      declare_parameter<std::string>("wheel_expected_frame", "odom");
  imu_expected_frame_ =
      declare_parameter<std::string>("imu_expected_frame", "camera_link");
  const bool deprecated_publish_tf =
      declare_parameter<bool>("publish_tf", false);
  if (deprecated_publish_tf) {
    RCLCPP_WARN(get_logger(), "fusion_gate publish_tf is ignored; "
                              "map_odom_correction owns final odometry TF");
  }

  visual_timeout_ = positive("visual_timeout_s", 0.4);
  tracking_timeout_ = positive("tracking_timeout_s", 0.6);
  wheel_timeout_ = positive("wheel_timeout_s", 0.35);
  imu_timeout_ = positive("imu_timeout_s", 0.15);
  command_timeout_ = positive("command_timeout_s", 0.4);
  raw_visual_timeout_ = positive("raw_visual_timeout_s", 0.3);
  raw_visual_max_tilt_ = positive("raw_visual_max_tilt_rad", 0.50);
  stamp_future_tolerance_ = nonnegative("stamp_future_tolerance_s", 0.20);
  // ORB needs a short warm-up and several coherent increments. Keep startup
  // recoverable for that interval, then expose a distinct fault if it never
  // becomes usable.
  initialization_timeout_ = positive("initialization_timeout_s", 10.0);
  initialization_started_at_ = steady_seconds();
  map_change_grace_ = positive("map_change_grace_s", 1.0);
  max_dead_reckoning_time_ = positive("max_dead_reckoning_time_s", 2.0);
  max_dead_reckoning_distance_ =
      positive("max_dead_reckoning_distance_m", 0.30);
  max_wheel_only_time_ = positive("max_wheel_only_time_s", 2.0);
  max_wheel_only_distance_ = positive("max_wheel_only_distance_m", 0.30);
  max_wheel_only_speed_ = positive("max_wheel_only_speed_mps", 0.30);
  max_wheel_speed_ = positive("max_wheel_speed_mps", 1.0);
  max_wheel_yaw_rate_ = positive("max_wheel_yaw_rate_radps", 4.0);
  max_imu_yaw_rate_ = positive("max_imu_yaw_rate_radps", 4.0);
  wheel_soft_residual_ = positive("wheel_visual_soft_mps", 0.15);
  visual_stationary_speed_ = positive("visual_stationary_speed_mps", 0.025);
  stationary_wheel_reject_speed_ =
      positive("stationary_wheel_reject_speed_mps", 0.03);
  stationary_command_yaw_speed_ =
      positive("stationary_command_yaw_speed_radps", 0.12);
  imu_soft_residual_ = positive("imu_visual_soft_radps", 0.20);
  imu_visual_covariance_cap_ =
      positive("imu_visual_covariance_cap_radps", 0.80);
  visual_ramp_duration_ = positive("visual_recovery_ramp_s", 0.75);
  visual_ramp_initial_scale_ =
      positive("visual_recovery_initial_covariance_scale", 50.0);
  wheel_imu_yaw_soft_ = positive("wheel_imu_yaw_soft_radps", 0.15);
  wheel_imu_yaw_reject_ = positive("wheel_imu_yaw_reject_radps", 0.45);
  wheel_yaw_validation_min_rate_ =
      positive("wheel_yaw_validation_min_rate_radps", 0.08);
  wheel_yaw_validation_time_ = positive("wheel_yaw_validation_s", 2.0);
  wheel_yaw_backup_time_ = positive("wheel_yaw_backup_s", 2.0);
  fuse_wheel_yaw_ = declare_parameter<bool>("fuse_wheel_yaw", false);
  wheel_vx_variance_ = positive("wheel_vx_variance", 0.08);
  wheel_turn_downweight_start_ =
      positive("wheel_turn_downweight_start_radps", 0.15);
  wheel_turn_full_downweight_ =
      positive("wheel_turn_full_downweight_radps", 0.60);
  if (wheel_turn_full_downweight_ <= wheel_turn_downweight_start_) {
    throw std::invalid_argument("wheel_turn_full_downweight_radps must exceed "
                                "wheel_turn_downweight_start_radps");
  }
  wheel_turn_covariance_scale_ = positive("wheel_turn_covariance_scale", 100.0);
  wheel_in_place_max_linear_speed_ =
      positive("wheel_in_place_max_linear_speed_mps", 0.10);
  wheel_in_place_min_yaw_rate_ =
      positive("wheel_in_place_min_yaw_rate_radps", 0.30);
  wheel_wz_variance_ = positive("wheel_wz_variance", 0.50);
  imu_wz_variance_ = positive("imu_wz_variance", 0.015);
  imu_bias_time_constant_ = positive("imu_bias_time_constant_s", 3.0);
  imu_bias_maximum_ = positive("imu_bias_max_radps", 0.08);
  imu_bias_learning_command_rate_ =
      positive("imu_bias_learning_max_command_radps", 0.08);
  imu_bias_learning_visual_rate_ =
      positive("imu_bias_learning_max_visual_rate_radps", 0.12);
  imu_.bias_estimator =
      YawBiasEstimator(imu_bias_time_constant_, imu_bias_maximum_);
  visual_xy_variance_ = positive("visual_xy_variance", 0.02);
  visual_yaw_variance_ = positive("visual_yaw_variance", 0.04);
  visual_vx_variance_ = positive("visual_vx_variance", 0.01);
  recovery_samples_ = positive_count("visual_recovery_samples", 5);
  robust_min_samples_ = positive_count("robust_min_samples", 5);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  visual_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      visual_output_topic_, sensor_qos);
  wheel_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      wheel_output_topic_, sensor_qos);
  imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>(imu_output_topic_, sensor_qos);
  status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
  diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          diagnostics_topic_, 10);
  raw_visual_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_visual_topic_, sensor_qos,
      std::bind(&FusionGateNode::raw_visual_callback, this,
                std::placeholders::_1));
  tracking_subscription_ = create_subscription<std_msgs::msg::Int32>(
      tracking_topic_, sensor_qos,
      std::bind(&FusionGateNode::tracking_callback, this,
                std::placeholders::_1));
  map_change_subscription_ = create_subscription<std_msgs::msg::UInt64>(
      map_change_topic_, 10,
      std::bind(&FusionGateNode::map_change_callback, this,
                std::placeholders::_1));
  wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, sensor_qos,
      std::bind(&FusionGateNode::wheel_callback, this, std::placeholders::_1));
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&FusionGateNode::imu_callback, this, std::placeholders::_1));
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&FusionGateNode::command_callback, this,
                std::placeholders::_1));

  status_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1),
      std::bind(&FusionGateNode::publish_status, this));
  RCLCPP_INFO(get_logger(),
              "Fusion gate ready: visual=%s wheel=%s imu=%s (%s/%s)",
              raw_visual_topic_.c_str(), wheel_topic_.c_str(),
              imu_topic_.c_str(), world_frame_.c_str(), base_frame_.c_str());
}

} // namespace fused_odometry
