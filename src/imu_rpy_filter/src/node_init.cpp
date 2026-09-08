#include "imu_rpy_filter/imu_rpy_filter_node.hpp"

namespace imu_rpy_filter {

ImuRpyFilterNode::ImuRpyFilterNode()
    : Node("imu_rpy_filter"),
      median_gyro_(declare_positive_integer("median_window", 5)),
      median_accel_(
          static_cast<std::size_t>(get_parameter("median_window").as_int())),
      mean_gyro_(declare_positive_integer("mean_window", 5)),
      mean_accel_(
          static_cast<std::size_t>(get_parameter("mean_window").as_int())),
      low_pass_gyro_(declare_nonnegative("low_pass_cutoff_hz", 5.0)),
      low_pass_accel_(get_parameter("low_pass_cutoff_hz").as_double()),
      stationary_detector_(
          declare_nonnegative("stationary_dwell_seconds", 0.5),
          declare_nonnegative("stationary_exit_dwell_seconds", 0.15),
          declare_nonnegative("stationary_accel_enter", 0.20),
          declare_nonnegative("stationary_accel_exit", 0.35),
          declare_nonnegative("stationary_gyro_enter", 0.02),
          declare_nonnegative("stationary_gyro_exit", 0.05)),
      yaw_filter_(radians(declare_nonnegative("kalman_gyro_noise_deg_s", 0.6)),
                  radians(declare_nonnegative("kalman_bias_walk_deg_s", 0.03))),
      stationary_bias_window_(
          declare_positive("stationary_bias_window_seconds", 0.5)) {
  imu_topic_ =
      declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
  rpy_topic_ = declare_parameter<std::string>("rpy_topic", "/imu/rpy");
  rpy_degrees_topic_ =
      declare_parameter<std::string>("rpy_degrees_topic", "/imu/rpy_degrees");
  filtered_imu_topic_ =
      declare_parameter<std::string>("filtered_imu_topic", "/imu/filtered");
  output_frame_ = declare_parameter<std::string>("output_frame", "camera_link");
  input_frame_ = declare_parameter<std::string>("input_frame", "camera_link");
  correction_time_ = declare_positive("tilt_correction_time", 0.5);
  transform_optical_frame_ =
      declare_parameter<bool>("transform_optical_frame", true);
  use_yaw_reference_ = declare_parameter<bool>("use_yaw_reference", false);
  yaw_reference_topic_ = declare_parameter<std::string>(
      "yaw_reference_topic", "/odometry/visual_continuous");
  tracking_topic_ =
      declare_parameter<std::string>("tracking_topic", "/tracking_state");
  command_topic_ =
      declare_parameter<std::string>("command_topic", "/cmd_vel_nav");
  wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/wheel/odom");
  external_motion_timeout_ =
      declare_positive("external_motion_timeout_seconds", 0.35);
  command_linear_stationary_threshold_ =
      declare_nonnegative("command_linear_stationary_threshold_mps", 0.02);
  command_yaw_stationary_threshold_ =
      declare_nonnegative("command_yaw_stationary_threshold_rad_s", 0.03);
  wheel_linear_stationary_threshold_ =
      declare_nonnegative("wheel_linear_stationary_threshold_mps", 0.02);
  wheel_yaw_stationary_threshold_ =
      declare_nonnegative("wheel_yaw_stationary_threshold_rad_s", 0.05);
  visual_yaw_std_ = radians(declare_positive("visual_yaw_std_degrees", 2.0));
  visual_yaw_gate_ = radians(declare_positive("visual_yaw_gate_degrees", 15.0));
  stationary_yaw_std_ =
      radians(declare_positive("stationary_yaw_std_degrees", 0.5));
  stationary_bias_std_ =
      radians(declare_positive("stationary_bias_std_deg_s", 0.15));
  moving_yaw_rate_variance_ =
      std::pow(radians(declare_positive("moving_yaw_rate_std_deg_s", 1.0)), 2);
  stationary_yaw_rate_variance_ = std::pow(
      radians(declare_positive("stationary_yaw_rate_std_deg_s", 0.15)), 2);
  roll_pitch_variance_ =
      std::pow(radians(declare_positive("roll_pitch_std_degrees", 1.0)), 2);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  rpy_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rpy_topic_, sensor_qos);
  rpy_degrees_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rpy_degrees_topic_, sensor_qos);
  filtered_imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>(filtered_imu_topic_, sensor_qos);
  diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
                                                              10);
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&ImuRpyFilterNode::imu_callback, this, std::placeholders::_1));
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&ImuRpyFilterNode::command_callback, this,
                std::placeholders::_1));
  wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, sensor_qos,
      std::bind(&ImuRpyFilterNode::wheel_callback, this,
                std::placeholders::_1));

  if (use_yaw_reference_) {
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        yaw_reference_topic_, sensor_qos,
        std::bind(&ImuRpyFilterNode::odometry_callback, this,
                  std::placeholders::_1));
    tracking_subscription_ = create_subscription<std_msgs::msg::Int32>(
        tracking_topic_, sensor_qos,
        std::bind(&ImuRpyFilterNode::tracking_callback, this,
                  std::placeholders::_1));
  }
  diagnostic_timer_ = rclcpp::create_timer(this, get_clock(),
                                           rclcpp::Duration::from_seconds(0.5),
                                           [this]() { publish_diagnostics(); });

  RCLCPP_INFO(
      get_logger(),
      "IMU RPY filter ready: input=%s rpy=%s filtered_imu=%s visual_yaw=%s",
      imu_topic_.c_str(), rpy_topic_.c_str(), filtered_imu_topic_.c_str(),
      use_yaw_reference_ ? "enabled" : "disabled");
}

} // namespace imu_rpy_filter
