#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"

#include "imu_rpy_filter/yaw_filter.hpp"

namespace imu_rpy_filter
{

constexpr double kGravity = 9.80665;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double degrees(double angle)
{
  return angle * 180.0 / kPi;
}

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;

Vector3 optical_to_body(const Vector3 & value)
{
  return {value[2], -value[0], -value[1]};
}

Matrix3 optical_covariance_to_body(const Matrix3 & covariance)
{
  constexpr Matrix3 rotation = {
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0};
  Matrix3 temporary{};
  Matrix3 transformed{};

  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t inner = 0; inner < 3; ++inner) {
        temporary[row * 3 + column] +=
          rotation[row * 3 + inner] * covariance[inner * 3 + column];
      }
    }
  }
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t inner = 0; inner < 3; ++inner) {
        transformed[row * 3 + column] +=
          temporary[row * 3 + inner] * rotation[column * 3 + inner];
      }
    }
  }
  return transformed;
}

double vector_norm(const Vector3 & value)
{
  return std::sqrt(
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

bool finite_vector(const Vector3 & value)
{
  return std::all_of(value.begin(), value.end(), [](double item) {
    return std::isfinite(item);
  });
}

bool finite_matrix(const Matrix3 & value)
{
  return std::all_of(value.begin(), value.end(), [](double item) {
    return std::isfinite(item);
  });
}

bool accepted_imu_frame(
  const std::string & frame, const std::string & expected_frame)
{
  if (frame.empty()) {
    return false;
  }
  if (frame == expected_frame) {
    return true;
  }
  constexpr char optical_suffix[] = "_optical_frame";
  const std::size_t suffix_length = sizeof(optical_suffix) - 1;
  return frame.size() >= suffix_length &&
    frame.compare(frame.size() - suffix_length, suffix_length, optical_suffix) == 0;
}

class VectorMedianFilter
{
public:
  explicit VectorMedianFilter(std::size_t window_size)
  : window_size_(std::max<std::size_t>(1, window_size))
  {
  }

  Vector3 update(const Vector3 & sample)
  {
    samples_.push_back(sample);
    if (samples_.size() > window_size_) {
      samples_.pop_front();
    }

    Vector3 result{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      std::vector<double> values;
      values.reserve(samples_.size());
      for (const auto & item : samples_) {
        values.push_back(item[axis]);
      }
      const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
      std::nth_element(values.begin(), middle, values.end());
      result[axis] = *middle;
      if (values.size() % 2 == 0) {
        const auto lower = std::max_element(values.begin(), middle);
        result[axis] = 0.5 * (result[axis] + *lower);
      }
    }
    return result;
  }

private:
  std::size_t window_size_;
  std::deque<Vector3> samples_;
};

class VectorMeanFilter
{
public:
  explicit VectorMeanFilter(std::size_t window_size)
  : window_size_(std::max<std::size_t>(1, window_size))
  {
  }

  Vector3 update(const Vector3 & sample)
  {
    samples_.push_back(sample);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sums_[axis] += sample[axis];
    }
    if (samples_.size() > window_size_) {
      const auto oldest = samples_.front();
      samples_.pop_front();
      for (std::size_t axis = 0; axis < 3; ++axis) {
        sums_[axis] -= oldest[axis];
      }
    }

    Vector3 result{};
    const double count = static_cast<double>(samples_.size());
    for (std::size_t axis = 0; axis < 3; ++axis) {
      result[axis] = sums_[axis] / count;
    }
    return result;
  }

private:
  std::size_t window_size_;
  std::deque<Vector3> samples_;
  Vector3 sums_{};
};

class LowPassFilter
{
public:
  explicit LowPassFilter(double cutoff_hz)
  : time_constant_(cutoff_hz > 0.0 ? 1.0 / (2.0 * kPi * cutoff_hz) : 0.0)
  {
  }

  Vector3 update(const Vector3 & sample, double dt)
  {
    if (!initialized_) {
      value_ = sample;
      initialized_ = true;
      return value_;
    }
    const double weight = time_constant_ > 0.0 ? dt / (time_constant_ + dt) : 1.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      value_[axis] += weight * (sample[axis] - value_[axis]);
    }
    return value_;
  }

private:
  double time_constant_;
  bool initialized_{false};
  Vector3 value_{};
};

class TimedScalarMedian
{
public:
  explicit TimedScalarMedian(double duration_s)
  : duration_s_(duration_s)
  {
  }

  void add(double stamp, double value)
  {
    samples_.emplace_back(stamp, value);
    while (!samples_.empty() && stamp - samples_.front().first > duration_s_) {
      samples_.pop_front();
    }
  }

  void clear() {samples_.clear();}

  bool ready(std::size_t minimum_samples) const
  {
    return samples_.size() >= minimum_samples &&
           samples_.back().first - samples_.front().first >= 0.5 * duration_s_;
  }

  double median() const
  {
    std::vector<double> values;
    values.reserve(samples_.size());
    for (const auto & sample : samples_) {
      values.push_back(sample.second);
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

private:
  double duration_s_;
  std::deque<std::pair<double, double>> samples_;
};

geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = cr * cp * cy + sr * sp * sy;
  quaternion.x = sr * cp * cy - cr * sp * sy;
  quaternion.y = cr * sp * cy + sr * cp * sy;
  quaternion.z = cr * cp * sy - sr * sp * cy;
  return quaternion;
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

double euler_yaw_rate(const Vector3 & gyro, double roll, double pitch)
{
  double cos_pitch = std::cos(pitch);
  if (std::abs(cos_pitch) < 1e-3) {
    cos_pitch = std::copysign(1e-3, cos_pitch);
  }
  return std::sin(roll) / cos_pitch * gyro[1] +
    std::cos(roll) / cos_pitch * gyro[2];
}

class ImuRpyFilterNode : public rclcpp::Node
{
public:
  ImuRpyFilterNode()
  : Node("imu_rpy_filter"),
    median_gyro_(declare_positive_integer("median_window", 5)),
    median_accel_(static_cast<std::size_t>(get_parameter("median_window").as_int())),
    mean_gyro_(declare_positive_integer("mean_window", 5)),
    mean_accel_(static_cast<std::size_t>(get_parameter("mean_window").as_int())),
    low_pass_gyro_(declare_nonnegative("low_pass_cutoff_hz", 5.0)),
    low_pass_accel_(get_parameter("low_pass_cutoff_hz").as_double()),
    stationary_detector_(
      declare_nonnegative("stationary_dwell_seconds", 0.5),
      declare_nonnegative("stationary_exit_dwell_seconds", 0.15),
      declare_nonnegative("stationary_accel_enter", 0.20),
      declare_nonnegative("stationary_accel_exit", 0.35),
      declare_nonnegative("stationary_gyro_enter", 0.02),
      declare_nonnegative("stationary_gyro_exit", 0.05)),
    yaw_filter_(
      radians(declare_nonnegative("kalman_gyro_noise_deg_s", 0.6)),
      radians(declare_nonnegative("kalman_bias_walk_deg_s", 0.03))),
    stationary_bias_window_(declare_positive("stationary_bias_window_seconds", 0.5))
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
    rpy_topic_ = declare_parameter<std::string>("rpy_topic", "/imu/rpy");
    rpy_degrees_topic_ =
      declare_parameter<std::string>("rpy_degrees_topic", "/imu/rpy_degrees");
    filtered_imu_topic_ =
      declare_parameter<std::string>("filtered_imu_topic", "/imu/filtered");
    output_frame_ = declare_parameter<std::string>("output_frame", "camera_link");
    input_frame_ = declare_parameter<std::string>("input_frame", "camera_link");
    correction_time_ = declare_positive("tilt_correction_time", 0.5);
    transform_optical_frame_ = declare_parameter<bool>("transform_optical_frame", true);
    use_yaw_reference_ = declare_parameter<bool>("use_yaw_reference", false);
    yaw_reference_topic_ = declare_parameter<std::string>(
      "yaw_reference_topic", "/odometry/visual_continuous");
    tracking_topic_ = declare_parameter<std::string>("tracking_topic", "/tracking_state");
    command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel_nav");
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/wheel/odom");
    external_motion_timeout_ = declare_positive("external_motion_timeout_seconds", 0.35);
    command_linear_stationary_threshold_ = declare_nonnegative(
      "command_linear_stationary_threshold_mps", 0.02);
    command_yaw_stationary_threshold_ = declare_nonnegative(
      "command_yaw_stationary_threshold_rad_s", 0.03);
    wheel_linear_stationary_threshold_ = declare_nonnegative(
      "wheel_linear_stationary_threshold_mps", 0.02);
    wheel_yaw_stationary_threshold_ = declare_nonnegative(
      "wheel_yaw_stationary_threshold_rad_s", 0.05);
    visual_yaw_std_ = radians(declare_positive("visual_yaw_std_degrees", 2.0));
    visual_yaw_gate_ = radians(declare_positive("visual_yaw_gate_degrees", 15.0));
    stationary_yaw_std_ = radians(declare_positive("stationary_yaw_std_degrees", 0.5));
    stationary_bias_std_ = radians(
      declare_positive("stationary_bias_std_deg_s", 0.15));
    moving_yaw_rate_variance_ = std::pow(
      radians(declare_positive("moving_yaw_rate_std_deg_s", 1.0)), 2);
    stationary_yaw_rate_variance_ = std::pow(
      radians(declare_positive("stationary_yaw_rate_std_deg_s", 0.15)), 2);
    roll_pitch_variance_ = std::pow(
      radians(declare_positive("roll_pitch_std_degrees", 1.0)), 2);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    rpy_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rpy_topic_, sensor_qos);
    rpy_degrees_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rpy_degrees_topic_, sensor_qos);
    filtered_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      filtered_imu_topic_, sensor_qos);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&ImuRpyFilterNode::imu_callback, this, std::placeholders::_1));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&ImuRpyFilterNode::command_callback, this, std::placeholders::_1));
    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, sensor_qos,
      std::bind(&ImuRpyFilterNode::wheel_callback, this, std::placeholders::_1));

    if (use_yaw_reference_) {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        yaw_reference_topic_, sensor_qos,
        std::bind(&ImuRpyFilterNode::odometry_callback, this, std::placeholders::_1));
      tracking_subscription_ = create_subscription<std_msgs::msg::Int32>(
        tracking_topic_, sensor_qos,
        std::bind(&ImuRpyFilterNode::tracking_callback, this, std::placeholders::_1));
    }
    diagnostic_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.5),
      [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(),
      "IMU RPY filter ready: input=%s rpy=%s filtered_imu=%s visual_yaw=%s",
      imu_topic_.c_str(), rpy_topic_.c_str(), filtered_imu_topic_.c_str(),
      use_yaw_reference_ ? "enabled" : "disabled");
  }

private:
  std::size_t declare_positive_integer(const std::string & name, std::int64_t default_value)
  {
    const std::int64_t value = declare_parameter<std::int64_t>(name, default_value);
    if (value < 1) {
      throw std::invalid_argument(name + " must be at least 1");
    }
    return static_cast<std::size_t>(value);
  }

  double declare_positive(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (value <= 0.0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return value;
  }

  double declare_nonnegative(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (value < 0.0) {
      throw std::invalid_argument(name + " cannot be negative");
    }
    return value;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_reference_yaw_ = quaternion_yaw(message->pose.pose.orientation);
    ++reference_sequence_;
    reference_received_at_ = std::chrono::steady_clock::now();
    have_reference_ = true;
  }

  void tracking_callback(const std_msgs::msg::Int32::SharedPtr message)
  {
    tracking_state_ = message->data;
    tracking_received_at_ = std::chrono::steady_clock::now();
    have_tracking_state_ = true;
  }

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    latest_command_linear_ = message->linear.x;
    latest_command_yaw_rate_ = message->angular.z;
    command_received_at_ = std::chrono::steady_clock::now();
    have_command_ = std::isfinite(latest_command_linear_) &&
      std::isfinite(latest_command_yaw_rate_);
  }

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_wheel_linear_ = message->twist.twist.linear.x;
    latest_wheel_yaw_rate_ = message->twist.twist.angular.z;
    wheel_received_at_ = std::chrono::steady_clock::now();
    have_wheel_ = std::isfinite(latest_wheel_linear_) &&
      std::isfinite(latest_wheel_yaw_rate_);
  }

  void update_external_motion_diagnostics()
  {
    const auto current = std::chrono::steady_clock::now();
    const auto observation = collect_external_motion_diagnostics(
      command_fresh(current), wheel_fresh(current), latest_command_linear_,
      latest_command_yaw_rate_, latest_wheel_linear_, latest_wheel_yaw_rate_,
      command_linear_stationary_threshold_, command_yaw_stationary_threshold_,
      wheel_linear_stationary_threshold_, wheel_yaw_stationary_threshold_);
    external_stationary_ = observation.stationary;
    external_moving_ = observation.moving;
  }

  bool command_fresh(const std::chrono::steady_clock::time_point & current) const
  {
    return have_command_ && std::chrono::duration<double>(
      current - command_received_at_).count() <= external_motion_timeout_;
  }

  bool wheel_fresh(const std::chrono::steady_clock::time_point & current) const
  {
    return have_wheel_ && std::chrono::duration<double>(
      current - wheel_received_at_).count() <= external_motion_timeout_;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    Vector3 gyro = {
      message->angular_velocity.x,
      message->angular_velocity.y,
      message->angular_velocity.z};
    Vector3 accel = {
      message->linear_acceleration.x,
      message->linear_acceleration.y,
      message->linear_acceleration.z};
    Matrix3 gyro_covariance = message->angular_velocity_covariance;
    Matrix3 accel_covariance = message->linear_acceleration_covariance;

    if (!std::isfinite(stamp) || stamp <= 0.0 ||
      !accepted_imu_frame(message->header.frame_id, input_frame_) ||
      !finite_vector(gyro) || !finite_vector(accel) ||
      !finite_matrix(gyro_covariance) || !finite_matrix(accel_covariance))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting IMU with invalid timestamp, frame, value or covariance");
      return;
    }

    if (initialized_) {
      const double dt = stamp - last_stamp_;
      if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting IMU dt %.6f seconds without updating filter history", dt);
        return;
      }
    }

    const bool input_is_optical = transform_optical_frame_ &&
      message->header.frame_id.size() >= 14 &&
      message->header.frame_id.compare(
      message->header.frame_id.size() - 14, 14, "_optical_frame") == 0;
    const bool optical_frame = message->header.frame_id.size() >= 14 &&
      message->header.frame_id.compare(
      message->header.frame_id.size() - 14, 14, "_optical_frame") == 0;
    if (optical_frame && !transform_optical_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting optical-frame IMU because transform_optical_frame is disabled");
      return;
    }
    if (input_is_optical) {
      gyro = optical_to_body(gyro);
      accel = optical_to_body(accel);
      gyro_covariance = optical_covariance_to_body(gyro_covariance);
      accel_covariance = optical_covariance_to_body(accel_covariance);
    }

    gyro = mean_gyro_.update(median_gyro_.update(gyro));
    accel = mean_accel_.update(median_accel_.update(accel));

    if (!initialized_) {
      gyro = low_pass_gyro_.update(gyro, 0.005);
      accel = low_pass_accel_.update(accel, 0.005);
      roll_ = std::atan2(accel[1], accel[2]);
      pitch_ = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));
      last_stamp_ = stamp;
      initialized_ = true;
      publish_outputs(
        *message, gyro, accel, gyro_covariance, accel_covariance, false, false,
        euler_yaw_rate(gyro, roll_, pitch_));
      return;
    }

    const double dt = stamp - last_stamp_;
    last_stamp_ = stamp;
    gyro = low_pass_gyro_.update(gyro, dt);
    accel = low_pass_accel_.update(accel, dt);
    const double accel_norm = vector_norm(accel);
    const double roll_accel = std::atan2(accel[1], accel[2]);
    const double pitch_accel = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));

    double cos_pitch = std::cos(pitch_);
    if (std::abs(cos_pitch) < 1e-3) {
      cos_pitch = std::copysign(1e-3, cos_pitch);
    }
    const double tan_pitch = std::sin(pitch_) / cos_pitch;
    const double roll_rate = gyro[0] + std::sin(roll_) * tan_pitch * gyro[1] +
      std::cos(roll_) * tan_pitch * gyro[2];
    const double pitch_rate = std::cos(roll_) * gyro[1] - std::sin(roll_) * gyro[2];
    // Convert the camera-frame angular velocity to Euler yaw rate. The D455 is
    // mounted with a fixed pitch, so using gyro.z directly undercounts turns.
    const double yaw_rate = euler_yaw_rate(gyro, roll_, pitch_);

    roll_ = wrap_angle(roll_ + roll_rate * dt);
    pitch_ = wrap_angle(pitch_ + pitch_rate * dt);
    yaw_filter_.predict(yaw_rate, dt);

    const double acceleration_error = std::abs(accel_norm - kGravity);
    const double acceleration_confidence = std::clamp(
      (0.75 - acceleration_error) / 0.5, 0.0, 1.0);
    if (acceleration_confidence > 0.0) {
      const double gyro_weight = correction_time_ / (correction_time_ + dt);
      const double accel_weight = (1.0 - gyro_weight) * acceleration_confidence;
      roll_ = wrap_angle(roll_ + accel_weight * wrap_angle(roll_accel - roll_));
      pitch_ = wrap_angle(pitch_ + accel_weight * wrap_angle(pitch_accel - pitch_));
    }

    const double corrected_yaw_rate = bias_corrected_yaw_rate(yaw_rate, yaw_filter_.bias());
    const double angular_rate_norm = std::sqrt(
      gyro[0] * gyro[0] + gyro[1] * gyro[1] +
      corrected_yaw_rate * corrected_yaw_rate);
    update_external_motion_diagnostics();
    const bool stationary = stationary_detector_.update(
      stamp, acceleration_error, angular_rate_norm);
    const bool stationary_like = stationary || stationary_detector_.has_candidate();
    const bool imu_quiet = acceleration_error <= 0.35 && angular_rate_norm <= 0.05;
    if (stationary_like && !stationary_detector_.exit_pending() && imu_quiet) {
      stationary_bias_window_.add(stamp, yaw_rate);
    } else if (!stationary_like) {
      stationary_bias_window_.clear();
    }
    apply_stationary_update(stamp, stationary);
    apply_visual_update();
    const double output_yaw_rate = bias_corrected_yaw_rate(yaw_rate, yaw_filter_.bias());
    publish_outputs(
      *message, gyro, accel, gyro_covariance, accel_covariance, stationary_like,
      stationary_detector_.exit_pending(), output_yaw_rate);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "RPY deg [%.3f %.3f %.3f], bias %.3f deg/s, source=%s",
      degrees(roll_), degrees(pitch_), degrees(yaw_filter_.yaw()),
      degrees(yaw_filter_.bias()), correction_source_.c_str());
  }

  void apply_stationary_update(double stamp, bool stationary)
  {
    const bool candidate = stationary_detector_.has_candidate();
    if (stationary || candidate) {
      if (!have_stationary_anchor_) {
        stationary_anchor_ = yaw_filter_.yaw();
        last_stationary_update_ = stamp;
        have_stationary_anchor_ = true;
      } else if (stamp - last_stationary_update_ >= 0.05) {
        if (stationary_detector_.exit_pending()) {
          correction_source_ = "stationary_exit_pending";
          return;
        }
        yaw_filter_.hold_yaw(stationary_anchor_);
        if (stationary_bias_window_.ready(20)) {
          yaw_filter_.update_bias(stationary_bias_window_.median(), stationary_bias_std_);
        }
        last_stationary_update_ = stamp;
        correction_source_ = stationary ? "stationary" : "stationary_candidate";
      }
    } else {
      have_stationary_anchor_ = false;
      correction_source_ = "gyro";
    }
  }

  void apply_visual_update()
  {
    if (!use_yaw_reference_ || !have_reference_ || !have_tracking_state_ ||
      reference_sequence_ == applied_reference_sequence_)
    {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double reference_age = std::chrono::duration<double>(
      now - reference_received_at_).count();
    const double tracking_age = std::chrono::duration<double>(
      now - tracking_received_at_).count();
    if (tracking_state_ != 2 || reference_age > 0.2 || tracking_age > 0.5) {
      return;
    }

    if (!have_reference_offset_) {
      reference_offset_ = wrap_angle(latest_reference_yaw_ - yaw_filter_.yaw());
      have_reference_offset_ = true;
    }
    const double relative_reference = wrap_angle(
      latest_reference_yaw_ - reference_offset_);
    if (std::abs(yaw_filter_.innovation(relative_reference)) <= visual_yaw_gate_) {
      yaw_filter_.update(relative_reference, visual_yaw_std_);
      rejected_reference_count_ = 0;
      correction_source_ = "visual";
    } else {
      ++rejected_reference_count_;
      correction_source_ = "visual_reject";
      if (rejected_reference_count_ >= 10) {
        reference_offset_ = wrap_angle(latest_reference_yaw_ - yaw_filter_.yaw());
        rejected_reference_count_ = 0;
        correction_source_ = "visual_rebase";
      }
    }
    applied_reference_sequence_ = reference_sequence_;
  }

  void publish_outputs(
    const sensor_msgs::msg::Imu & input, const Vector3 & gyro, const Vector3 & accel,
    const Matrix3 & gyro_covariance, const Matrix3 & accel_covariance, bool stationary_like,
    bool stationary_exit_pending, double corrected_yaw_rate)
  {
    std_msgs::msg::Header header = input.header;
    header.frame_id = output_frame_;

    geometry_msgs::msg::Vector3Stamped rpy;
    rpy.header = header;
    rpy.vector.x = roll_;
    rpy.vector.y = pitch_;
    rpy.vector.z = yaw_filter_.yaw();
    rpy_publisher_->publish(rpy);

    geometry_msgs::msg::Vector3Stamped rpy_degrees = rpy;
    rpy_degrees.vector.x = degrees(roll_);
    rpy_degrees.vector.y = degrees(pitch_);
    rpy_degrees.vector.z = degrees(yaw_filter_.yaw());
    rpy_degrees_publisher_->publish(rpy_degrees);

    sensor_msgs::msg::Imu filtered;
    filtered.header = header;
    filtered.orientation = quaternion_from_rpy(roll_, pitch_, yaw_filter_.yaw());
    filtered.orientation_covariance = {
      roll_pitch_variance_, 0.0, 0.0,
      0.0, roll_pitch_variance_, 0.0,
      0.0, 0.0, std::max(yaw_filter_.yaw_variance(), stationary_yaw_std_ * stationary_yaw_std_)};
    filtered.angular_velocity.x = gyro[0];
    filtered.angular_velocity.y = gyro[1];
    // Stationarity only changes uncertainty and bias learning. Publishing the
    // measured rate avoids hiding a real turn during stationary debounce.
    filtered.angular_velocity.z = corrected_yaw_rate;
    filtered.angular_velocity_covariance = gyro_covariance;
    filtered.angular_velocity_covariance[8] = yaw_rate_variance(
      stationary_like, stationary_exit_pending, stationary_yaw_rate_variance_,
      moving_yaw_rate_variance_);
    filtered.linear_acceleration.x = accel[0];
    filtered.linear_acceleration.y = accel[1];
    filtered.linear_acceleration.z = accel[2];
    filtered.linear_acceleration_covariance = accel_covariance;
    filtered_imu_publisher_->publish(filtered);
  }

  static diagnostic_msgs::msg::KeyValue diagnostic_value(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": yaw filter");
    status.hardware_id = "d455_imu";
    status.level = initialized_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = !initialized_ ? "waiting for IMU" :
      (stationary_detector_.exit_pending() ? "stationary exit debounce" :
      (stationary_detector_.active() ? "stationary bias tracking" : "integrating yaw rate"));
    status.values.push_back(diagnostic_value(
      "stationary", stationary_detector_.active() ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "stationary_candidate", stationary_detector_.has_candidate() ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "stationary_exit_pending", stationary_detector_.exit_pending() ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "external_stationary", external_stationary_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "external_moving", external_moving_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "gyro_bias_z_deg_s", std::to_string(degrees(yaw_filter_.bias()))));
    status.values.push_back(diagnostic_value(
      "gyro_bias_std_deg_s",
      std::to_string(degrees(std::sqrt(yaw_filter_.bias_variance())))));
    status.values.push_back(diagnostic_value(
      "yaw_deg", std::to_string(degrees(yaw_filter_.yaw()))));
    status.values.push_back(diagnostic_value(
      "yaw_std_deg", std::to_string(degrees(std::sqrt(std::max(
        yaw_filter_.yaw_variance(), stationary_yaw_std_ * stationary_yaw_std_))))));
    status.values.push_back(diagnostic_value(
      "rejected_stationary_transients",
      std::to_string(stationary_detector_.rejected_transient_count())));
    status.values.push_back(diagnostic_value("correction_source", correction_source_));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  VectorMedianFilter median_gyro_;
  VectorMedianFilter median_accel_;
  VectorMeanFilter mean_gyro_;
  VectorMeanFilter mean_accel_;
  LowPassFilter low_pass_gyro_;
  LowPassFilter low_pass_accel_;
  StationaryDetector stationary_detector_;
  YawBiasKalman yaw_filter_;
  TimedScalarMedian stationary_bias_window_;

  std::string imu_topic_;
  std::string rpy_topic_;
  std::string rpy_degrees_topic_;
  std::string filtered_imu_topic_;
  std::string output_frame_;
  std::string input_frame_;
  std::string yaw_reference_topic_;
  std::string tracking_topic_;
  std::string command_topic_;
  std::string wheel_topic_;
  std::string correction_source_{"gyro"};
  double correction_time_{0.5};
  double visual_yaw_std_{radians(2.0)};
  double visual_yaw_gate_{radians(15.0)};
  double stationary_yaw_std_{radians(0.5)};
  double stationary_bias_std_{radians(0.15)};
  double moving_yaw_rate_variance_{std::pow(radians(1.0), 2)};
  double stationary_yaw_rate_variance_{std::pow(radians(0.15), 2)};
  double external_motion_timeout_{0.35};
  double command_linear_stationary_threshold_{0.02};
  double command_yaw_stationary_threshold_{0.03};
  double wheel_linear_stationary_threshold_{0.02};
  double wheel_yaw_stationary_threshold_{0.05};
  double roll_pitch_variance_{std::pow(radians(1.0), 2)};
  double last_stamp_{0.0};
  double roll_{0.0};
  double pitch_{0.0};
  double stationary_anchor_{0.0};
  double last_stationary_update_{0.0};
  double latest_reference_yaw_{0.0};
  double reference_offset_{0.0};
  double latest_command_linear_{0.0};
  double latest_command_yaw_rate_{0.0};
  double latest_wheel_linear_{0.0};
  double latest_wheel_yaw_rate_{0.0};
  bool transform_optical_frame_{true};
  bool use_yaw_reference_{false};
  bool initialized_{false};
  bool have_stationary_anchor_{false};
  bool have_reference_{false};
  bool have_reference_offset_{false};
  bool have_tracking_state_{false};
  bool have_command_{false};
  bool have_wheel_{false};
  bool external_stationary_{false};
  bool external_moving_{false};
  int tracking_state_{0};
  int rejected_reference_count_{0};
  std::size_t reference_sequence_{0};
  std::size_t applied_reference_sequence_{0};
  std::chrono::steady_clock::time_point reference_received_at_{};
  std::chrono::steady_clock::time_point tracking_received_at_{};
  std::chrono::steady_clock::time_point command_received_at_{};
  std::chrono::steady_clock::time_point wheel_received_at_{};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracking_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    rpy_degrees_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace imu_rpy_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_rpy_filter::ImuRpyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
