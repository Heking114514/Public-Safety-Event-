#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"

namespace imu_rpy_filter
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double degrees(double angle)
{
  return angle * 180.0 / kPi;
}

double wrap_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
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

class StationaryDetector
{
public:
  StationaryDetector(
    double dwell_time, double acceleration_enter, double acceleration_exit,
    double gyro_enter, double gyro_exit)
  : dwell_time_(dwell_time),
    acceleration_enter_(acceleration_enter),
    acceleration_exit_(acceleration_exit),
    gyro_enter_(gyro_enter),
    gyro_exit_(gyro_exit)
  {
  }

  bool update(double stamp, double acceleration_error, double gyro_norm)
  {
    if (active_) {
      if (acceleration_error > acceleration_exit_ || gyro_norm > gyro_exit_) {
        active_ = false;
        candidate_since_ = -1.0;
      }
      return active_;
    }

    if (acceleration_error <= acceleration_enter_ && gyro_norm <= gyro_enter_) {
      if (candidate_since_ < 0.0) {
        candidate_since_ = stamp;
      } else if (stamp - candidate_since_ >= dwell_time_) {
        active_ = true;
      }
    } else {
      candidate_since_ = -1.0;
    }
    return active_;
  }

  bool has_candidate() const
  {
    return candidate_since_ >= 0.0;
  }

private:
  double dwell_time_;
  double acceleration_enter_;
  double acceleration_exit_;
  double gyro_enter_;
  double gyro_exit_;
  double candidate_since_{-1.0};
  bool active_{false};
};

class YawBiasKalman
{
public:
  YawBiasKalman(double gyro_noise, double bias_walk)
  : gyro_noise_(gyro_noise), bias_walk_(bias_walk)
  {
  }

  void predict(double measured_yaw_rate, double dt)
  {
    yaw_ = wrap_angle(yaw_ + (measured_yaw_rate - bias_) * dt);

    const double old_p00 = p00_;
    const double old_p01 = p01_;
    const double old_p10 = p10_;
    const double old_p11 = p11_;
    p00_ = old_p00 - dt * (old_p01 + old_p10) + dt * dt * old_p11 +
      std::pow(gyro_noise_ * dt, 2);
    p01_ = old_p01 - dt * old_p11;
    p10_ = old_p10 - dt * old_p11;
    p11_ = old_p11 + bias_walk_ * bias_walk_ * dt;
  }

  double innovation(double measured_yaw) const
  {
    return wrap_angle(measured_yaw - yaw_);
  }

  void update(double measured_yaw, double measurement_std)
  {
    const double residual = innovation(measured_yaw);
    const double residual_covariance = p00_ + measurement_std * measurement_std;
    const double gain_yaw = p00_ / residual_covariance;
    const double gain_bias = p10_ / residual_covariance;

    yaw_ = wrap_angle(yaw_ + gain_yaw * residual);
    bias_ = std::clamp(bias_ + gain_bias * residual, radians(-5.0), radians(5.0));

    const double old_p00 = p00_;
    const double old_p01 = p01_;
    const double old_p10 = p10_;
    const double old_p11 = p11_;
    p00_ = std::max((1.0 - gain_yaw) * old_p00, 1e-12);
    p01_ = (1.0 - gain_yaw) * old_p01;
    p10_ = old_p10 - gain_bias * old_p00;
    p11_ = std::max(old_p11 - gain_bias * old_p01, 1e-12);
    const double off_diagonal = 0.5 * (p01_ + p10_);
    p01_ = off_diagonal;
    p10_ = off_diagonal;
  }

  double yaw() const {return yaw_;}
  double bias() const {return bias_;}
  double yaw_variance() const {return p00_;}

private:
  double yaw_{0.0};
  double bias_{0.0};
  double p00_{std::pow(radians(5.0), 2)};
  double p01_{0.0};
  double p10_{0.0};
  double p11_{std::pow(radians(1.0), 2)};
  double gyro_noise_;
  double bias_walk_;
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
      declare_nonnegative("stationary_accel_enter", 0.20),
      declare_nonnegative("stationary_accel_exit", 0.35),
      declare_nonnegative("stationary_gyro_enter", 0.02),
      declare_nonnegative("stationary_gyro_exit", 0.05)),
    yaw_filter_(
      radians(declare_nonnegative("kalman_gyro_noise_deg_s", 0.6)),
      radians(declare_nonnegative("kalman_bias_walk_deg_s", 0.03)))
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
    rpy_topic_ = declare_parameter<std::string>("rpy_topic", "/imu/rpy");
    rpy_degrees_topic_ =
      declare_parameter<std::string>("rpy_degrees_topic", "/imu/rpy_degrees");
    filtered_imu_topic_ =
      declare_parameter<std::string>("filtered_imu_topic", "/imu/filtered");
    output_frame_ = declare_parameter<std::string>("output_frame", "camera_link");
    correction_time_ = declare_positive("tilt_correction_time", 0.5);
    transform_optical_frame_ = declare_parameter<bool>("transform_optical_frame", true);
    use_yaw_reference_ = declare_parameter<bool>("use_yaw_reference", false);
    yaw_reference_topic_ = declare_parameter<std::string>("yaw_reference_topic", "/odom");
    tracking_topic_ = declare_parameter<std::string>("tracking_topic", "/tracking_state");
    visual_yaw_std_ = radians(declare_positive("visual_yaw_std_degrees", 2.0));
    visual_yaw_gate_ = radians(declare_positive("visual_yaw_gate_degrees", 15.0));
    stationary_yaw_std_ = radians(declare_positive("stationary_yaw_std_degrees", 0.5));
    roll_pitch_variance_ = std::pow(
      radians(declare_positive("roll_pitch_std_degrees", 1.0)), 2);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    rpy_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rpy_topic_, sensor_qos);
    rpy_degrees_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      rpy_degrees_topic_, sensor_qos);
    filtered_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      filtered_imu_topic_, sensor_qos);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&ImuRpyFilterNode::imu_callback, this, std::placeholders::_1));

    if (use_yaw_reference_) {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        yaw_reference_topic_, sensor_qos,
        std::bind(&ImuRpyFilterNode::odometry_callback, this, std::placeholders::_1));
      tracking_subscription_ = create_subscription<std_msgs::msg::Int32>(
        tracking_topic_, sensor_qos,
        std::bind(&ImuRpyFilterNode::tracking_callback, this, std::placeholders::_1));
    }

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

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
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

    const bool input_is_optical = transform_optical_frame_ &&
      message->header.frame_id.size() >= 14 &&
      message->header.frame_id.compare(
      message->header.frame_id.size() - 14, 14, "_optical_frame") == 0;
    if (input_is_optical) {
      gyro = optical_to_body(gyro);
      accel = optical_to_body(accel);
      gyro_covariance = optical_covariance_to_body(gyro_covariance);
      accel_covariance = optical_covariance_to_body(accel_covariance);
    }

    gyro = mean_gyro_.update(median_gyro_.update(gyro));
    accel = mean_accel_.update(median_accel_.update(accel));

    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (!initialized_) {
      gyro = low_pass_gyro_.update(gyro, 0.005);
      accel = low_pass_accel_.update(accel, 0.005);
      roll_ = std::atan2(accel[1], accel[2]);
      pitch_ = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));
      last_stamp_ = stamp;
      initialized_ = true;
      publish_outputs(*message, gyro, accel, gyro_covariance, accel_covariance, false);
      return;
    }

    const double dt = stamp - last_stamp_;
    last_stamp_ = stamp;
    if (dt <= 0.0 || dt > 0.1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected IMU dt %.6f seconds", dt);
      return;
    }

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
    const double yaw_rate = std::sin(roll_) / cos_pitch * gyro[1] +
      std::cos(roll_) / cos_pitch * gyro[2];

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

    const double corrected_yaw_rate = yaw_rate - yaw_filter_.bias();
    const double angular_rate_norm = std::sqrt(
      roll_rate * roll_rate + pitch_rate * pitch_rate +
      corrected_yaw_rate * corrected_yaw_rate);
    const bool stationary = stationary_detector_.update(
      stamp, acceleration_error, angular_rate_norm);
    apply_stationary_update(stamp, stationary);
    apply_visual_update();
    publish_outputs(*message, gyro, accel, gyro_covariance, accel_covariance, stationary);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "RPY deg [%.3f %.3f %.3f], bias %.3f deg/s, source=%s",
      degrees(roll_), degrees(pitch_), degrees(yaw_filter_.yaw()),
      degrees(yaw_filter_.bias()), correction_source_.c_str());
  }

  void apply_stationary_update(double stamp, bool stationary)
  {
    if (stationary) {
      if (!have_stationary_anchor_) {
        stationary_anchor_ = yaw_filter_.yaw();
        last_stationary_update_ = stamp;
        have_stationary_anchor_ = true;
      } else if (stamp - last_stationary_update_ >= 0.05) {
        yaw_filter_.update(stationary_anchor_, stationary_yaw_std_);
        last_stationary_update_ = stamp;
        correction_source_ = "stationary";
      }
    } else if (stationary_detector_.has_candidate()) {
      if (!have_stationary_anchor_) {
        stationary_anchor_ = yaw_filter_.yaw();
        last_stationary_update_ = stamp;
        have_stationary_anchor_ = true;
      }
      correction_source_ = "gyro";
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
    const Matrix3 & gyro_covariance, const Matrix3 & accel_covariance, bool stationary)
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
      0.0, 0.0, yaw_filter_.yaw_variance()};
    filtered.angular_velocity.x = gyro[0];
    filtered.angular_velocity.y = gyro[1];
    filtered.angular_velocity.z = stationary ? 0.0 : gyro[2] - yaw_filter_.bias();
    filtered.angular_velocity_covariance = gyro_covariance;
    filtered.linear_acceleration.x = accel[0];
    filtered.linear_acceleration.y = accel[1];
    filtered.linear_acceleration.z = accel[2];
    filtered.linear_acceleration_covariance = accel_covariance;
    filtered_imu_publisher_->publish(filtered);
  }

  VectorMedianFilter median_gyro_;
  VectorMedianFilter median_accel_;
  VectorMeanFilter mean_gyro_;
  VectorMeanFilter mean_accel_;
  LowPassFilter low_pass_gyro_;
  LowPassFilter low_pass_accel_;
  StationaryDetector stationary_detector_;
  YawBiasKalman yaw_filter_;

  std::string imu_topic_;
  std::string rpy_topic_;
  std::string rpy_degrees_topic_;
  std::string filtered_imu_topic_;
  std::string output_frame_;
  std::string yaw_reference_topic_;
  std::string tracking_topic_;
  std::string correction_source_{"gyro"};
  double correction_time_{0.5};
  double visual_yaw_std_{radians(2.0)};
  double visual_yaw_gate_{radians(15.0)};
  double stationary_yaw_std_{radians(0.5)};
  double roll_pitch_variance_{std::pow(radians(1.0), 2)};
  double last_stamp_{0.0};
  double roll_{0.0};
  double pitch_{0.0};
  double stationary_anchor_{0.0};
  double last_stationary_update_{0.0};
  double latest_reference_yaw_{0.0};
  double reference_offset_{0.0};
  bool transform_optical_frame_{true};
  bool use_yaw_reference_{false};
  bool initialized_{false};
  bool have_stationary_anchor_{false};
  bool have_reference_{false};
  bool have_reference_offset_{false};
  bool have_tracking_state_{false};
  int tracking_state_{0};
  int rejected_reference_count_{0};
  std::size_t reference_sequence_{0};
  std::size_t applied_reference_sequence_{0};
  std::chrono::steady_clock::time_point reference_received_at_{};
  std::chrono::steady_clock::time_point tracking_received_at_{};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracking_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    rpy_degrees_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_publisher_;
};

}  // namespace imu_rpy_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_rpy_filter::ImuRpyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
