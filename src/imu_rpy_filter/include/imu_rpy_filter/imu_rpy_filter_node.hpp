#ifndef IMU_RPY_FILTER__IMURPYFILTERNODE_HPP_
#define IMU_RPY_FILTER__IMURPYFILTERNODE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

#include "imu_rpy_filter/imu_time.hpp"
#include "imu_rpy_filter/yaw_filter.hpp"

namespace imu_rpy_filter {

constexpr double kGravity = 9.80665;

inline double radians(double degrees) { return degrees * kPi / 180.0; }

inline double degrees(double angle) { return angle * 180.0 / kPi; }

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;

inline Vector3 optical_to_body(const Vector3 &value) {
  return {value[2], -value[0], -value[1]};
}

inline Matrix3 optical_covariance_to_body(const Matrix3 &covariance) {
  constexpr Matrix3 rotation = {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
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

inline double vector_norm(const Vector3 &value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

inline bool finite_vector(const Vector3 &value) {
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

inline bool finite_matrix(const Matrix3 &value) {
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

inline bool accepted_imu_frame(const std::string &frame,
                               const std::string &expected_frame) {
  if (frame.empty()) {
    return false;
  }
  if (frame == expected_frame) {
    return true;
  }
  constexpr char optical_suffix[] = "_optical_frame";
  const std::size_t suffix_length = sizeof(optical_suffix) - 1;
  return frame.size() >= suffix_length &&
         frame.compare(frame.size() - suffix_length, suffix_length,
                       optical_suffix) == 0;
}

class VectorMedianFilter {
public:
  explicit VectorMedianFilter(std::size_t window_size)
      : window_size_(std::max<std::size_t>(1, window_size)) {}

  Vector3 update(const Vector3 &sample) {
    samples_.push_back(sample);
    if (samples_.size() > window_size_) {
      samples_.pop_front();
    }

    Vector3 result{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      std::vector<double> values;
      values.reserve(samples_.size());
      for (const auto &item : samples_) {
        values.push_back(item[axis]);
      }
      const auto middle =
          values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
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

class VectorMeanFilter {
public:
  explicit VectorMeanFilter(std::size_t window_size)
      : window_size_(std::max<std::size_t>(1, window_size)) {}

  Vector3 update(const Vector3 &sample) {
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

class LowPassFilter {
public:
  explicit LowPassFilter(double cutoff_hz)
      : time_constant_(cutoff_hz > 0.0 ? 1.0 / (2.0 * kPi * cutoff_hz) : 0.0) {}

  Vector3 update(const Vector3 &sample, double dt) {
    if (!initialized_) {
      value_ = sample;
      initialized_ = true;
      return value_;
    }
    const double weight =
        time_constant_ > 0.0 ? dt / (time_constant_ + dt) : 1.0;
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

class TimedScalarMedian {
public:
  explicit TimedScalarMedian(double duration_s) : duration_s_(duration_s) {}

  void add(double stamp, double value) {
    samples_.emplace_back(stamp, value);
    while (!samples_.empty() && stamp - samples_.front().first > duration_s_) {
      samples_.pop_front();
    }
  }

  void clear() { samples_.clear(); }

  bool ready(std::size_t minimum_samples) const {
    return samples_.size() >= minimum_samples &&
           samples_.back().first - samples_.front().first >= 0.5 * duration_s_;
  }

  double median() const {
    std::vector<double> values;
    values.reserve(samples_.size());
    for (const auto &sample : samples_) {
      values.push_back(sample.second);
    }
    const auto middle =
        values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

private:
  double duration_s_;
  std::deque<std::pair<double, double>> samples_;
};

inline geometry_msgs::msg::Quaternion
quaternion_from_rpy(double roll, double pitch, double yaw) {
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

inline double quaternion_yaw(const geometry_msgs::msg::Quaternion &quaternion) {
  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

inline double euler_yaw_rate(const Vector3 &gyro, double roll, double pitch) {
  double cos_pitch = std::cos(pitch);
  if (std::abs(cos_pitch) < 1e-3) {
    cos_pitch = std::copysign(1e-3, cos_pitch);
  }
  return std::sin(roll) / cos_pitch * gyro[1] +
         std::cos(roll) / cos_pitch * gyro[2];
}

class ImuRpyFilterNode : public rclcpp::Node {
public:
  ImuRpyFilterNode();

private:
  std::size_t declare_positive_integer(const std::string &name,
                                       std::int64_t default_value);

  double declare_positive(const std::string &name, double default_value);

  double declare_nonnegative(const std::string &name, double default_value);

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  void tracking_callback(const std_msgs::msg::Int32::SharedPtr message);

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  void update_external_motion_diagnostics();

  bool
  command_fresh(const std::chrono::steady_clock::time_point &current) const;

  bool wheel_fresh(const std::chrono::steady_clock::time_point &current) const;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message);

  void apply_stationary_update(double stamp, bool stationary);

  void apply_visual_update();

  void publish_outputs(const sensor_msgs::msg::Imu &input, const Vector3 &gyro,
                       const Vector3 &accel, const Matrix3 &gyro_covariance,
                       const Matrix3 &accel_covariance, bool stationary_like,
                       bool stationary_exit_pending, double corrected_yaw_rate);

  static diagnostic_msgs::msg::KeyValue
  diagnostic_value(const std::string &key, const std::string &value);

  void publish_diagnostics();

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
  double correction_time_;
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
  double wheel_yaw_stationary_threshold_;
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
  std::chrono::steady_clock::time_point wheel_received_at_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracking_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      rpy_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      rpy_degrees_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

} // namespace imu_rpy_filter

#endif // IMU_RPY_FILTER__IMURPYFILTERNODE_HPP_
