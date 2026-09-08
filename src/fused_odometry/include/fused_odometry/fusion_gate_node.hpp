#ifndef FUSED_ODOMETRY__FUSIONGATENODE_HPP_
#define FUSED_ODOMETRY__FUSIONGATENODE_HPP_

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

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "fused_odometry/fusion_health.hpp"
#include "fused_odometry/fusion_logic.hpp"
#include "fused_odometry/sensor_input_validation.hpp"

namespace fused_odometry {
namespace detail {

using SteadyTime = std::chrono::steady_clock::time_point;

inline double age_seconds(const SteadyTime &stamp) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp)
      .count();
}

inline double steady_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline bool finite(double value) { return std::isfinite(value); }

inline double quaternion_yaw(const geometry_msgs::msg::Quaternion &quaternion) {
  const double inverse_norm =
      1.0 /
      std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x * inverse_norm;
  const double y = quaternion.y * inverse_norm;
  const double z = quaternion.z * inverse_norm;
  const double w = quaternion.w * inverse_norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline geometry_msgs::msg::Quaternion yaw_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

inline diagnostic_msgs::msg::KeyValue value(const std::string &key,
                                            const std::string &data) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = data;
  return result;
}

} // namespace detail

using detail::SteadyTime;
using detail::age_seconds;
using detail::finite;
using detail::quaternion_yaw;
using detail::steady_seconds;
using detail::value;
using detail::yaw_quaternion;

class FusionGateNode : public rclcpp::Node {
public:
  FusionGateNode();

private:
  struct InputState {
    bool received{false};
    SteadyTime received_at{};

    void update() {
      received = true;
      received_at = std::chrono::steady_clock::now();
    }

    bool fresh(double timeout) const {
      return received && age_seconds(received_at) <= timeout;
    }
  };

  struct StampState {
    rclcpp::Time last{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct TrackingState {
    InputState input;
    bool good{false};
    int code{-1};
  };

  struct VisualState {
    InputState input;
    bool accepted{false};
    bool ever_accepted{false};
    bool interrupted{true};
    bool velocity_valid{false};
    bool ramp_active{false};
    std::size_t recovery_samples{0};
    std::size_t realign_count{0};
    std::size_t rejected_recoveries{0};
    double forward_velocity{0.0};
    double yaw_rate{0.0};
    double yaw_disagreement_scale{1.0};
    double ramp_started_at{0.0};
    double realigned_until{0.0};
  };

  struct RawVisualState {
    InputState input;
    StampState stamp;
    bool velocity_valid{false};
    bool increment_pose_valid{false};
    std::size_t invalid_samples{0};
    double forward_velocity{0.0};
    double yaw_rate{0.0};
    Pose2d last_increment_pose{};
    rclcpp::Time last_increment_stamp;
  };

  struct WheelYawState {
    bool rate_finite{false};
    bool validation_active{false};
    bool validated{false};
    bool excited{false};
    bool instantly_consistent{false};
    double rate{0.0};
    double validation_started_at{0.0};
    double last_validated_at{0.0};
  };

  struct WheelState {
    InputState input;
    StampState stamp;
    WheelYawState yaw;
    bool vx_zeroed{false};
    std::size_t invalid_samples{0};
    double velocity{0.0};
    double residual{0.0};
    double vx_turn_scale{1.0};
  };

  struct ImuState {
    InputState input;
    StampState stamp;
    std::size_t invalid_samples{0};
    double yaw_rate{0.0};
    double visual_residual{0.0};
    double robust_visual_residual{0.0};
    YawBiasEstimator bias_estimator{};
  };

  struct CommandState {
    InputState input;
    double velocity{0.0};
    double yaw_rate{0.0};
  };

  struct MapChangeState {
    bool pending{false};
    double grace_until{0.0};
    uint64_t sequence{0};
  };

  double positive(const std::string &name, double default_value);

  double nonnegative(const std::string &name, double default_value);

  bool
  accept_measurement_stamp(const builtin_interfaces::msg::Time &stamp_message,
                           StampState &stamp_state, double max_age,
                           const char *source);

  std::size_t positive_count(const std::string &name,
                             std::int64_t default_value);

  bool tracking_fresh_and_good() const;

  bool vision_healthy() const;

  bool wheel_healthy() const;

  bool imu_healthy() const;

  bool raw_visual_increment_healthy() const;

  void reject_raw_visual_sample();

  void tracking_callback(const std_msgs::msg::Int32::SharedPtr message);

  void map_change_callback(const std_msgs::msg::UInt64::SharedPtr message);

  void mark_visual_interrupted();

  void raw_visual_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);

  void publish_raw_visual(const nav_msgs::msg::Odometry &message,
                          const Pose2d &raw_pose);

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message);

  void publish_status();

  ResidualGate wheel_gate_;
  RobustWindow visual_vx_window_;
  RobustWindow visual_wz_window_;
  RobustWindow raw_visual_vx_window_;
  RobustWindow raw_visual_wz_window_;
  RobustWindow wheel_vx_window_;
  RobustWindow wheel_visual_residual_window_;
  RobustWindow wheel_imu_yaw_residual_window_;
  RobustWindow imu_visual_residual_window_;
  FusionHealthMonitor health_monitor_;

  std::string raw_visual_topic_;
  std::string tracking_topic_;
  std::string map_change_topic_;
  std::string wheel_topic_;
  std::string imu_topic_;
  std::string command_topic_;
  std::string visual_output_topic_;
  std::string wheel_output_topic_;
  std::string imu_output_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string world_frame_;
  std::string wheel_frame_;
  std::string base_frame_;
  std::string visual_expected_child_frame_;
  std::string raw_visual_expected_frame_;
  std::string wheel_expected_child_frame_;
  std::string wheel_expected_frame_;
  std::string imu_expected_frame_;

  double visual_timeout_{0.4};
  double tracking_timeout_{0.6};
  double wheel_timeout_{0.35};
  double imu_timeout_{0.15};
  double command_timeout_{0.4};
  double raw_visual_timeout_{0.3};
  double raw_visual_max_tilt_{0.5};
  double map_change_grace_{1.0};
  double stamp_future_tolerance_{0.20};
  double initialization_timeout_{10.0};
  double initialization_started_at_{0.0};
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

  bool fuse_wheel_yaw_{false};
  TrackingState tracking_;
  VisualState visual_;
  RawVisualState raw_visual_;
  WheelState wheel_;
  ImuState imu_;
  CommandState command_;
  MapChangeState map_change_;
  FusionMode last_mode_{FusionMode::kFull};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr visual_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      raw_visual_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tracking_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      map_change_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      command_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace fused_odometry

#endif // FUSED_ODOMETRY__FUSIONGATENODE_HPP_
