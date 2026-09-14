#ifndef VISUAL_NAVIGATION__WAYPOINT_BRAKE_CONTROLLER_HPP_
#define VISUAL_NAVIGATION__WAYPOINT_BRAKE_CONTROLLER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

#include "visual_navigation/path_control.hpp"

namespace visual_navigation {

inline bool ControlTelemetrySampleIsFresh(
    std::uint32_t previous_sequence, std::uint32_t previous_mcu_time,
    std::uint32_t sequence, std::uint32_t mcu_time) {
  const std::uint32_t advance = sequence - previous_sequence;
  if (advance == 0U)
    return false;
  if (advance < (std::uint32_t{1} << 31U))
    return true;
  return mcu_time < previous_mcu_time;
}

struct BrakeSpeedObservation {
  bool valid{false};
  double absolute_speed{std::numeric_limits<double>::quiet_NaN()};
  bool from_wheel_telemetry{false};
};

inline BrakeSpeedObservation SelectWaypointBrakeSpeed(
    bool wheel_sample_after_brake, double wheel_sample_age,
    double wheel_sample_timeout, double measured_left_speed,
    double measured_right_speed, bool fallback_valid, double fallback_speed) {
  BrakeSpeedObservation observation;
  if (wheel_sample_after_brake && std::isfinite(wheel_sample_age) &&
      wheel_sample_age >= 0.0 &&
      wheel_sample_age <= std::max(0.0, wheel_sample_timeout) &&
      std::isfinite(measured_left_speed) &&
      std::isfinite(measured_right_speed)) {
    observation.valid = true;
    observation.absolute_speed =
        std::max(std::abs(measured_left_speed),
                 std::abs(measured_right_speed));
    observation.from_wheel_telemetry = true;
    return observation;
  }

  if (fallback_valid && std::isfinite(fallback_speed)) {
    observation.valid = true;
    observation.absolute_speed = std::abs(fallback_speed);
  }
  return observation;
}

inline double IndependentWheelStopEvidence(
    double measured_left_speed, double measured_right_speed,
    double target_left_speed, double target_right_speed,
    double stop_speed, std::size_t consecutive_stop_samples,
    std::size_t required_stop_samples) {
  if (!std::isfinite(measured_left_speed) ||
      !std::isfinite(measured_right_speed) ||
      !std::isfinite(target_left_speed) ||
      !std::isfinite(target_right_speed)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double reported_speed = std::max(
      std::max(std::abs(measured_left_speed), std::abs(measured_right_speed)),
      std::max(std::abs(target_left_speed), std::abs(target_right_speed)));
  const double threshold = std::max(0.0, stop_speed);
  if (reported_speed <= threshold &&
      consecutive_stop_samples < std::max<std::size_t>(1, required_stop_samples)) {
    return std::nextafter(threshold, std::numeric_limits<double>::infinity());
  }
  return reported_speed;
}

inline bool BrakeFallbackMayConfirmStop(
    std::size_t timeout_count, std::size_t required_timeouts,
    bool pose_speed_valid, double pose_speed, double maximum_pose_speed,
    bool yaw_rate_valid, double yaw_rate, double maximum_yaw_rate) {
  return timeout_count >= std::max<std::size_t>(1, required_timeouts) &&
         pose_speed_valid && std::isfinite(pose_speed) &&
         std::abs(pose_speed) <= std::max(0.0, maximum_pose_speed) &&
         yaw_rate_valid && std::isfinite(yaw_rate) &&
         std::abs(yaw_rate) <= std::max(0.0, maximum_yaw_rate);
}

class WaypointBrakeController {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void configure(double stop_speed, double stop_dwell, double minimum_stop_time,
                 double timeout) {
    stop_speed_ = std::max(0.0, stop_speed);
    stop_dwell_ = std::max(0.0, stop_dwell);
    minimum_stop_time_ = std::max(0.0, minimum_stop_time);
    timeout_ = std::max(minimum_stop_time_, timeout);
  }

  void reset() {
    timed_out_ = false;
    stop_timer_initialized_ = false;
    speed_settle_timer_initialized_ = false;
  }

  void begin(TimePoint now) {
    timed_out_ = false;
    stop_started_ = now;
    stop_timer_initialized_ = true;
    speed_settle_timer_initialized_ = false;
  }

  bool check(TimePoint now, bool speed_valid, double speed) {
    if (!stop_timer_initialized_) {
      stop_started_ = now;
      stop_timer_initialized_ = true;
    }
    const double total_stop_seconds =
        std::chrono::duration<double>(now - stop_started_).count();
    const double absolute_speed =
        speed_valid ? speed : std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(absolute_speed) && absolute_speed <= stop_speed_) {
      if (!speed_settle_timer_initialized_) {
        speed_settle_started_ = now;
        speed_settle_timer_initialized_ = true;
      }
    } else {
      speed_settle_timer_initialized_ = false;
    }
    const double settled_seconds =
        speed_settle_timer_initialized_
            ? std::chrono::duration<double>(now - speed_settle_started_).count()
            : 0.0;
    const bool complete = WaypointStopSatisfied(
        absolute_speed, settled_seconds, total_stop_seconds, stop_speed_,
        stop_dwell_, minimum_stop_time_, timeout_);
    if (!complete && total_stop_seconds >= timeout_)
      timed_out_ = true;
    return complete;
  }

  bool timed_out() const { return timed_out_; }

private:
  double stop_speed_{0.03};
  double stop_dwell_{0.10};
  double minimum_stop_time_{0.20};
  double timeout_{0.60};
  bool timed_out_{false};
  bool stop_timer_initialized_{false};
  bool speed_settle_timer_initialized_{false};
  TimePoint stop_started_{};
  TimePoint speed_settle_started_{};
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__WAYPOINT_BRAKE_CONTROLLER_HPP_
