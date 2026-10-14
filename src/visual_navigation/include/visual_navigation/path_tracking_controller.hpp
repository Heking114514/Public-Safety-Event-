#ifndef VISUAL_NAVIGATION__PATH_TRACKING_CONTROLLER_HPP_
#define VISUAL_NAVIGATION__PATH_TRACKING_CONTROLLER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>

namespace visual_navigation
{

class PathTrackingController
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void configure(
    double kp, double ki, double kd, double yaw_rate_damping,
    double integral_limit, double maximum_angular_speed)
  {
    kp_ = std::max(0.0, kp);
    ki_ = std::max(0.0, ki);
    kd_ = std::max(0.0, kd);
    yaw_rate_damping_ = std::max(0.0, yaw_rate_damping);
    integral_limit_ = std::max(0.0, integral_limit);
    maximum_angular_speed_ = std::max(0.0, maximum_angular_speed);
  }

  void reset_pid()
  {
    integral_ = 0.0;
    previous_error_ = 0.0;
    pid_initialized_ = false;
  }

  double update_pid(
    double error, double cross_track_error, bool allow_integral,
    double yaw_rate, TimePoint current_time)
  {
    if (!std::isfinite(error) || !std::isfinite(cross_track_error) ||
      !std::isfinite(yaw_rate))
    {
      reset_pid();
      return 0.0;
    }
    double derivative = 0.0;
    if (pid_initialized_) {
      const double dt = std::chrono::duration<double>(current_time - last_pid_time_).count();
      if (dt > 0.0 && dt <= 0.2) {
        if (allow_integral) {
          integral_ = std::max(
            -integral_limit_, std::min(integral_limit_, integral_ - cross_track_error * dt));
        } else {
          integral_ *= std::max(0.0, 1.0 - 4.0 * dt);
        }
        derivative = (error - previous_error_) / dt;
      }
    }
    previous_error_ = error;
    last_pid_time_ = current_time;
    pid_initialized_ = true;
    const double command = kp_ * error + ki_ * integral_ + kd_ * derivative -
      yaw_rate_damping_ * yaw_rate;
    return std::max(-maximum_angular_speed_, std::min(maximum_angular_speed_, command));
  }

  void update_observed_speed(
    double x, double y, TimePoint sample_time, double window_seconds)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(window_seconds) ||
      window_seconds <= 0.0)
    {
      observed_speed_valid_ = false;
      return;
    }
    if (!position_samples_.empty()) {
      const double gap = std::chrono::duration<double>(
        sample_time - position_samples_.back().time).count();
      if (gap > std::max(0.5, 2.0 * window_seconds))
        position_samples_.clear();
    }
    position_samples_.push_back({sample_time, x, y});
    while (position_samples_.size() >= 2) {
      const double second_age = std::chrono::duration<double>(
        sample_time - position_samples_[1].time).count();
      if (second_age < window_seconds)
        break;
      position_samples_.pop_front();
    }
    const double span = std::chrono::duration<double>(
      sample_time - position_samples_.front().time).count();
    observed_speed_valid_ = span >= 0.5 * window_seconds;
    if (observed_speed_valid_) {
      observed_speed_ = std::hypot(
        x - position_samples_.front().x, y - position_samples_.front().y) / span;
      observed_speed_valid_ = std::isfinite(observed_speed_);
    }
  }

  bool observed_speed_valid() const {return observed_speed_valid_;}
  double observed_speed() const {return observed_speed_;}

private:
  struct PositionSample
  {
    TimePoint time;
    double x{0.0};
    double y{0.0};
  };

  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
  double yaw_rate_damping_{0.0};
  double integral_limit_{0.0};
  double maximum_angular_speed_{0.0};
  double integral_{0.0};
  double previous_error_{0.0};
  bool pid_initialized_{false};
  TimePoint last_pid_time_{};
  std::deque<PositionSample> position_samples_;
  double observed_speed_{0.0};
  bool observed_speed_valid_{false};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__PATH_TRACKING_CONTROLLER_HPP_
