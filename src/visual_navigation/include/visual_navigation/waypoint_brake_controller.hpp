#ifndef VISUAL_NAVIGATION__WAYPOINT_BRAKE_CONTROLLER_HPP_
#define VISUAL_NAVIGATION__WAYPOINT_BRAKE_CONTROLLER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "visual_navigation/path_control.hpp"

namespace visual_navigation {

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
