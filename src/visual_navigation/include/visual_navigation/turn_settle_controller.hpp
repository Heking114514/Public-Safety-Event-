#ifndef VISUAL_NAVIGATION__TURN_SETTLE_CONTROLLER_HPP_
#define VISUAL_NAVIGATION__TURN_SETTLE_CONTROLLER_HPP_

#include <chrono>

namespace visual_navigation
{

class TurnSettleController
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void reset()
  {
    settling_ = false;
    timer_initialized_ = false;
  }

  void begin_settling() {settling_ = true; timer_initialized_ = false;}
  bool settling() const {return settling_;}

  void reset_timer() {timer_initialized_ = false;}

  bool update(TimePoint now, double activity, double threshold, double dwell)
  {
    if (activity > threshold) {
      timer_initialized_ = false;
      return false;
    }
    if (!timer_initialized_) {
      started_at_ = now;
      timer_initialized_ = true;
    }
    return std::chrono::duration<double>(now - started_at_).count() >= dwell;
  }

private:
  bool settling_{false};
  bool timer_initialized_{false};
  TimePoint started_at_{};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__TURN_SETTLE_CONTROLLER_HPP_
