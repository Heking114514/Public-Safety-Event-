#ifndef VISUAL_NAVIGATION__RECOVERY_STOP_HPP_
#define VISUAL_NAVIGATION__RECOVERY_STOP_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace visual_navigation {
// 停止证据只接受开始之后的新遥测，重复缓存不能增加连续停止样本。
class RecoveryStopGuard {
public:
  enum class Result { WAITING, STOPPED, TIMED_OUT };
  void configure(double speed, double yaw_rate, double dwell, double timeout,
                 unsigned samples) {
    speed_ = speed; yaw_rate_ = yaw_rate; dwell_ = dwell;
    timeout_ = timeout; required_ = samples;
  }
  void begin(double now, std::uint64_t sample) {
    started_ = now; stable_since_ = now; last_sample_ = sample;
    consecutive_ = 0;
  }
  Result update(double now, std::uint64_t sample, bool fresh,
                double left, double right, double target_left,
                double target_right, double yaw_rate) {
    const bool stationary = fresh && std::isfinite(left) && std::isfinite(right) &&
      std::isfinite(target_left) && std::isfinite(target_right) && std::isfinite(yaw_rate) &&
      std::abs(yaw_rate) <= yaw_rate_ &&
      std::max({std::abs(left), std::abs(right), std::abs(target_left),
                std::abs(target_right)}) <= speed_;
    if (!stationary) { consecutive_ = 0; stable_since_ = now; }
    else if (sample > last_sample_) {
      if (consecutive_ == 0) stable_since_ = now;
      ++consecutive_;
    }
    last_sample_ = sample;
    if (stationary && consecutive_ >= required_ && now - stable_since_ >= dwell_)
      return Result::STOPPED;
    return now - started_ >= timeout_ ? Result::TIMED_OUT : Result::WAITING;
  }
private:
  double speed_{0.0}, yaw_rate_{0.0}, dwell_{0.0}, timeout_{0.0};
  double started_{0.0}, stable_since_{0.0};
  std::uint64_t last_sample_{0};
  unsigned required_{0}, consecutive_{0};
};
} // namespace visual_navigation
#endif
