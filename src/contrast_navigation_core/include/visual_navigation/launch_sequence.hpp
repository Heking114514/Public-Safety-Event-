#ifndef VISUAL_NAVIGATION__LAUNCH_SEQUENCE_HPP_
#define VISUAL_NAVIGATION__LAUNCH_SEQUENCE_HPP_

#include <cmath>
#include "visual_navigation/turn_maneuver.hpp"

namespace visual_navigation {
// 只管理起步往返和入位；路口中心与规划路线保持不变。
class LaunchSequence {
public:
  enum class Phase { IDLE, INITIAL_STOP, FORWARD, FORWARD_STOP,
                     RETURN, ORIGIN_STOP, ENTRY, ENTRY_STOP, COMPLETE, FAULT };
  void begin(CornerShuntPose origin, double probe, double entry,
             double tolerance, double timeout) {
    origin_ = origin; probe_ = probe; entry_ = entry;
    tolerance_ = tolerance; timeout_ = timeout;
    phase_ = Phase::INITIAL_STOP; elapsed_ = 0.0; time_ = 0.0;
  }
  Phase phase() const { return phase_; }
  double time() const { return time_; }
  bool active() const {
    return phase_ != Phase::IDLE && phase_ != Phase::COMPLETE && phase_ != Phase::FAULT;
  }
  bool stopping() const {
    return phase_ == Phase::INITIAL_STOP || phase_ == Phase::FORWARD_STOP ||
           phase_ == Phase::ORIGIN_STOP || phase_ == Phase::ENTRY_STOP;
  }
  double heading() const { return origin_.yaw; }
  double remaining(CornerShuntPose pose) const {
    const double along = std::cos(origin_.yaw)*(pose.x-origin_.x) +
                         std::sin(origin_.yaw)*(pose.y-origin_.y);
    const double target = phase_ == Phase::FORWARD ? probe_ :
                          phase_ == Phase::RETURN ? 0.0 : entry_;
    return (phase_ == Phase::RETURN ? -1.0 : 1.0)*(target-along);
  }
  int direction() const { return phase_ == Phase::RETURN ? -1 : 1; }
  void update(CornerShuntPose pose, bool stopped, double dt) {
    if (!active()) return;
    time_ += dt; elapsed_ += dt;
    if (elapsed_ >= timeout_) { phase_ = Phase::FAULT; return; }
    if (stopping()) {
      if (!stopped) return;
      phase_ = phase_ == Phase::INITIAL_STOP ? Phase::FORWARD :
               phase_ == Phase::FORWARD_STOP ? Phase::RETURN :
               phase_ == Phase::ORIGIN_STOP ? Phase::ENTRY : Phase::COMPLETE;
      elapsed_ = 0.0;
    } else if (remaining(pose) <= tolerance_) {
      phase_ = phase_ == Phase::FORWARD ? Phase::FORWARD_STOP :
               phase_ == Phase::RETURN ? Phase::ORIGIN_STOP : Phase::ENTRY_STOP;
      elapsed_ = 0.0;
    }
  }
private:
  Phase phase_{Phase::IDLE};
  CornerShuntPose origin_;
  double probe_{0.0}, entry_{0.0}, tolerance_{0.0}, timeout_{0.0};
  double elapsed_{0.0}, time_{0.0};
};
} // namespace visual_navigation
#endif
