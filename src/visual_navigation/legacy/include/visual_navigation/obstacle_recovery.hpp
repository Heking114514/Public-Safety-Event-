#ifndef VISUAL_NAVIGATION__OBSTACLE_RECOVERY_HPP_
#define VISUAL_NAVIGATION__OBSTACLE_RECOVERY_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace visual_navigation {

struct RecoveryPoint {
  double x{0.0};
  double y{0.0};
};

inline double RecoveryPointDistance(const RecoveryPoint &first,
                                    const RecoveryPoint &second) {
  return std::hypot(first.x - second.x, first.y - second.y);
}

// A wall seen beyond a planned stop-and-turn point belongs to normal arena
// geometry. The range is measured from the sensor; both offsets are expressed
// forward from base_link. Unknown range cannot suppress a blocked event.
inline bool UnexpectedObstacle(bool blocked, bool range_fresh,
                               double sensor_range, bool planned_turn_stop,
                               double distance_to_turn,
                               double sensor_forward_offset,
                               double vehicle_front_offset,
                               double comparison_tolerance) {
  if (!blocked)
    return false;
  if (!range_fresh || !planned_turn_stop || !std::isfinite(sensor_range) ||
      sensor_range < 0.0 || !std::isfinite(distance_to_turn) ||
      distance_to_turn < 0.0 || !std::isfinite(sensor_forward_offset) ||
      !std::isfinite(vehicle_front_offset) ||
      !std::isfinite(comparison_tolerance)) {
    return true;
  }
  const double clearance_from_vehicle_front =
      sensor_range + sensor_forward_offset - vehicle_front_offset;
  return clearance_from_vehicle_front + std::max(0.0, comparison_tolerance) <
         distance_to_turn;
}

enum class ObstacleRecoveryPhase {
  IDLE,
  BRAKING,
  REVERSING,
  RETRY_BRAKING,
  REPLAN_REQUIRED,
  FAULT_NO_PROGRESS,
  FAULT_TIMEOUT
};

enum class ObstacleRecoveryAction {
  NONE,
  RETRY_BRAKING,
  REPLAN_REQUIRED,
  FAULT_NO_PROGRESS,
  FAULT_TIMEOUT
};

struct ObstacleRecoveryDecision {
  ObstacleRecoveryAction action{ObstacleRecoveryAction::NONE};
  int recovery_attempt{0};
};

struct ObstacleRecoveryConfig {
  double breadcrumb_spacing{0.04};
  double maximum_breadcrumb_step{0.25};
  double reverse_lookahead{0.12};
  double target_tolerance{0.06};
  double anchor_tolerance{0.08};
  double minimum_progress{0.03};
  double no_progress_timeout{3.0};
  double attempt_timeout{20.0};
  int maximum_recovery_attempts{2};
  std::size_t maximum_breadcrumbs{2000};
};

// Records only the path actually driven since route activation. On an obstacle
// event the trace is trimmed to the selected road-junction anchor and consumed
// backwards. This keeps reverse control independent of the forward follower.
class ObstacleRecoveryController {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  ObstacleRecoveryController() = default;

  explicit ObstacleRecoveryController(const ObstacleRecoveryConfig &config) {
    configure(config);
  }

  void configure(const ObstacleRecoveryConfig &config) {
    config_ = config;
    config_.breadcrumb_spacing =
        NonnegativeOrDefault(config_.breadcrumb_spacing, 0.04);
    config_.maximum_breadcrumb_step =
        std::max(config_.breadcrumb_spacing,
                 NonnegativeOrDefault(config_.maximum_breadcrumb_step, 0.25));
    config_.reverse_lookahead =
        NonnegativeOrDefault(config_.reverse_lookahead, 0.12);
    config_.target_tolerance =
        NonnegativeOrDefault(config_.target_tolerance, 0.06);
    config_.anchor_tolerance =
        NonnegativeOrDefault(config_.anchor_tolerance, 0.08);
    config_.minimum_progress =
        NonnegativeOrDefault(config_.minimum_progress, 0.03);
    config_.no_progress_timeout =
        NonnegativeOrDefault(config_.no_progress_timeout, 3.0);
    config_.attempt_timeout =
        NonnegativeOrDefault(config_.attempt_timeout, 20.0);
    config_.maximum_recovery_attempts =
        std::max(0, config_.maximum_recovery_attempts);
    config_.maximum_breadcrumbs =
        std::max<std::size_t>(2, config_.maximum_breadcrumbs);
    reset();
  }

  void reset() {
    phase_ = ObstacleRecoveryPhase::IDLE;
    breadcrumbs_.clear();
    target_index_ = 0;
    target_valid_ = false;
    recovery_attempts_ = 0;
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
    best_remaining_length_ = std::numeric_limits<double>::infinity();
  }

  void reset_trace() {
    breadcrumbs_.clear();
    target_index_ = 0;
    target_valid_ = false;
  }

  // A latched navigation fault stops the control loop before obstacle retreat
  // can finish. Leave the controller in the same replaceable state used after
  // reaching the recovery anchor so a fresh route can restart from the current
  // pose instead of being rejected forever as "still reversing".
  void require_route_takeover() {
    phase_ = ObstacleRecoveryPhase::REPLAN_REQUIRED;
    target_valid_ = false;
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
  }

  void record_pose(double x, double y, bool force = false) {
    if (phase_ != ObstacleRecoveryPhase::IDLE || !std::isfinite(x) ||
        !std::isfinite(y)) {
      return;
    }
    const RecoveryPoint point{x, y};
    if (!breadcrumbs_.empty()) {
      const double step = RecoveryPointDistance(point, breadcrumbs_.back());
      // Fused odometry is expected to be continuous. Do not turn one rejected
      // localization jump into a long reverse segment; a later normal sample
      // can continue from the last trusted breadcrumb.
      if (step > config_.maximum_breadcrumb_step)
        return;
      if (!force && step < config_.breadcrumb_spacing)
        return;
    }
    AppendPoint(point);
  }

  bool trigger(double current_x, double current_y, double anchor_x,
               double anchor_y) {
    if (phase_ != ObstacleRecoveryPhase::IDLE ||
        !CoordinatesAreFinite(current_x, current_y) ||
        !CoordinatesAreFinite(anchor_x, anchor_y)) {
      return false;
    }

    if (breadcrumbs_.empty())
      AppendPoint({anchor_x, anchor_y});
    AppendPoint({current_x, current_y});
    TrimToClosestAnchor({anchor_x, anchor_y});
    // Keep the measured trace endpoint exact even if the last regular sample
    // was closer than breadcrumb_spacing.
    if (breadcrumbs_.empty() ||
        RecoveryPointDistance(breadcrumbs_.back(), {current_x, current_y}) >
            1.0e-9) {
      AppendPoint({current_x, current_y});
    } else {
      breadcrumbs_.back() = {current_x, current_y};
    }

    phase_ = ObstacleRecoveryPhase::BRAKING;
    recovery_attempts_ = 0;
    target_valid_ = false;
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
    return true;
  }

  void confirm_stopped(double current_x, double current_y, TimePoint now) {
    if (phase_ != ObstacleRecoveryPhase::BRAKING &&
        phase_ != ObstacleRecoveryPhase::RETRY_BRAKING) {
      return;
    }
    const bool initial_brake = phase_ == ObstacleRecoveryPhase::BRAKING;
    if (initial_brake && CoordinatesAreFinite(current_x, current_y)) {
      if (breadcrumbs_.empty())
        breadcrumbs_.push_back({current_x, current_y});
      else
        breadcrumbs_.back() = {current_x, current_y};
    }
    phase_ = ObstacleRecoveryPhase::REVERSING;
    SelectTargetForPosition({current_x, current_y});
    BeginAttempt(current_x, current_y, now);
  }

  ObstacleRecoveryDecision update_reverse(double current_x, double current_y,
                                          TimePoint now) {
    if (phase_ != ObstacleRecoveryPhase::REVERSING ||
        !CoordinatesAreFinite(current_x, current_y) || breadcrumbs_.empty()) {
      return {};
    }

    const RecoveryPoint current{current_x, current_y};
    if (RecoveryPointDistance(current, breadcrumbs_.front()) <=
        config_.anchor_tolerance) {
      phase_ = ObstacleRecoveryPhase::REPLAN_REQUIRED;
      target_valid_ = false;
      return {ObstacleRecoveryAction::REPLAN_REQUIRED, recovery_attempts_};
    }

    AdvanceTargetWhenReached(current);
    const double remaining = RemainingLength(current);
    if (!attempt_timer_initialized_ || !progress_timer_initialized_)
      BeginAttempt(current_x, current_y, now);
    if (best_remaining_length_ - remaining >= config_.minimum_progress) {
      best_remaining_length_ = remaining;
      progress_started_ = now;
    }

    const double attempt_seconds =
        std::chrono::duration<double>(now - attempt_started_).count();
    const double no_progress_seconds =
        std::chrono::duration<double>(now - progress_started_).count();
    if (config_.attempt_timeout > 0.0 &&
        attempt_seconds >= config_.attempt_timeout) {
      return RetryOrFault(true);
    }
    if (config_.no_progress_timeout > 0.0 &&
        no_progress_seconds >= config_.no_progress_timeout) {
      return RetryOrFault(false);
    }
    return {};
  }

  bool reverse_target(RecoveryPoint &target) const {
    if (phase_ != ObstacleRecoveryPhase::REVERSING || !target_valid_ ||
        target_index_ >= breadcrumbs_.size()) {
      return false;
    }
    target = breadcrumbs_[target_index_];
    return true;
  }

  ObstacleRecoveryPhase phase() const { return phase_; }
  bool idle() const { return phase_ == ObstacleRecoveryPhase::IDLE; }
  bool braking() const {
    return phase_ == ObstacleRecoveryPhase::BRAKING ||
           phase_ == ObstacleRecoveryPhase::RETRY_BRAKING;
  }
  bool reversing() const { return phase_ == ObstacleRecoveryPhase::REVERSING; }
  bool replan_required() const {
    return phase_ == ObstacleRecoveryPhase::REPLAN_REQUIRED;
  }
  bool faulted() const {
    return phase_ == ObstacleRecoveryPhase::FAULT_NO_PROGRESS ||
           phase_ == ObstacleRecoveryPhase::FAULT_TIMEOUT;
  }
  bool route_may_take_over() const { return replan_required() || faulted(); }
  int recovery_attempts() const { return recovery_attempts_; }
  std::size_t breadcrumb_count() const { return breadcrumbs_.size(); }
  RecoveryPoint anchor() const {
    return breadcrumbs_.empty() ? RecoveryPoint{} : breadcrumbs_.front();
  }

private:
  static double NonnegativeOrDefault(double value, double fallback) {
    return std::isfinite(value) ? std::max(0.0, value) : fallback;
  }

  static bool CoordinatesAreFinite(double x, double y) {
    return std::isfinite(x) && std::isfinite(y);
  }

  void AppendPoint(const RecoveryPoint &point) {
    if (!breadcrumbs_.empty() &&
        RecoveryPointDistance(point, breadcrumbs_.back()) <= 1.0e-9) {
      breadcrumbs_.back() = point;
      return;
    }
    breadcrumbs_.push_back(point);
    if (breadcrumbs_.size() > config_.maximum_breadcrumbs)
      breadcrumbs_.erase(breadcrumbs_.begin());
  }

  void TrimToClosestAnchor(const RecoveryPoint &nominal_anchor) {
    if (breadcrumbs_.empty())
      return;
    std::size_t closest_index = 0;
    double closest_distance =
        RecoveryPointDistance(breadcrumbs_.front(), nominal_anchor);
    // Prefer the latest equally-close visit when a route crosses the same
    // junction more than once.
    for (std::size_t index = 1; index < breadcrumbs_.size(); ++index) {
      const double distance =
          RecoveryPointDistance(breadcrumbs_[index], nominal_anchor);
      if (distance <= closest_distance) {
        closest_distance = distance;
        closest_index = index;
      }
    }
    if (closest_index > 0) {
      breadcrumbs_.erase(breadcrumbs_.begin(),
                         breadcrumbs_.begin() + closest_index);
    }
  }

  void SelectTargetForPosition(const RecoveryPoint &current) {
    if (breadcrumbs_.size() < 2) {
      target_index_ = 0;
      target_valid_ = !breadcrumbs_.empty();
      return;
    }
    std::size_t closest_index = 0;
    double closest_distance =
        RecoveryPointDistance(current, breadcrumbs_.front());
    for (std::size_t index = 1; index < breadcrumbs_.size(); ++index) {
      const double distance =
          RecoveryPointDistance(current, breadcrumbs_[index]);
      // Equal distances select the earlier point, which is always toward the
      // anchor rather than back toward the obstacle.
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = index;
      }
    }
    target_index_ = closest_index;
    SelectLookaheadTarget();
    target_valid_ = true;
  }

  void SelectLookaheadTarget() {
    if (breadcrumbs_.empty()) {
      target_valid_ = false;
      return;
    }
    double accumulated = 0.0;
    std::size_t index = target_index_;
    while (index > 0 && accumulated < config_.reverse_lookahead) {
      accumulated +=
          RecoveryPointDistance(breadcrumbs_[index], breadcrumbs_[index - 1]);
      --index;
    }
    target_index_ = index;
  }

  void AdvanceTargetWhenReached(const RecoveryPoint &current) {
    if (!target_valid_ || target_index_ >= breadcrumbs_.size())
      return;
    if (RecoveryPointDistance(current, breadcrumbs_[target_index_]) >
        config_.target_tolerance) {
      return;
    }
    if (target_index_ == 0)
      return;
    --target_index_;
    SelectLookaheadTarget();
  }

  double RemainingLength(const RecoveryPoint &current) const {
    if (!target_valid_ || breadcrumbs_.empty() ||
        target_index_ >= breadcrumbs_.size()) {
      return RecoveryPointDistance(current, breadcrumbs_.front());
    }
    double remaining =
        RecoveryPointDistance(current, breadcrumbs_[target_index_]);
    for (std::size_t index = target_index_; index > 0; --index) {
      remaining +=
          RecoveryPointDistance(breadcrumbs_[index], breadcrumbs_[index - 1]);
    }
    return remaining;
  }

  void BeginAttempt(double current_x, double current_y, TimePoint now) {
    attempt_started_ = now;
    progress_started_ = now;
    attempt_timer_initialized_ = true;
    progress_timer_initialized_ = true;
    best_remaining_length_ = RemainingLength({current_x, current_y});
  }

  ObstacleRecoveryDecision RetryOrFault(bool timed_out) {
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
    if (recovery_attempts_ < config_.maximum_recovery_attempts) {
      ++recovery_attempts_;
      phase_ = ObstacleRecoveryPhase::RETRY_BRAKING;
      return {ObstacleRecoveryAction::RETRY_BRAKING, recovery_attempts_};
    }
    phase_ = timed_out ? ObstacleRecoveryPhase::FAULT_TIMEOUT
                       : ObstacleRecoveryPhase::FAULT_NO_PROGRESS;
    return {timed_out ? ObstacleRecoveryAction::FAULT_TIMEOUT
                      : ObstacleRecoveryAction::FAULT_NO_PROGRESS,
            recovery_attempts_};
  }

  ObstacleRecoveryConfig config_;
  ObstacleRecoveryPhase phase_{ObstacleRecoveryPhase::IDLE};
  std::vector<RecoveryPoint> breadcrumbs_;
  std::size_t target_index_{0};
  bool target_valid_{false};
  int recovery_attempts_{0};
  TimePoint attempt_started_{};
  TimePoint progress_started_{};
  bool attempt_timer_initialized_{false};
  bool progress_timer_initialized_{false};
  double best_remaining_length_{std::numeric_limits<double>::infinity()};
};

struct ReverseMotionCommand {
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double heading_error{0.0};
  bool valid{false};
};

// Reverse tracking has deliberately different signs and limits from forward
// Stanley tracking. The chassis keeps facing away from the retreat target while
// negative linear velocity moves the rear of the car along the recorded trace.
inline ReverseMotionCommand ComputeReverseMotionCommand(
    double current_x, double current_y, double current_yaw,
    const RecoveryPoint &target, double requested_speed, double angular_gain,
    double yaw_rate, double yaw_rate_damping, double maximum_angular_speed,
    double maximum_heading_error) {
  ReverseMotionCommand command;
  if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
      !std::isfinite(current_yaw) || !std::isfinite(target.x) ||
      !std::isfinite(target.y) || !std::isfinite(requested_speed) ||
      !std::isfinite(angular_gain) || !std::isfinite(yaw_rate) ||
      !std::isfinite(yaw_rate_damping) ||
      !std::isfinite(maximum_angular_speed) ||
      !std::isfinite(maximum_heading_error)) {
    return command;
  }
  const double target_bearing =
      std::atan2(target.y - current_y, target.x - current_x);
  constexpr double kRecoveryPi = 3.14159265358979323846;
  const double desired_chassis_yaw =
      std::remainder(target_bearing + kRecoveryPi, 2.0 * kRecoveryPi);
  command.heading_error =
      std::remainder(desired_chassis_yaw - current_yaw, 2.0 * kRecoveryPi);
  const double heading_scale = std::max(0.0, std::cos(command.heading_error));
  const double heading_limit = std::max(0.0, maximum_heading_error);
  if (std::abs(command.heading_error) > 1.35) {
    command.valid = true;
    return command;
  }
  command.linear_velocity = -std::max(0.0, requested_speed) *
    std::max(heading_scale, std::abs(command.heading_error) > heading_limit ? 0.35 : 0.0);
  const double angular_limit = std::max(0.0, maximum_angular_speed);
  const double effective_angular_limit = std::abs(command.heading_error) > heading_limit ?
    std::min(angular_limit, 0.15) : angular_limit;
  command.angular_velocity =
      std::max(-effective_angular_limit,
               std::min(effective_angular_limit,
                        std::max(0.0, angular_gain) * command.heading_error -
                            std::max(0.0, yaw_rate_damping) * yaw_rate));
  command.valid = true;
  return command;
}

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__OBSTACLE_RECOVERY_HPP_
