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

struct RecoveryTraceSample {
  RecoveryPoint point;
  double yaw{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
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

enum class RetreatExtensionResult { ACCEPTED, EXHAUSTED, INVALID_STATE };

struct RecoveryReplayStep {
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double position_error{0.0};
  double yaw_error{0.0};
  bool command_valid{false};
  bool publish_recorded{false};
  ObstacleRecoveryAction action{ObstacleRecoveryAction::NONE};
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
  double attempt_timeout{30.0};
  double replay_start_yaw_tolerance{0.03};
  double replay_start_max_yaw_error{0.70};
  double replay_alignment_gain{1.20};
  double replay_alignment_max_angular_speed{0.15};
  double replay_yaw_tolerance{0.25};
  double replay_fallback_linear_speed{0.035};
  int maximum_recovery_attempts{2};
  std::size_t maximum_breadcrumbs{2000};
  std::size_t maximum_trace_samples{12000};
};

// Records only the path actually driven since route activation. On an obstacle
// event an index selects the first recovery anchor while older breadcrumbs are
// retained for bounded retreat extensions requested by the mission layer.
// Reverse control remains independent of the forward follower.
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
        NonnegativeOrDefault(config_.attempt_timeout, 30.0);
    config_.replay_start_yaw_tolerance =
        NonnegativeOrDefault(config_.replay_start_yaw_tolerance, 0.03);
    config_.replay_start_max_yaw_error =
        NonnegativeOrDefault(config_.replay_start_max_yaw_error, 0.70);
    config_.replay_alignment_gain =
        NonnegativeOrDefault(config_.replay_alignment_gain, 1.20);
    config_.replay_alignment_max_angular_speed =
        NonnegativeOrDefault(config_.replay_alignment_max_angular_speed, 0.15);
    config_.replay_yaw_tolerance =
        NonnegativeOrDefault(config_.replay_yaw_tolerance, 0.25);
    config_.replay_fallback_linear_speed = std::max(
        0.035, NonnegativeOrDefault(config_.replay_fallback_linear_speed, 0.035));
    config_.maximum_recovery_attempts =
        std::max(0, config_.maximum_recovery_attempts);
    config_.maximum_breadcrumbs =
        std::max<std::size_t>(2, config_.maximum_breadcrumbs);
    config_.maximum_trace_samples =
        std::max<std::size_t>(2, config_.maximum_trace_samples);
    reset();
  }

  void reset() {
    phase_ = ObstacleRecoveryPhase::IDLE;
    breadcrumbs_.clear();
    trace_.clear();
    anchor_index_ = 0;
    target_index_ = 0;
    target_valid_ = false;
    trace_anchor_index_ = 0;
    replay_index_ = 0;
    replay_stop_index_ = 0;
    replay_active_ = false;
    replay_pose_aligned_ = false;
    replay_started_ = false;
    replay_start_valid_ = false;
    replay_goal_valid_ = false;
    replay_progress_initialized_ = false;
    recovery_attempts_ = 0;
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
    best_remaining_length_ = std::numeric_limits<double>::infinity();
  }

  void reset_trace() {
    breadcrumbs_.clear();
    trace_.clear();
    anchor_index_ = 0;
    target_index_ = 0;
    target_valid_ = false;
    trace_anchor_index_ = 0;
    replay_index_ = 0;
    replay_stop_index_ = 0;
    replay_active_ = false;
    replay_pose_aligned_ = false;
    replay_started_ = false;
    replay_start_valid_ = false;
    replay_goal_valid_ = false;
    replay_progress_initialized_ = false;
  }

  void prepare_for_route(bool preserve_trace) {
    if (!preserve_trace) {
      reset();
      return;
    }
    phase_ = ObstacleRecoveryPhase::IDLE;
    anchor_index_ = breadcrumbs_.empty() ? 0 : breadcrumbs_.size() - 1;
    target_index_ = anchor_index_;
    target_valid_ = false;
    trace_anchor_index_ = trace_.empty() ? 0 : trace_.size() - 1;
    replay_index_ = 0;
    replay_stop_index_ = 0;
    replay_active_ = false;
    replay_pose_aligned_ = false;
    replay_started_ = false;
    replay_start_valid_ = false;
    replay_goal_valid_ = false;
    replay_progress_initialized_ = false;
    recovery_attempts_ = 0;
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
    best_remaining_length_ = std::numeric_limits<double>::infinity();
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
    record_sample(x, y, 0.0, 0.0, 0.0, force);
  }

  void record_sample(double x, double y, double yaw, double linear_velocity,
                     double angular_velocity, bool force = false) {
    if (phase_ != ObstacleRecoveryPhase::IDLE || !std::isfinite(x) ||
        !std::isfinite(y) || !std::isfinite(yaw) ||
        !std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)) {
      return;
    }
    const RecoveryPoint point{x, y};
    if (!trace_.empty() && RecoveryPointDistance(point, trace_.back().point) >
                               config_.maximum_breadcrumb_step) {
      return;
    }
    AppendTraceSample({point, yaw, linear_velocity, angular_velocity});
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
    return trigger(current_x, current_y, 0.0, 0.0, 0.0, anchor_x, anchor_y);
  }

  bool trigger(double current_x, double current_y, double current_yaw,
               double current_linear_velocity, double current_angular_velocity,
               double anchor_x, double anchor_y) {
    if (phase_ != ObstacleRecoveryPhase::IDLE ||
        !CoordinatesAreFinite(current_x, current_y) ||
        !CoordinatesAreFinite(anchor_x, anchor_y) ||
        !std::isfinite(current_yaw) ||
        !std::isfinite(current_linear_velocity) ||
        !std::isfinite(current_angular_velocity)) {
      return false;
    }

    if (breadcrumbs_.empty())
      AppendPoint({anchor_x, anchor_y});
    AppendPoint({current_x, current_y});
    anchor_index_ = ClosestAnchorIndex({anchor_x, anchor_y});
    AppendTraceSample({{current_x, current_y},
                       current_yaw,
                       current_linear_velocity,
                       current_angular_velocity});
    trace_anchor_index_ = ClosestTraceAnchorIndex({anchor_x, anchor_y});
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
    if (initial_brake && !replay_active_ &&
        CoordinatesAreFinite(current_x, current_y)) {
      if (breadcrumbs_.empty())
        breadcrumbs_.push_back({current_x, current_y});
      else
        breadcrumbs_.back() = {current_x, current_y};
    }
    phase_ = ObstacleRecoveryPhase::REVERSING;
    if (!replay_active_)
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
    if (RecoveryPointDistance(current, breadcrumbs_[anchor_index_]) <=
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

  RetreatExtensionResult extend_retreat(
      double minimum_distance,
      double current_x = std::numeric_limits<double>::quiet_NaN(),
      double current_y = std::numeric_limits<double>::quiet_NaN(),
      double current_yaw = std::numeric_limits<double>::quiet_NaN()) {
    if (phase_ != ObstacleRecoveryPhase::REPLAN_REQUIRED)
      return RetreatExtensionResult::INVALID_STATE;
    if (trace_.empty() || trace_anchor_index_ == 0)
      return RetreatExtensionResult::EXHAUSTED;
    if (CoordinatesAreFinite(current_x, current_y) &&
        std::isfinite(current_yaw)) {
      trace_anchor_index_ =
          ClosestTracePoseIndex(current_x, current_y, current_yaw);
      if (trace_anchor_index_ == 0)
        return RetreatExtensionResult::EXHAUSTED;
    }

    const double requested =
        NonnegativeOrDefault(minimum_distance, config_.breadcrumb_spacing);
    const double target_distance =
        std::max(config_.breadcrumb_spacing, requested);
    double accumulated = 0.0;
    std::size_t candidate = trace_anchor_index_;
    while (candidate > 0 && accumulated < target_distance) {
      accumulated += RecoveryPointDistance(trace_[candidate].point,
                                           trace_[candidate - 1].point);
      --candidate;
    }
    if (candidate == trace_anchor_index_)
      return RetreatExtensionResult::EXHAUSTED;

    replay_index_ = trace_anchor_index_;
    replay_stop_index_ = candidate;
    replay_start_ = trace_[trace_anchor_index_];
    replay_start_valid_ = true;
    replay_goal_ = trace_[candidate];
    replay_goal_valid_ = true;
    replay_active_ = true;
    replay_pose_aligned_ = false;
    replay_started_ = false;
    replay_progress_initialized_ = false;
    target_valid_ = false;
    recovery_attempts_ = 0;
    attempt_timer_initialized_ = false;
    progress_timer_initialized_ = false;
    phase_ = ObstacleRecoveryPhase::BRAKING;
    return RetreatExtensionResult::ACCEPTED;
  }

  RetreatExtensionResult begin_retreat(double minimum_distance,
                                       double current_x, double current_y,
                                       double current_yaw) {
    if (phase_ != ObstacleRecoveryPhase::IDLE)
      return RetreatExtensionResult::INVALID_STATE;
    if (trace_.empty() || !CoordinatesAreFinite(current_x, current_y) ||
        !std::isfinite(current_yaw)) {
      return RetreatExtensionResult::EXHAUSTED;
    }
    trace_anchor_index_ =
        ClosestTracePoseIndex(current_x, current_y, current_yaw);
    phase_ = ObstacleRecoveryPhase::REPLAN_REQUIRED;
    return extend_retreat(minimum_distance, current_x, current_y, current_yaw);
  }

  RecoveryReplayStep replay_step(double current_x, double current_y,
                                 double current_yaw, double applied_linear,
                                 double applied_angular, TimePoint now) {
    RecoveryReplayStep step;
    if (phase_ != ObstacleRecoveryPhase::REVERSING || !replay_active_ ||
        !CoordinatesAreFinite(current_x, current_y) ||
        !std::isfinite(current_yaw) || !std::isfinite(applied_linear) ||
        !std::isfinite(applied_angular)) {
      return step;
    }
    const double attempt_seconds =
        std::chrono::duration<double>(now - attempt_started_).count();
    if (config_.attempt_timeout > 0.0 &&
        attempt_seconds >= config_.attempt_timeout) {
      replay_active_ = false;
      phase_ = ObstacleRecoveryPhase::FAULT_TIMEOUT;
      step.action = ObstacleRecoveryAction::FAULT_TIMEOUT;
      return step;
    }
    if (!replay_pose_aligned_) {
      const double position_error =
          RecoveryPointDistance({current_x, current_y}, replay_start_.point);
      const double yaw_error = std::remainder(replay_start_.yaw - current_yaw,
                                              2.0 * 3.14159265358979323846);
      if (position_error > config_.anchor_tolerance ||
          std::abs(yaw_error) > config_.replay_start_max_yaw_error) {
        replay_active_ = false;
        phase_ = ObstacleRecoveryPhase::FAULT_NO_PROGRESS;
        step.position_error = position_error;
        step.yaw_error = std::abs(yaw_error);
        step.action = ObstacleRecoveryAction::FAULT_NO_PROGRESS;
        return step;
      }
      if (std::abs(yaw_error) > config_.replay_start_yaw_tolerance) {
        const double angular_limit =
            std::max(0.0, config_.replay_alignment_max_angular_speed);
        step.angular_velocity = std::max(
            -angular_limit,
            std::min(angular_limit, config_.replay_alignment_gain * yaw_error));
        step.command_valid = true;
        return step;
      }
      replay_pose_aligned_ = true;
    }
    const double goal_distance =
        RecoveryPointDistance({current_x, current_y}, replay_goal_.point);
    if (!replay_progress_initialized_) {
      replay_best_distance_ = goal_distance;
      replay_progress_started_ = now;
      replay_progress_initialized_ = true;
    } else if (replay_best_distance_ - goal_distance >=
               config_.minimum_progress) {
      replay_best_distance_ = goal_distance;
      replay_progress_started_ = now;
    }
    if (config_.no_progress_timeout > 0.0 &&
        std::chrono::duration<double>(now - replay_progress_started_).count() >=
            config_.no_progress_timeout) {
      const auto retry = RetryOrFault(false);
      if (retry.action == ObstacleRecoveryAction::FAULT_NO_PROGRESS)
        replay_active_ = false;
      step.position_error = goal_distance;
      step.action = retry.action;
      return step;
    }

    // Consume a historical sample only after the measured pose has moved
    // from the current sample toward the next one. A control callback alone
    // must never advance the replay cursor.
    if (replay_started_ && replay_index_ > replay_stop_index_) {
      const std::size_t next_index = replay_index_ - 1;
      const double segment_distance = RecoveryPointDistance(
          trace_[replay_index_].point, trace_[next_index].point);
      const double required_motion = std::max(
          0.005, std::min(0.5 * segment_distance, config_.minimum_progress));
      const double moved_from_sample = RecoveryPointDistance(
          {current_x, current_y}, trace_[replay_index_].point);
      if (moved_from_sample >= required_motion &&
          RecoveryPointDistance({current_x, current_y},
                                trace_[next_index].point) <=
              config_.target_tolerance) {
        replay_index_ = next_index;
      }
    }

    if (replay_index_ <= replay_stop_index_) {
      const double position_error = goal_distance;
      const double yaw_error = std::abs(std::remainder(
          replay_goal_.yaw - current_yaw, 2.0 * 3.14159265358979323846));
      step.position_error = position_error;
      step.yaw_error = yaw_error;
      replay_active_ = false;
      if (position_error > config_.anchor_tolerance ||
          yaw_error > config_.replay_yaw_tolerance) {
        phase_ = ObstacleRecoveryPhase::FAULT_NO_PROGRESS;
        step.action = ObstacleRecoveryAction::FAULT_NO_PROGRESS;
        return step;
      }
      trace_anchor_index_ = replay_stop_index_;
      phase_ = ObstacleRecoveryPhase::REPLAN_REQUIRED;
      step.action = ObstacleRecoveryAction::REPLAN_REQUIRED;
      return step;
    }

    const auto &sample = trace_[replay_index_];
    double replay_speed = std::abs(sample.linear_velocity);
    if (replay_speed < 1.0e-6) {
      for (std::size_t index = replay_index_; index > replay_stop_index_;
           --index) {
        const double candidate = std::abs(trace_[index].linear_velocity);
        if (candidate >= 1.0e-6) {
          replay_speed = candidate;
          break;
        }
      }
    }
    if (replay_speed < 1.0e-6)
      replay_speed = config_.replay_fallback_linear_speed;
    step.linear_velocity = -replay_speed;
    step.angular_velocity = -sample.angular_velocity;
    step.command_valid = true;
    constexpr double kCommandMatchTolerance = 1.0e-4;
    if (!replay_started_) {
      if (std::abs(applied_linear - step.linear_velocity) <=
              kCommandMatchTolerance &&
          std::abs(applied_angular - step.angular_velocity) <=
              kCommandMatchTolerance) {
        replay_started_ = true;
      }
    } else {
      step.publish_recorded = true;
    }
    return step;
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
  std::size_t remaining_breadcrumbs_before_anchor() const {
    if (replay_goal_valid_)
      return replay_stop_index_;
    return trace_.empty() ? anchor_index_ : trace_anchor_index_;
  }
  bool replaying() const { return replay_active_; }
  RecoveryTraceSample replay_start() const {
    return replay_start_valid_ ? replay_start_ : RecoveryTraceSample{};
  }
  RecoveryTraceSample replay_goal() const {
    return replay_goal_valid_ ? replay_goal_ : RecoveryTraceSample{};
  }
  RecoveryPoint anchor() const {
    if (replay_goal_valid_)
      return replay_goal_.point;
    return breadcrumbs_.empty() ? RecoveryPoint{} : breadcrumbs_[anchor_index_];
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

  void AppendTraceSample(const RecoveryTraceSample &sample) {
    if (!trace_.empty() &&
        RecoveryPointDistance(sample.point, trace_.back().point) <= 1.0e-9 &&
        std::abs(std::remainder(sample.yaw - trace_.back().yaw,
                                2.0 * 3.14159265358979323846)) <= 1.0e-6 &&
        std::abs(sample.linear_velocity - trace_.back().linear_velocity) <=
            1.0e-6 &&
        std::abs(sample.angular_velocity - trace_.back().angular_velocity) <=
            1.0e-6) {
      trace_.back() = sample;
      return;
    }
    trace_.push_back(sample);
    if (trace_.size() > config_.maximum_trace_samples)
      trace_.erase(trace_.begin());
  }

  std::size_t ClosestAnchorIndex(const RecoveryPoint &nominal_anchor) const {
    if (breadcrumbs_.empty())
      return 0;
    std::size_t closest_index = 0;
    double closest_distance =
        RecoveryPointDistance(breadcrumbs_.front(), nominal_anchor);
    std::size_t latest_recovery_visit = breadcrumbs_.size();
    // A turn can pass near its semantic junction more than once. Prefer the
    // latest point inside the recovery tolerance so the trace starts on the
    // lane the vehicle actually exited, not on the earlier approach branch.
    for (std::size_t index = 1; index < breadcrumbs_.size(); ++index) {
      const double distance =
          RecoveryPointDistance(breadcrumbs_[index], nominal_anchor);
      if (distance <= config_.anchor_tolerance)
        latest_recovery_visit = index;
      if (distance <= closest_distance) {
        closest_distance = distance;
        closest_index = index;
      }
    }
    return latest_recovery_visit < breadcrumbs_.size() ? latest_recovery_visit
                                                       : closest_index;
  }

  std::size_t
  ClosestTraceAnchorIndex(const RecoveryPoint &nominal_anchor) const {
    if (trace_.empty())
      return 0;
    std::size_t closest_index = 0;
    double closest_distance =
        RecoveryPointDistance(trace_.front().point, nominal_anchor);
    std::size_t latest_recovery_visit = trace_.size();
    for (std::size_t index = 1; index < trace_.size(); ++index) {
      const double distance =
          RecoveryPointDistance(trace_[index].point, nominal_anchor);
      if (distance <= config_.anchor_tolerance)
        latest_recovery_visit = index;
      if (distance <= closest_distance) {
        closest_distance = distance;
        closest_index = index;
      }
    }
    return latest_recovery_visit < trace_.size() ? latest_recovery_visit
                                                 : closest_index;
  }

  std::size_t ClosestTracePoseIndex(double x, double y, double yaw) const {
    std::size_t closest_index = 0;
    double closest_score = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < trace_.size(); ++index) {
      const double position_error =
          RecoveryPointDistance({x, y}, trace_[index].point);
      const double yaw_error = std::abs(std::remainder(
          trace_[index].yaw - yaw, 2.0 * 3.14159265358979323846));
      const double score = position_error + 0.08 * yaw_error;
      if (score < closest_score) {
        closest_score = score;
        closest_index = index;
      }
    }
    return closest_index;
  }

  void SelectTargetForPosition(const RecoveryPoint &current) {
    if (breadcrumbs_.size() < 2) {
      target_index_ = 0;
      target_valid_ = !breadcrumbs_.empty();
      return;
    }
    std::size_t closest_index = anchor_index_;
    double closest_distance =
        RecoveryPointDistance(current, breadcrumbs_[anchor_index_]);
    for (std::size_t index = anchor_index_ + 1; index < breadcrumbs_.size();
         ++index) {
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
    while (index > anchor_index_ && accumulated < config_.reverse_lookahead) {
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
    if (target_index_ == anchor_index_)
      return;
    --target_index_;
    SelectLookaheadTarget();
  }

  double RemainingLength(const RecoveryPoint &current) const {
    if (!target_valid_ || breadcrumbs_.empty() ||
        target_index_ >= breadcrumbs_.size()) {
      return RecoveryPointDistance(current, breadcrumbs_[anchor_index_]);
    }
    double remaining =
        RecoveryPointDistance(current, breadcrumbs_[target_index_]);
    for (std::size_t index = target_index_; index > anchor_index_; --index) {
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
  std::vector<RecoveryTraceSample> trace_;
  std::size_t anchor_index_{0};
  std::size_t target_index_{0};
  bool target_valid_{false};
  std::size_t trace_anchor_index_{0};
  std::size_t replay_index_{0};
  std::size_t replay_stop_index_{0};
  bool replay_active_{false};
  bool replay_pose_aligned_{false};
  bool replay_started_{false};
  RecoveryTraceSample replay_start_;
  bool replay_start_valid_{false};
  RecoveryTraceSample replay_goal_;
  bool replay_goal_valid_{false};
  bool replay_progress_initialized_{false};
  double replay_best_distance_{std::numeric_limits<double>::infinity()};
  TimePoint replay_progress_started_{};
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
  command.linear_velocity =
      -std::max(0.0, requested_speed) *
      std::max(heading_scale,
               std::abs(command.heading_error) > heading_limit ? 0.35 : 0.0);
  const double angular_limit = std::max(0.0, maximum_angular_speed);
  const double effective_angular_limit =
      std::abs(command.heading_error) > heading_limit
          ? std::min(angular_limit, 0.15)
          : angular_limit;
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
