#ifndef VISUAL_NAVIGATION__PATH_PROGRESS_SUPERVISOR_HPP_
#define VISUAL_NAVIGATION__PATH_PROGRESS_SUPERVISOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace visual_navigation
{

enum class PathProgressAction
{
  IDLE,
  MONITORING,
  RECOVER,
  FAULT
};

struct PathProgressConfig
{
  double timeout{10.0};
  double minimum_advance{0.05};
  double minimum_actual_displacement{0.25};
  int maximum_recovery_attempts{2};
  // Independent stall check: a sustained, meaningful forward command with
  // almost no measured translation must not wait for the 0.25 m displacement
  // gate used by the ordinary path-progress check.
  double command_threshold{0.08};
  double stationary_speed_threshold{0.02};
  double no_motion_timeout{3.0};
  double no_motion_displacement{0.05};
};

struct PathProgressDecision
{
  PathProgressAction action{PathProgressAction::IDLE};
  int recovery_attempt{0};
};

class PathProgressSupervisor
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  PathProgressSupervisor() = default;

  explicit PathProgressSupervisor(const PathProgressConfig &config)
  : config_(config)
  {
    config_.timeout = NonnegativeOrDefault(config_.timeout, 10.0);
    config_.minimum_advance =
      NonnegativeOrDefault(config_.minimum_advance, 0.05);
    config_.minimum_actual_displacement =
      NonnegativeOrDefault(config_.minimum_actual_displacement, 0.25);
    config_.maximum_recovery_attempts =
      std::max(0, config_.maximum_recovery_attempts);
    config_.command_threshold =
      NonnegativeOrDefault(config_.command_threshold, 0.08);
    config_.stationary_speed_threshold =
      NonnegativeOrDefault(config_.stationary_speed_threshold, 0.02);
    config_.no_motion_timeout =
      NonnegativeOrDefault(config_.no_motion_timeout, 3.0);
    config_.no_motion_displacement =
      NonnegativeOrDefault(config_.no_motion_displacement, 0.05);
  }

  PathProgressDecision Evaluate(
    std::size_t segment_id, double measured_progress,
    double current_x, double current_y,
    bool monitoring_enabled, TimePoint control_time,
    double commanded_linear_velocity = 0.0,
    double measured_linear_velocity = 0.0,
    bool measured_linear_velocity_valid = false,
    bool linear_motion_expected = true)
  {
    if (!monitoring_enabled || config_.timeout <= 0.0 ||
      !std::isfinite(measured_progress) ||
      !std::isfinite(current_x) || !std::isfinite(current_y))
    {
      Pause();
      return {};
    }

    if (!segment_initialized_ || segment_id != segment_id_)
      BeginSegment(segment_id);

    if (fault_latched_)
      return {PathProgressAction::FAULT, recovery_attempts_};

    if (!window_initialized_)
    {
      best_progress_ = measured_progress;
      window_start_progress_ = measured_progress;
      window_start_x_ = current_x;
      window_start_y_ = current_y;
      maximum_displacement_ = 0.0;
      window_started_ = control_time;
      window_initialized_ = true;
      // Still initialize the independent no-motion timer on this first valid
      // sample when a real command is already active.
      UpdateNoMotionState(
        current_x, current_y, commanded_linear_velocity,
        measured_linear_velocity, measured_linear_velocity_valid,
        linear_motion_expected, control_time);
      return {PathProgressAction::MONITORING, recovery_attempts_};
    }

    const PathProgressDecision no_motion_decision = UpdateNoMotionState(
      current_x, current_y, commanded_linear_velocity,
      measured_linear_velocity, measured_linear_velocity_valid,
      linear_motion_expected, control_time);
    if (no_motion_decision.action != PathProgressAction::IDLE)
      return no_motion_decision;

    maximum_displacement_ = std::max(
      maximum_displacement_,
      std::hypot(current_x - window_start_x_, current_y - window_start_y_));
    best_progress_ = std::max(best_progress_, measured_progress);
    if (best_progress_ - window_start_progress_ >= config_.minimum_advance)
    {
      window_start_progress_ = best_progress_;
      window_start_x_ = current_x;
      window_start_y_ = current_y;
      maximum_displacement_ = 0.0;
      window_started_ = control_time;
      return {PathProgressAction::MONITORING, recovery_attempts_};
    }

    const double stalled_seconds =
      std::chrono::duration<double>(control_time - window_started_).count();
    if (stalled_seconds < config_.timeout)
      return {PathProgressAction::MONITORING, recovery_attempts_};
    if (maximum_displacement_ < config_.minimum_actual_displacement)
      return {PathProgressAction::MONITORING, recovery_attempts_};

    return TriggerRecoveryOrFault();
  }

  void Pause()
  {
    window_initialized_ = false;
    no_motion_initialized_ = false;
  }

  void Reset()
  {
    segment_initialized_ = false;
    segment_id_ = 0;
    recovery_attempts_ = 0;
    fault_latched_ = false;
    window_initialized_ = false;
    best_progress_ = 0.0;
    window_start_progress_ = 0.0;
    maximum_displacement_ = 0.0;
    window_start_x_ = 0.0;
    window_start_y_ = 0.0;
    window_started_ = TimePoint{};
    no_motion_initialized_ = false;
    no_motion_start_x_ = 0.0;
    no_motion_start_y_ = 0.0;
    no_motion_started_ = TimePoint{};
  }

private:
  static double NonnegativeOrDefault(double value, double fallback)
  {
    return std::isfinite(value) ? std::max(0.0, value) : fallback;
  }

  PathProgressDecision TriggerRecoveryOrFault()
  {
    window_initialized_ = false;
    no_motion_initialized_ = false;
    if (recovery_attempts_ < config_.maximum_recovery_attempts)
    {
      ++recovery_attempts_;
      return {PathProgressAction::RECOVER, recovery_attempts_};
    }

    fault_latched_ = true;
    return {PathProgressAction::FAULT, recovery_attempts_};
  }

  PathProgressDecision UpdateNoMotionState(
    double current_x, double current_y, double commanded_linear_velocity,
    double measured_linear_velocity, bool measured_linear_velocity_valid,
    bool linear_motion_expected, TimePoint control_time)
  {
    const bool command_is_meaningful = linear_motion_expected &&
      std::isfinite(commanded_linear_velocity) &&
      std::abs(commanded_linear_velocity) >= config_.command_threshold &&
      std::abs(commanded_linear_velocity) > 1.0e-9 &&
      config_.no_motion_timeout > 0.0;
    if (!command_is_meaningful)
    {
      no_motion_initialized_ = false;
      return {};
    }

    if (!no_motion_initialized_ || control_time < no_motion_started_)
    {
      no_motion_start_x_ = current_x;
      no_motion_start_y_ = current_y;
      no_motion_started_ = control_time;
      no_motion_initialized_ = true;
      return {};
    }

    const double displacement = std::hypot(
      current_x - no_motion_start_x_, current_y - no_motion_start_y_);
    const bool position_moved = displacement > config_.no_motion_displacement;
    const bool measured_moving = measured_linear_velocity_valid &&
      std::isfinite(measured_linear_velocity) &&
      std::abs(measured_linear_velocity) > config_.stationary_speed_threshold;
    if (position_moved || measured_moving)
    {
      // Re-anchor after genuine motion so a later stop is measured on its own
      // dwell window rather than against an old command start.
      no_motion_start_x_ = current_x;
      no_motion_start_y_ = current_y;
      no_motion_started_ = control_time;
      return {};
    }

    const double stationary_seconds =
      std::chrono::duration<double>(control_time - no_motion_started_).count();
    if (stationary_seconds < config_.no_motion_timeout)
      return {};

    return TriggerRecoveryOrFault();
  }

  void BeginSegment(std::size_t segment_id)
  {
    segment_initialized_ = true;
    segment_id_ = segment_id;
    recovery_attempts_ = 0;
    fault_latched_ = false;
    window_initialized_ = false;
    no_motion_initialized_ = false;
  }

  PathProgressConfig config_;
  bool segment_initialized_{false};
  std::size_t segment_id_{0};
  int recovery_attempts_{0};
  bool fault_latched_{false};
  bool window_initialized_{false};
  double best_progress_{0.0};
  double window_start_progress_{0.0};
  double maximum_displacement_{0.0};
  double window_start_x_{0.0};
  double window_start_y_{0.0};
  TimePoint window_started_{};
  bool no_motion_initialized_{false};
  double no_motion_start_x_{0.0};
  double no_motion_start_y_{0.0};
  TimePoint no_motion_started_{};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__PATH_PROGRESS_SUPERVISOR_HPP_
