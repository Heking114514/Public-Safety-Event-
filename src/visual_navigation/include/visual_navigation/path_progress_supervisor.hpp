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
    config_.timeout = std::max(0.0, config_.timeout);
    config_.minimum_advance = std::max(0.0, config_.minimum_advance);
    config_.minimum_actual_displacement =
      std::max(0.0, config_.minimum_actual_displacement);
    config_.maximum_recovery_attempts =
      std::max(0, config_.maximum_recovery_attempts);
  }

  PathProgressDecision Evaluate(
    std::size_t segment_id, double measured_progress,
    double current_x, double current_y,
    bool monitoring_enabled, TimePoint control_time)
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
      return {PathProgressAction::MONITORING, recovery_attempts_};
    }

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

    window_initialized_ = false;
    if (recovery_attempts_ < config_.maximum_recovery_attempts)
    {
      ++recovery_attempts_;
      return {PathProgressAction::RECOVER, recovery_attempts_};
    }

    fault_latched_ = true;
    return {PathProgressAction::FAULT, recovery_attempts_};
  }

  void Pause()
  {
    window_initialized_ = false;
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
  }

private:
  void BeginSegment(std::size_t segment_id)
  {
    segment_initialized_ = true;
    segment_id_ = segment_id;
    recovery_attempts_ = 0;
    fault_latched_ = false;
    window_initialized_ = false;
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
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__PATH_PROGRESS_SUPERVISOR_HPP_
