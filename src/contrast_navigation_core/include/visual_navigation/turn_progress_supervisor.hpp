#ifndef VISUAL_NAVIGATION__TURN_PROGRESS_SUPERVISOR_HPP_
#define VISUAL_NAVIGATION__TURN_PROGRESS_SUPERVISOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace visual_navigation
{

enum class TurnProgressAction
{
  IDLE,
  MONITORING,
  RECOVER,
  FAULT
};

struct TurnProgressConfig
{
  double stall_timeout{6.0};
  double minimum_error_reduction{0.08};
  double base_total_timeout{12.0};
  double minimum_expected_yaw_rate{0.12};
  double maximum_total_timeout{40.0};
  int maximum_recovery_attempts{2};
};

struct TurnProgressDecision
{
  TurnProgressAction action{TurnProgressAction::IDLE};
  int recovery_attempt{0};
};

class TurnProgressSupervisor
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  TurnProgressSupervisor() = default;

  explicit TurnProgressSupervisor(const TurnProgressConfig &config)
  : config_(config)
  {
    config_.stall_timeout = std::max(0.0, config_.stall_timeout);
    config_.minimum_error_reduction = std::max(0.0, config_.minimum_error_reduction);
    config_.base_total_timeout = std::max(0.0, config_.base_total_timeout);
    config_.minimum_expected_yaw_rate =
      std::max(1.0e-3, config_.minimum_expected_yaw_rate);
    config_.maximum_total_timeout = std::max(
      config_.base_total_timeout, config_.maximum_total_timeout);
    config_.maximum_recovery_attempts =
      std::max(0, config_.maximum_recovery_attempts);
  }

  TurnProgressDecision Evaluate(
    std::size_t phase_id, double absolute_yaw_error,
    double expected_yaw_rate, bool monitoring_enabled, TimePoint control_time)
  {
    if (!monitoring_enabled || config_.stall_timeout <= 0.0 ||
      !std::isfinite(absolute_yaw_error) || !std::isfinite(expected_yaw_rate))
    {
      Pause();
      return {};
    }

    const double error = std::abs(absolute_yaw_error);
    if (!phase_initialized_ || phase_id != phase_id_)
      BeginPhase(phase_id);
    if (fault_latched_)
      return {TurnProgressAction::FAULT, recovery_attempts_};

    if (!window_initialized_)
    {
      const double useful_rate = std::max(
        config_.minimum_expected_yaw_rate, std::abs(expected_yaw_rate));
      allowed_total_seconds_ = std::min(
        config_.maximum_total_timeout,
        config_.base_total_timeout + error / useful_rate);
      best_error_ = error;
      window_start_error_ = error;
      attempt_started_ = control_time;
      progress_started_ = control_time;
      window_initialized_ = true;
      return {TurnProgressAction::MONITORING, recovery_attempts_};
    }

    best_error_ = std::min(best_error_, error);
    if (window_start_error_ - best_error_ >= config_.minimum_error_reduction)
    {
      window_start_error_ = best_error_;
      progress_started_ = control_time;
    }

    const double stalled_seconds =
      std::chrono::duration<double>(control_time - progress_started_).count();
    const double total_seconds =
      std::chrono::duration<double>(control_time - attempt_started_).count();
    if (stalled_seconds < config_.stall_timeout &&
      total_seconds < allowed_total_seconds_)
    {
      return {TurnProgressAction::MONITORING, recovery_attempts_};
    }

    window_initialized_ = false;
    if (recovery_attempts_ < config_.maximum_recovery_attempts)
    {
      ++recovery_attempts_;
      return {TurnProgressAction::RECOVER, recovery_attempts_};
    }

    fault_latched_ = true;
    return {TurnProgressAction::FAULT, recovery_attempts_};
  }

  void Reset()
  {
    phase_initialized_ = false;
    phase_id_ = 0;
    recovery_attempts_ = 0;
    fault_latched_ = false;
    ResetWindow();
  }

  void Pause()
  {
    ResetWindow();
  }

private:
  void BeginPhase(std::size_t phase_id)
  {
    phase_initialized_ = true;
    phase_id_ = phase_id;
    recovery_attempts_ = 0;
    fault_latched_ = false;
    ResetWindow();
  }

  void ResetWindow()
  {
    window_initialized_ = false;
    best_error_ = 0.0;
    window_start_error_ = 0.0;
    allowed_total_seconds_ = 0.0;
    attempt_started_ = TimePoint{};
    progress_started_ = TimePoint{};
  }

  TurnProgressConfig config_;
  bool phase_initialized_{false};
  std::size_t phase_id_{0};
  int recovery_attempts_{0};
  bool fault_latched_{false};
  bool window_initialized_{false};
  double best_error_{0.0};
  double window_start_error_{0.0};
  double allowed_total_seconds_{0.0};
  TimePoint attempt_started_{};
  TimePoint progress_started_{};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__TURN_PROGRESS_SUPERVISOR_HPP_
