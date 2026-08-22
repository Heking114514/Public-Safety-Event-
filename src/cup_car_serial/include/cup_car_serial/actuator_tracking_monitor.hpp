#ifndef CUP_CAR_SERIAL__ACTUATOR_TRACKING_MONITOR_HPP_
#define CUP_CAR_SERIAL__ACTUATOR_TRACKING_MONITOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace cup_car_serial
{

enum class WheelTrackingIssue
{
  NONE,
  NO_RESPONSE,
  WRONG_DIRECTION,
  SEVERE_ERROR
};

enum class ActuatorTrackingState
{
  STARTUP_GRACE,
  HEALTHY,
  SUSPECT,
  UNHEALTHY,
  RECOVERING,
  LATCHED
};

struct ActuatorTrackingConfig
{
  double command_deadband_mps{0.08};
  double response_floor_mps{0.03};
  double severe_absolute_error_mps{0.15};
  double severe_relative_error{0.80};
  double startup_grace_s{1.0};
  double fault_persistence_s{1.5};
  double recovery_stop_s{0.5};
  double retry_rearm_tracking_s{3.0};
  int maximum_recovery_attempts{2};
};

struct ActuatorTrackingDecision
{
  bool healthy{true};
  bool command_eligible{false};
  ActuatorTrackingState state{ActuatorTrackingState::HEALTHY};
  WheelTrackingIssue left_issue{WheelTrackingIssue::NONE};
  WheelTrackingIssue right_issue{WheelTrackingIssue::NONE};
  int recovery_attempts{0};
};

inline const char * ToString(WheelTrackingIssue issue)
{
  switch (issue)
  {
    case WheelTrackingIssue::NONE: return "OK";
    case WheelTrackingIssue::NO_RESPONSE: return "NO_RESPONSE";
    case WheelTrackingIssue::WRONG_DIRECTION: return "WRONG_DIRECTION";
    case WheelTrackingIssue::SEVERE_ERROR: return "SEVERE_ERROR";
  }
  return "UNKNOWN";
}

inline const char * ToString(ActuatorTrackingState state)
{
  switch (state)
  {
    case ActuatorTrackingState::STARTUP_GRACE: return "STARTUP_GRACE";
    case ActuatorTrackingState::HEALTHY: return "HEALTHY";
    case ActuatorTrackingState::SUSPECT: return "SUSPECT";
    case ActuatorTrackingState::UNHEALTHY: return "UNHEALTHY";
    case ActuatorTrackingState::RECOVERING: return "RECOVERING";
    case ActuatorTrackingState::LATCHED: return "LATCHED";
  }
  return "UNKNOWN";
}

inline std::string Describe(const ActuatorTrackingDecision &decision)
{
  std::string description = ToString(decision.state);
  if (decision.left_issue != WheelTrackingIssue::NONE ||
    decision.right_issue != WheelTrackingIssue::NONE)
  {
    description += ":LEFT_";
    description += ToString(decision.left_issue);
    description += "+RIGHT_";
    description += ToString(decision.right_issue);
  }
  return description;
}

class ActuatorTrackingMonitor
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  ActuatorTrackingMonitor() = default;

  explicit ActuatorTrackingMonitor(const ActuatorTrackingConfig &config)
  : config_(config)
  {
    config_.command_deadband_mps = std::max(0.0, config_.command_deadband_mps);
    config_.response_floor_mps = std::max(0.0, config_.response_floor_mps);
    config_.severe_absolute_error_mps =
      std::max(0.0, config_.severe_absolute_error_mps);
    config_.severe_relative_error = std::max(0.0, config_.severe_relative_error);
    config_.startup_grace_s = std::max(0.0, config_.startup_grace_s);
    config_.fault_persistence_s = std::max(0.0, config_.fault_persistence_s);
    config_.recovery_stop_s = std::max(0.0, config_.recovery_stop_s);
    config_.retry_rearm_tracking_s = std::max(0.0, config_.retry_rearm_tracking_s);
    config_.maximum_recovery_attempts =
      std::max(0, config_.maximum_recovery_attempts);
  }

  ActuatorTrackingDecision Update(
    double target_left_mps, double target_right_mps,
    double measured_left_mps, double measured_right_mps,
    TimePoint sample_time)
  {
    if (!initialized_)
    {
      initialized_ = true;
      startup_started_ = sample_time;
    }

    const bool targets_finite = std::isfinite(target_left_mps) &&
      std::isfinite(target_right_mps);
    const bool measurements_finite = std::isfinite(measured_left_mps) &&
      std::isfinite(measured_right_mps);
    const bool finite = targets_finite && measurements_finite;
    const bool left_eligible = targets_finite &&
      std::abs(target_left_mps) >= config_.command_deadband_mps;
    const bool right_eligible = targets_finite &&
      std::abs(target_right_mps) >= config_.command_deadband_mps;
    const bool command_eligible = left_eligible || right_eligible;
    const int left_command_direction = left_eligible ?
      (target_left_mps > 0.0 ? 1 : -1) : 0;
    const int right_command_direction = right_eligible ?
      (target_right_mps > 0.0 ? 1 : -1) : 0;
    if (!command_signature_initialized_ ||
      left_command_direction != left_command_direction_ ||
      right_command_direction != right_command_direction_)
    {
      command_signature_initialized_ = true;
      left_command_direction_ = left_command_direction;
      right_command_direction_ = right_command_direction;
      bad_started_initialized_ = false;
      healthy_tracking_started_initialized_ = false;
      active_left_issue_ = WheelTrackingIssue::NONE;
      active_right_issue_ = WheelTrackingIssue::NONE;
    }
    const WheelTrackingIssue left_issue = left_eligible && measurements_finite ?
      ClassifyWheel(target_left_mps, measured_left_mps) : WheelTrackingIssue::NONE;
    const WheelTrackingIssue right_issue = right_eligible && measurements_finite ?
      ClassifyWheel(target_right_mps, measured_right_mps) : WheelTrackingIssue::NONE;
    const bool bad = (command_eligible && !finite) ||
      left_issue != WheelTrackingIssue::NONE ||
      right_issue != WheelTrackingIssue::NONE;

    if (latched_)
      return Decision(false, command_eligible, ActuatorTrackingState::LATCHED);

    if (unhealthy_)
    {
      if (command_eligible || bad)
      {
        recovery_started_initialized_ = false;
        return Decision(false, command_eligible, ActuatorTrackingState::UNHEALTHY);
      }

      if (!recovery_started_initialized_)
      {
        recovery_started_ = sample_time;
        recovery_started_initialized_ = true;
      }
      const double stopped_seconds =
        std::chrono::duration<double>(sample_time - recovery_started_).count();
      if (stopped_seconds < config_.recovery_stop_s)
        return Decision(false, false, ActuatorTrackingState::RECOVERING);

      unhealthy_ = false;
      recovery_started_initialized_ = false;
      bad_started_initialized_ = false;
      healthy_tracking_started_initialized_ = false;
      active_left_issue_ = WheelTrackingIssue::NONE;
      active_right_issue_ = WheelTrackingIssue::NONE;
      return Decision(true, false, ActuatorTrackingState::HEALTHY);
    }

    if (!command_eligible)
    {
      bad_started_initialized_ = false;
      healthy_tracking_started_initialized_ = false;
      active_left_issue_ = WheelTrackingIssue::NONE;
      active_right_issue_ = WheelTrackingIssue::NONE;
      return Decision(true, false, ActuatorTrackingState::HEALTHY);
    }

    if (!bad)
    {
      bad_started_initialized_ = false;
      active_left_issue_ = WheelTrackingIssue::NONE;
      active_right_issue_ = WheelTrackingIssue::NONE;
      if (!healthy_tracking_started_initialized_)
      {
        healthy_tracking_started_ = sample_time;
        healthy_tracking_started_initialized_ = true;
      }
      const double healthy_seconds = std::chrono::duration<double>(
        sample_time - healthy_tracking_started_).count();
      if (healthy_seconds >= config_.retry_rearm_tracking_s)
        recovery_attempts_ = 0;
      return Decision(true, true, ActuatorTrackingState::HEALTHY);
    }

    healthy_tracking_started_initialized_ = false;
    active_left_issue_ = left_issue;
    active_right_issue_ = right_issue;
    if (!bad_started_initialized_)
    {
      bad_started_ = sample_time;
      bad_started_initialized_ = true;
    }
    const double startup_seconds =
      std::chrono::duration<double>(sample_time - startup_started_).count();
    const double bad_seconds =
      std::chrono::duration<double>(sample_time - bad_started_).count();
    if (startup_seconds < config_.startup_grace_s)
      return Decision(true, true, ActuatorTrackingState::STARTUP_GRACE);
    if (bad_seconds < config_.fault_persistence_s)
      return Decision(true, true, ActuatorTrackingState::SUSPECT);

    bad_started_initialized_ = false;
    if (recovery_attempts_ >= config_.maximum_recovery_attempts)
    {
      latched_ = true;
      return Decision(false, true, ActuatorTrackingState::LATCHED);
    }
    ++recovery_attempts_;
    unhealthy_ = true;
    recovery_started_initialized_ = false;
    return Decision(false, true, ActuatorTrackingState::UNHEALTHY);
  }

  void Pause()
  {
    bad_started_initialized_ = false;
    healthy_tracking_started_initialized_ = false;
    recovery_started_initialized_ = false;
  }

  void Reset()
  {
    initialized_ = false;
    unhealthy_ = false;
    latched_ = false;
    recovery_attempts_ = 0;
    command_signature_initialized_ = false;
    left_command_direction_ = 0;
    right_command_direction_ = 0;
    active_left_issue_ = WheelTrackingIssue::NONE;
    active_right_issue_ = WheelTrackingIssue::NONE;
    Pause();
  }

private:
  WheelTrackingIssue ClassifyWheel(double target_mps, double measured_mps) const
  {
    if (std::abs(measured_mps) < config_.response_floor_mps)
      return WheelTrackingIssue::NO_RESPONSE;
    if (target_mps * measured_mps < 0.0)
      return WheelTrackingIssue::WRONG_DIRECTION;
    const double severe_error = std::max(
      config_.severe_absolute_error_mps,
      config_.severe_relative_error * std::abs(target_mps));
    if (std::abs(measured_mps - target_mps) > severe_error)
      return WheelTrackingIssue::SEVERE_ERROR;
    return WheelTrackingIssue::NONE;
  }

  ActuatorTrackingDecision Decision(
    bool healthy, bool command_eligible, ActuatorTrackingState state) const
  {
    return {
      healthy, command_eligible, state, active_left_issue_, active_right_issue_,
      recovery_attempts_};
  }

  ActuatorTrackingConfig config_;
  bool initialized_{false};
  bool unhealthy_{false};
  bool latched_{false};
  int recovery_attempts_{0};
  bool command_signature_initialized_{false};
  int left_command_direction_{0};
  int right_command_direction_{0};
  WheelTrackingIssue active_left_issue_{WheelTrackingIssue::NONE};
  WheelTrackingIssue active_right_issue_{WheelTrackingIssue::NONE};
  bool bad_started_initialized_{false};
  bool healthy_tracking_started_initialized_{false};
  bool recovery_started_initialized_{false};
  TimePoint startup_started_{};
  TimePoint bad_started_{};
  TimePoint healthy_tracking_started_{};
  TimePoint recovery_started_{};
};

}  // namespace cup_car_serial

#endif  // CUP_CAR_SERIAL__ACTUATOR_TRACKING_MONITOR_HPP_
