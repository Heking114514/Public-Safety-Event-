#ifndef VISUAL_NAVIGATION__NAVIGATION_SUPERVISOR_HPP_
#define VISUAL_NAVIGATION__NAVIGATION_SUPERVISOR_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

#include "visual_navigation/actuator_health_policy.hpp"
#include "visual_navigation/fusion_health_policy.hpp"

namespace visual_navigation
{

struct NavigationInputStatus
{
  bool odometry_received{false};
  bool odometry_frame_valid{false};
  bool odometry_pose_valid{false};
  bool odometry_fresh{false};

  bool fusion_required{true};
  bool fusion_status_received{false};
  bool fusion_status_fresh{false};
  std::string fusion_status;
  FusionHealthDecision fusion_health;

  bool tracking_required{false};
  bool tracking_state_received{false};
  int tracking_state{-1};

  bool actuator_required{false};
  bool actuator_status_received{false};
  bool actuator_status_fresh{false};
  bool actuator_connected{false};
};

inline bool OdometryIsValid(const NavigationInputStatus &input)
{
  return input.odometry_received && input.odometry_frame_valid &&
    input.odometry_pose_valid && input.odometry_fresh;
}

inline bool TrackingStateIsValid(const NavigationInputStatus &input)
{
  return !input.tracking_required ||
    (input.tracking_state_received &&
    (input.tracking_state == 2 || input.tracking_state == 5));
}

inline FusionHealthDecision EffectiveFusionHealth(const NavigationInputStatus &input)
{
  if (!input.fusion_required)
    return FusionHealthDecision{true, false, 1.0};
  if (!input.fusion_status_received || !input.fusion_status_fresh)
    return FusionHealthDecision{};
  return input.fusion_health;
}

inline bool ActuatorHealthIsValid(const NavigationInputStatus &input)
{
  return ActuatorHealthIsValid(
    input.actuator_required, input.actuator_status_received,
    input.actuator_status_fresh, input.actuator_connected);
}

inline std::string LocalizationFailureState(
  const NavigationInputStatus &input, const FusionHealthDecision &health)
{
  if (!OdometryIsValid(input))
  {
    if (input.odometry_received && !input.odometry_frame_valid)
      return "FAULT_ODOMETRY_FRAME";
    if (input.odometry_received && !input.odometry_pose_valid)
      return "FAULT_ODOMETRY_INVALID";
    return "WAITING_FOR_ODOMETRY";
  }
  if (input.fusion_required &&
    (!input.fusion_status_received || !input.fusion_status_fresh))
  {
    return "FAULT_FUSION_STATUS_STALE";
  }
  if (health.fault)
    return "FAULT_FUSION_STATUS";
  if (input.fusion_required && !health.allowed)
    return "WAITING_FOR_ALLOWED_FUSION_STATUS";
  if (!TrackingStateIsValid(input))
    return "FAULT_TRACKING_LOST";
  return "WAITING_FOR_LOCALIZATION";
}

inline std::string ActuatorFailureState(const NavigationInputStatus &input)
{
  if (!input.actuator_status_received || !input.actuator_status_fresh)
    return "FAULT_ACTUATOR_STALE";
  return "FAULT_ACTUATOR_DISCONNECTED";
}

struct NavigationStartDecision
{
  bool ready{false};
  std::string failure_state;
};

inline NavigationStartDecision EvaluateNavigationStart(const NavigationInputStatus &input)
{
  const auto health = EffectiveFusionHealth(input);
  if (!LocalizationCanStart(
      OdometryIsValid(input), health, TrackingStateIsValid(input)))
  {
    return {false, LocalizationFailureState(input, health)};
  }
  if (!ActuatorHealthIsValid(input))
    return {false, ActuatorFailureState(input)};
  return {true, ""};
}

enum class NavigationRuntimeAction
{
  DRIVE,
  STOP_AND_WAIT,
  STOP_AND_LATCH
};

struct NavigationRuntimeDecision
{
  NavigationRuntimeAction action{NavigationRuntimeAction::STOP_AND_WAIT};
  FusionHealthDecision fusion_health;
  std::string state;
  bool reset_path_control{false};
};

struct NavigationSupervisorConfig
{
  double transient_localization_grace{2.0};
  double transient_fault_speed_scale{0.25};
  bool abort_on_tracking_loss{false};
};

class NavigationSupervisor
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  NavigationSupervisor() = default;

  explicit NavigationSupervisor(const NavigationSupervisorConfig &config)
  : config_(config)
  {
    config_.transient_localization_grace =
      std::max(0.0, config_.transient_localization_grace);
    config_.transient_fault_speed_scale = std::max(
      0.0, std::min(1.0, config_.transient_fault_speed_scale));
  }

  NavigationRuntimeDecision Evaluate(
    const NavigationInputStatus &input, TimePoint control_time)
  {
    if (!ActuatorHealthIsValid(input))
    {
      return {
        NavigationRuntimeAction::STOP_AND_WAIT, {},
        "WAITING_FOR_ACTUATOR_RECOVERY", false};
    }

    auto health = EffectiveFusionHealth(input);
    const bool tracking_valid = TrackingStateIsValid(input);
    const bool localization_valid =
      OdometryIsValid(input) && health.allowed && tracking_valid;
    if (localization_valid)
    {
      last_valid_localization_time_ = control_time;
      valid_localization_time_initialized_ = true;
    }

    const double seconds_since_valid = valid_localization_time_initialized_ ?
      std::chrono::duration<double>(
        control_time - last_valid_localization_time_).count() :
      std::numeric_limits<double>::infinity();
    const bool hard_fault = health.fault || input.fusion_status == "FAULT_STALLED";
    const bool bridge_transient_loss = !localization_valid &&
      CanBridgeTransientLocalizationLoss(
        localization_was_valid_, hard_fault, seconds_since_valid,
        config_.transient_localization_grace);

    if (bridge_transient_loss)
    {
      localization_was_valid_ = true;
      health.allowed = true;
      health.fault = false;
      health.speed_scale = config_.transient_fault_speed_scale;
      return {
        NavigationRuntimeAction::DRIVE, health,
        "DEGRADED_TRANSIENT_LOCALIZATION", false};
    }

    if (!localization_valid)
    {
      const bool latch = hard_fault ||
        (!tracking_valid && localization_was_valid_ && config_.abort_on_tracking_loss);
      return {
        latch ? NavigationRuntimeAction::STOP_AND_LATCH :
        NavigationRuntimeAction::STOP_AND_WAIT,
        health, LocalizationFailureState(input, health), true};
    }

    localization_was_valid_ = true;
    return {NavigationRuntimeAction::DRIVE, health, "", false};
  }

  void ResetLocalizationHistory()
  {
    localization_was_valid_ = false;
    valid_localization_time_initialized_ = false;
    last_valid_localization_time_ = TimePoint{};
  }

private:
  NavigationSupervisorConfig config_;
  bool localization_was_valid_{false};
  bool valid_localization_time_initialized_{false};
  TimePoint last_valid_localization_time_{};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__NAVIGATION_SUPERVISOR_HPP_
