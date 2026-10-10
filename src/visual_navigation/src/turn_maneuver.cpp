#include "visual_navigation/turn_maneuver.hpp"

#include <algorithm>
#include <cmath>

namespace visual_navigation {
namespace {
bool Finite(const CornerShuntPose &p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
double Clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }
}

const char *TurnManeuverStatus(TurnManeuverPhase phase) {
  switch (phase) {
    case TurnManeuverPhase::IDLE: return "TURN_IDLE";
    case TurnManeuverPhase::CAPTURE: return "TURN_CAPTURE";
    case TurnManeuverPhase::APPROACH_ENTRY: return "TURN_APPROACH_ENTRY";
    case TurnManeuverPhase::TURN_60: return "TURN_60";
    case TurnManeuverPhase::DIAGONAL_CREEP: return "TURN_DIAGONAL_CREEP";
    case TurnManeuverPhase::TURN_30: return "TURN_30";
    case TurnManeuverPhase::COMPLETE: return "PATH_ALIGNED";
    case TurnManeuverPhase::PAUSED: return "TURN_PAUSED";
    case TurnManeuverPhase::FAULT: return "FAULT_TURN_MANEUVER";
  }
  return "FAULT_TURN_MANEUVER";
}

TurnManeuverController::TurnManeuverController(TurnManeuverConfig config) : config_(config) {}
void TurnManeuverController::configure(const TurnManeuverConfig &config) { config_ = config; reset(); }
void TurnManeuverController::reset() {
  captured_ = {}; phase_ = TurnManeuverPhase::IDLE; active_ = false; capture_valid_ = false;
  phase_started_at_ = 0.0; creep_start_x_ = 0.0; creep_start_y_ = 0.0;
}
void TurnManeuverController::begin(double now_s) { reset(); active_ = true; phase_ = TurnManeuverPhase::CAPTURE; phase_started_at_ = now_s; }
bool TurnManeuverController::active() const { return active_; }
bool TurnManeuverController::enabled_for(double turn_error) const { return config_.enabled && std::isfinite(turn_error); }
TurnManeuverDecision TurnManeuverController::terminal(TurnManeuverPhase phase, TurnManeuverResult result) { phase_ = phase; active_ = false; return {phase_, {}, result}; }
CornerShuntContext TurnManeuverController::live_context(const TurnManeuverInput &input) const {
  auto context = captured_; context.pose = input.context.pose; context.applied_twist = input.context.applied_twist;
  context.measured_twist = input.context.measured_twist; context.measured_yaw_rate_valid = input.context.measured_yaw_rate_valid; return context;
}
CornerShuntPose TurnManeuverController::local_pose(const CornerShuntPose &pose) const {
  const double c = std::cos(captured_.geometry_heading), s = std::sin(captured_.geometry_heading);
  const double dx = pose.x - captured_.junction_x, dy = pose.y - captured_.junction_y;
  return {c * dx + s * dy, -s * dx + c * dy, NormalizeShuntAngle(pose.yaw - captured_.geometry_heading)};
}
bool TurnManeuverController::context_valid(const CornerShuntContext &context) const {
  return Finite(context.pose) && std::isfinite(context.junction_x) && std::isfinite(context.junction_y) &&
         std::isfinite(context.geometry_heading) && std::isfinite(context.target_heading) &&
         enabled_for(NormalizeShuntAngle(context.target_heading - context.geometry_heading));
}

TurnManeuverDecision TurnManeuverController::update(const TurnManeuverInput &input, double now_s) {
  if (!active_) return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
  if (input.cancel) return terminal(TurnManeuverPhase::IDLE, TurnManeuverResult::CANCELED);
  if (!input.motion_allowed || !input.localization_valid || input.obstacle_blocked) {
    phase_ = TurnManeuverPhase::PAUSED; return {phase_, {}, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::CAPTURE) {
    if (!context_valid(input.context)) return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
    captured_ = input.context; capture_valid_ = true; phase_ = TurnManeuverPhase::APPROACH_ENTRY; phase_started_at_ = now_s;
    return {phase_, {}, TurnManeuverResult::RUNNING};
  }
  if (!capture_valid_) return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
  const auto context = live_context(input); const auto local = local_pose(context.pose);
  constexpr double kPi = 3.14159265358979323846;
  const double direction = NormalizeShuntAngle(captured_.target_heading - captured_.geometry_heading) >= 0.0 ? 1.0 : -1.0;
  CornerShuntTwist cmd;
  if (phase_ == TurnManeuverPhase::PAUSED) { phase_ = TurnManeuverPhase::APPROACH_ENTRY; phase_started_at_ = now_s; return {phase_, {}, TurnManeuverResult::RUNNING}; }
  if (phase_ == TurnManeuverPhase::APPROACH_ENTRY) {
    const double x_err = config_.entry_target_x - local.x;
    if (std::abs(x_err) <= config_.entry_position_tolerance) { phase_ = TurnManeuverPhase::TURN_60; phase_started_at_ = now_s; return {phase_, {}, TurnManeuverResult::RUNNING}; }
    if (x_err < 0.0) { cmd.linear_x = -config_.entry_speed; cmd.angular_z = Clamp(1.2 * NormalizeShuntAngle(-local.yaw), -0.15, 0.15); }
    else { cmd.linear_x = config_.entry_speed; const double y_err = config_.entry_target_y * direction - local.y; cmd.angular_z = Clamp(1.2 * NormalizeShuntAngle(-local.yaw) + y_err, -0.15, 0.15); }
    return {phase_, cmd, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::TURN_60) {
    const double yaw_err = NormalizeShuntAngle(direction * kPi / 3.0 - local.yaw);
    if (std::abs(yaw_err) <= 0.06) { phase_ = TurnManeuverPhase::DIAGONAL_CREEP; creep_start_x_ = local.x; creep_start_y_ = local.y; phase_started_at_ = now_s; return {phase_, {}, TurnManeuverResult::RUNNING}; }
    cmd.linear_x = 0.010; cmd.angular_z = Clamp(1.2 * yaw_err, -config_.turn_angular_speed, config_.turn_angular_speed);
    if (std::abs(cmd.angular_z) < config_.turn_min_angular_speed) cmd.angular_z = std::copysign(config_.turn_min_angular_speed, yaw_err);
    return {phase_, cmd, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::DIAGONAL_CREEP) {
    if (std::hypot(local.x - creep_start_x_, local.y - creep_start_y_) >= 0.025) { phase_ = TurnManeuverPhase::TURN_30; phase_started_at_ = now_s; return {phase_, {}, TurnManeuverResult::RUNNING}; }
    cmd.linear_x = config_.creep_linear_speed; cmd.angular_z = Clamp(NormalizeShuntAngle(direction * kPi / 3.0 - local.yaw), -0.20, 0.20); return {phase_, cmd, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::TURN_30) {
    const double yaw_err = NormalizeShuntAngle(direction * kPi / 2.0 - local.yaw);
    if (std::abs(yaw_err) <= 0.05) return terminal(TurnManeuverPhase::COMPLETE, TurnManeuverResult::COMPLETED);
    cmd.linear_x = 0.010; cmd.angular_z = Clamp(1.2 * yaw_err, -config_.turn_angular_speed, config_.turn_angular_speed);
    if (std::abs(cmd.angular_z) < config_.turn_min_angular_speed) cmd.angular_z = std::copysign(config_.turn_min_angular_speed, yaw_err);
    return {phase_, cmd, TurnManeuverResult::RUNNING};
  }
  return terminal(TurnManeuverPhase::COMPLETE, TurnManeuverResult::COMPLETED);
}
}  // namespace visual_navigation
