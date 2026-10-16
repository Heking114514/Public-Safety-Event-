#include "visual_navigation/turn_maneuver.hpp"

#include <algorithm>
#include <cmath>

namespace visual_navigation {
namespace {
constexpr double kPi = 3.14159265358979323846;

bool Finite(const CornerShuntPose &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

bool ConfigValid(const TurnManeuverConfig &config) {
  const double values[] = {config.entry_target_x,
                           config.entry_target_y,
                           config.entry_speed,
                           config.turn_angular_speed,
                           config.turn_min_angular_speed,
                           config.creep_linear_speed,
                           config.turn_linear_speed,
                           config.entry_position_tolerance,
                           config.entry_yaw_tolerance,
                           config.phase_timeout,
                           config.no_progress_timeout,
                           config.min_position_progress,
                           config.min_yaw_progress,
                           config.creep_distance,
                           config.entry_heading_gain,
                           config.entry_max_angular_speed};
  for (double value : values) {
    if (!std::isfinite(value))
      return false;
  }
  return config.entry_speed > 0.0 && config.creep_linear_speed > 0.0 &&
         config.turn_linear_speed >= config.entry_speed &&
         config.turn_linear_speed >= config.creep_linear_speed &&
         config.turn_min_angular_speed > 0.0 &&
         config.turn_angular_speed >= config.turn_min_angular_speed &&
         config.entry_position_tolerance > 0.0 &&
         config.entry_yaw_tolerance > 0.0 && config.phase_timeout > 0.0 &&
         config.no_progress_timeout > 0.0 &&
         config.min_position_progress > 0.0 && config.min_yaw_progress > 0.0 &&
         config.creep_distance > 0.0 && config.entry_heading_gain > 0.0 &&
         config.entry_max_angular_speed > 0.0;
}

double TurnAngularCommand(double error, double angle,
                          const TurnManeuverConfig &config) {
  const double fraction = Clamp(std::abs(error) / angle, 0.0, 1.0);
  return std::copysign(
      config.turn_min_angular_speed +
          (config.turn_angular_speed - config.turn_min_angular_speed) *
              fraction,
      error);
}
} // namespace

const char *TurnManeuverStatus(TurnManeuverPhase phase) {
  switch (phase) {
  case TurnManeuverPhase::IDLE:
    return "TURN_IDLE";
  case TurnManeuverPhase::CAPTURE:
    return "TURN_CAPTURE";
  case TurnManeuverPhase::APPROACH_ENTRY:
    return "TURN_APPROACH_ENTRY";
  case TurnManeuverPhase::TURN_60:
    return "TURN_60";
  case TurnManeuverPhase::DIAGONAL_CREEP:
    return "TURN_DIAGONAL_CREEP";
  case TurnManeuverPhase::TURN_30:
    return "TURN_30";
  case TurnManeuverPhase::COMPLETE:
    return "PATH_ALIGNED";
  case TurnManeuverPhase::PAUSED:
    return "TURN_PAUSED";
  case TurnManeuverPhase::FAULT:
    return "FAULT_TURN_MANEUVER";
  }
  return "FAULT_TURN_MANEUVER";
}

TurnManeuverController::TurnManeuverController(TurnManeuverConfig config)
    : config_(config) {}

void TurnManeuverController::configure(const TurnManeuverConfig &config) {
  config_ = config;
  reset();
}

void TurnManeuverController::reset() {
  captured_ = {};
  phase_ = TurnManeuverPhase::IDLE;
  active_ = false;
  capture_valid_ = false;
  phase_started_at_ = 0.0;
  creep_start_x_ = 0.0;
  creep_start_y_ = 0.0;
  progress_pose_ = {};
  progress_started_at_ = 0.0;
  paused_phase_ = TurnManeuverPhase::IDLE;
}

void TurnManeuverController::begin(double now_s) {
  reset();
  active_ = true;
  phase_ = TurnManeuverPhase::CAPTURE;
  phase_started_at_ = now_s;
  progress_started_at_ = now_s;
}

bool TurnManeuverController::active() const { return active_; }

bool TurnManeuverController::enabled_for(double turn_error) const {
  return config_.enabled && std::isfinite(turn_error);
}

TurnManeuverDecision
TurnManeuverController::terminal(TurnManeuverPhase phase,
                                 TurnManeuverResult result) {
  phase_ = phase;
  active_ = false;
  return {phase_, {}, result};
}

CornerShuntContext
TurnManeuverController::live_context(const TurnManeuverInput &input) const {
  auto context = captured_;
  context.pose = input.context.pose;
  context.applied_twist = input.context.applied_twist;
  context.measured_twist = input.context.measured_twist;
  context.measured_yaw_rate_valid = input.context.measured_yaw_rate_valid;
  return context;
}

CornerShuntPose
TurnManeuverController::local_pose(const CornerShuntPose &pose) const {
  const double cosine = std::cos(captured_.geometry_heading);
  const double sine = std::sin(captured_.geometry_heading);
  const double dx = pose.x - captured_.junction_x;
  const double dy = pose.y - captured_.junction_y;
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy,
          NormalizeShuntAngle(pose.yaw - captured_.geometry_heading)};
}

bool TurnManeuverController::context_valid(
    const CornerShuntContext &context) const {
  return Finite(context.pose) && std::isfinite(context.junction_x) &&
         std::isfinite(context.junction_y) &&
         std::isfinite(context.geometry_heading) &&
         std::isfinite(context.target_heading) &&
         enabled_for(NormalizeShuntAngle(context.target_heading -
                                         context.geometry_heading));
}

TurnManeuverDecision
TurnManeuverController::update(const TurnManeuverInput &input, double now_s) {
  if (!active_ || !std::isfinite(now_s))
    return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
  if (input.cancel)
    return terminal(TurnManeuverPhase::IDLE, TurnManeuverResult::CANCELED);
  if (!input.motion_allowed || !input.localization_valid ||
      input.obstacle_blocked) {
    if (phase_ != TurnManeuverPhase::PAUSED) {
      paused_phase_ = phase_;
      phase_ = TurnManeuverPhase::PAUSED;
      progress_started_at_ = now_s;
    }
    return {phase_, {}, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::PAUSED) {
    phase_started_at_ += std::max(0.0, now_s - progress_started_at_);
    phase_ = paused_phase_;
    paused_phase_ = TurnManeuverPhase::IDLE;
    progress_pose_ = input.context.pose;
    progress_started_at_ = now_s;
    return {phase_, {}, TurnManeuverResult::RUNNING};
  }
  if (!ConfigValid(config_))
    return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
  if (phase_ == TurnManeuverPhase::CAPTURE) {
    if (!capture_valid_) {
      if (!context_valid(input.context))
        return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
      captured_ = input.context;
      capture_valid_ = true;
      progress_pose_ = input.context.pose;
      progress_started_at_ = now_s;
      return {phase_, {}, TurnManeuverResult::RUNNING};
    }
    phase_ = TurnManeuverPhase::APPROACH_ENTRY;
    phase_started_at_ = now_s;
    progress_pose_ = input.context.pose;
    progress_started_at_ = now_s;
    return {phase_, {}, TurnManeuverResult::RUNNING};
  }
  if (!capture_valid_ || !Finite(input.context.pose))
    return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);

  const auto context = live_context(input);
  const auto local = local_pose(context.pose);
  const double direction =
      NormalizeShuntAngle(captured_.target_heading -
                          captured_.geometry_heading) >= 0.0
          ? 1.0
          : -1.0;
  const auto enter_phase = [&](TurnManeuverPhase next) {
    phase_ = next;
    phase_started_at_ = now_s;
    progress_pose_ = context.pose;
    progress_started_at_ = now_s;
    return TurnManeuverDecision{phase_, {}, TurnManeuverResult::RUNNING};
  };
  const auto motion_failed = [&]() {
    if (now_s - phase_started_at_ >= config_.phase_timeout)
      return true;
    if (now_s - progress_started_at_ < config_.no_progress_timeout)
      return false;
    bool progress;
    if (phase_ == TurnManeuverPhase::TURN_60 ||
        phase_ == TurnManeuverPhase::TURN_30) {
      const double target =
          captured_.geometry_heading +
          direction *
              (phase_ == TurnManeuverPhase::TURN_60 ? kPi / 3.0 : kPi / 2.0);
      const double reduction =
          std::abs(NormalizeShuntAngle(target - progress_pose_.yaw)) -
          std::abs(NormalizeShuntAngle(target - context.pose.yaw));
      progress = reduction >= config_.min_yaw_progress;
    } else {
      const double heading =
          captured_.geometry_heading +
          (phase_ == TurnManeuverPhase::DIAGONAL_CREEP ? direction * kPi / 3.0
                                                       : 0.0);
      const double longitudinal =
          std::cos(heading) * (context.pose.x - progress_pose_.x) +
          std::sin(heading) * (context.pose.y - progress_pose_.y);
      const double drive_direction =
          phase_ == TurnManeuverPhase::DIAGONAL_CREEP
              ? 1.0
              : (config_.entry_target_x >= local_pose(captured_.pose).x ? 1.0
                                                                        : -1.0);
      progress =
          drive_direction * longitudinal >= config_.min_position_progress;
    }
    progress_pose_ = context.pose;
    progress_started_at_ = now_s;
    return !progress;
  };

  CornerShuntTwist command;
  if (phase_ == TurnManeuverPhase::APPROACH_ENTRY) {
    const double x_error = config_.entry_target_x - local.x;
    const double yaw_error = NormalizeShuntAngle(-local.yaw);
    // A fixed-direction capture avoids reversing every tick around the target.
    const double drive_direction =
        config_.entry_target_x >= local_pose(captured_.pose).x ? 1.0 : -1.0;
    // 仅按行驶方向的纵向距离验收入位，横向偏差不参与判定。
    if (drive_direction * x_error <= config_.entry_position_tolerance)
      return enter_phase(TurnManeuverPhase::TURN_60);
    if (motion_failed())
      return terminal(phase_, TurnManeuverResult::NEED_REPLAN);
    command.linear_x = drive_direction * config_.turn_linear_speed;
    command.angular_z = Clamp(
        config_.entry_heading_gain * yaw_error,
        -config_.entry_max_angular_speed, config_.entry_max_angular_speed);
    return {phase_, command, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::TURN_60) {
    const double yaw_error =
        NormalizeShuntAngle(direction * kPi / 3.0 - local.yaw);
    if (std::abs(yaw_error) <= config_.entry_yaw_tolerance) {
      creep_start_x_ = local.x;
      creep_start_y_ = local.y;
      return enter_phase(TurnManeuverPhase::DIAGONAL_CREEP);
    }
    if (motion_failed())
      return terminal(phase_, TurnManeuverResult::NEED_REPLAN);
    command.linear_x = config_.turn_linear_speed;
    command.angular_z = TurnAngularCommand(yaw_error, kPi / 3.0, config_);
    return {phase_, command, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::DIAGONAL_CREEP) {
    const double target = direction * kPi / 3.0;
    const double yaw_error = NormalizeShuntAngle(target - local.yaw);
    const double distance = std::cos(target) * (local.x - creep_start_x_) +
                            std::sin(target) * (local.y - creep_start_y_);
    if (distance >= config_.creep_distance &&
        std::abs(yaw_error) <= config_.entry_yaw_tolerance)
      return enter_phase(TurnManeuverPhase::TURN_30);
    if (motion_failed())
      return terminal(phase_, TurnManeuverResult::NEED_REPLAN);
    command.linear_x = config_.turn_linear_speed;
    command.angular_z =
        Clamp(config_.turn_min_angular_speed * yaw_error /
                  config_.entry_yaw_tolerance,
              -config_.turn_angular_speed, config_.turn_angular_speed);
    return {phase_, command, TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::TURN_30) {
    const double yaw_error =
        NormalizeShuntAngle(direction * kPi / 2.0 - local.yaw);
    if (std::abs(yaw_error) <= config_.entry_yaw_tolerance) {
      return terminal(TurnManeuverPhase::COMPLETE,
                      TurnManeuverResult::COMPLETED);
    }
    if (motion_failed())
      return terminal(phase_, TurnManeuverResult::NEED_REPLAN);
    command.linear_x = config_.turn_linear_speed;
    command.angular_z = TurnAngularCommand(yaw_error, kPi / 6.0, config_);
    return {phase_, command, TurnManeuverResult::RUNNING};
  }
  return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
}
} // namespace visual_navigation
