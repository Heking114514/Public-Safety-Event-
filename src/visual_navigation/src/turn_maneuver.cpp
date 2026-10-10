#include "visual_navigation/turn_maneuver.hpp"

#include <algorithm>
#include <cmath>

namespace visual_navigation {
namespace {

bool FinitePose(const CornerShuntPose &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

CornerShuntPose Integrate(CornerShuntPose pose, CornerShuntTwist command,
                          double elapsed_s) {
  if (std::abs(command.angular_z) <= 1.0e-6) {
    pose.x += command.linear_x * elapsed_s * std::cos(pose.yaw);
    pose.y += command.linear_x * elapsed_s * std::sin(pose.yaw);
    return pose;
  }
  const double next_yaw = pose.yaw + command.angular_z * elapsed_s;
  const double radius = command.linear_x / command.angular_z;
  pose.x += radius * (std::sin(next_yaw) - std::sin(pose.yaw));
  pose.y -= radius * (std::cos(next_yaw) - std::cos(pose.yaw));
  pose.yaw = next_yaw;
  return pose;
}

CornerShuntTwist LimitedCommand(const CornerShuntConfig &config,
                                CornerShuntTwist current,
                                CornerShuntTwist target, double elapsed_s) {
  current.linear_x = LimitRate(target.linear_x, current.linear_x,
                               config.linear_acceleration,
                               config.linear_deceleration, elapsed_s);
  if (std::abs(target.linear_x) > 1.0e-9 &&
      std::abs(current.linear_x) < config.closed_loop_min_speed) {
    current.linear_x =
        std::copysign(config.closed_loop_min_speed, target.linear_x);
  } else if (std::abs(target.linear_x) <= 1.0e-9 &&
             std::abs(current.linear_x) < config.closed_loop_min_speed) {
    current.linear_x = 0.0;
  }
  current.angular_z = LimitRate(target.angular_z, current.angular_z,
                                config.angular_acceleration,
                                config.angular_deceleration, elapsed_s);
  return current;
}

} // namespace

const char *TurnManeuverStatus(TurnManeuverPhase phase) {
  switch (phase) {
  case TurnManeuverPhase::IDLE:
    return "TURN_IDLE";
  case TurnManeuverPhase::CAPTURE:
    return "TURN_CAPTURE";
  case TurnManeuverPhase::ENTRY_CHECK:
    return "TURN_ENTRY_CHECK";
  case TurnManeuverPhase::TRIM_SEARCH:
    return "TURN_TRIM_SEARCH";
  case TurnManeuverPhase::REVERSE_TRIM:
    return "TURN_REVERSE_TRIM";
  case TurnManeuverPhase::FORWARD_TRIM:
    return "TURN_FORWARD_TRIM";
  case TurnManeuverPhase::TRIM_SETTLE:
    return "TURN_TRIM_SETTLE";
  case TurnManeuverPhase::ARC:
    return "TURN_ARC";
  case TurnManeuverPhase::EXIT_ALIGN:
    return "TURN_EXIT_ALIGN";
  case TurnManeuverPhase::COMPLETE:
    return "PATH_ALIGNED";
  case TurnManeuverPhase::PAUSED:
    return "TURN_PAUSED";
  case TurnManeuverPhase::NEED_REPLAN:
    return "TURN_NEED_REPLAN";
  case TurnManeuverPhase::FAULT:
    return "FAULT_TURN_MANEUVER";
  }
  return "FAULT_TURN_MANEUVER";
}

TurnManeuverController::TurnManeuverController(TurnManeuverConfig config)
    : config_(config), arc_controller_(config.corner) {}

void TurnManeuverController::configure(const TurnManeuverConfig &config) {
  config_ = config;
  arc_controller_.configure(config_.corner);
  reset();
}

void TurnManeuverController::reset() {
  arc_controller_.reset();
  captured_ = {};
  phase_ = TurnManeuverPhase::IDLE;
  active_ = false;
  capture_valid_ = false;
  arc_started_ = false;
  paused_ = false;
  paused_at_ = 0.0;
  started_at_ = 0.0;
  phase_started_at_ = 0.0;
  trim_search_distance_ = 0.0;
  trim_target_x_ = 0.0;
  trim_direction_ = 0;
  trim_direction_started_at_ = 0.0;
  trim_direction_start_x_ = 0.0;
  trim_completion_samples_ = 0;
  trim_direction_switches_ = 0;
  trim_attempts_ = 0;
  trim_candidates_checked_ = 0;
  trim_footprint_rejections_ = 0;
  trim_turn_rejections_ = 0;
  has_safety_report_ = false;
  safety_report_ = {};
  progress_started_at_ = 0.0;
  progress_pose_ = {};
  progress_longitudinal_error_ = 0.0;
  progress_lateral_error_ = 0.0;
  progress_yaw_error_ = 0.0;
}

void TurnManeuverController::begin(double now_s) {
  reset();
  active_ = true;
  phase_ = TurnManeuverPhase::CAPTURE;
  started_at_ = now_s;
  phase_started_at_ = now_s;
}

bool TurnManeuverController::active() const { return active_; }

bool TurnManeuverController::enabled_for(double turn_error) const {
  return config_.corner.enabled && std::isfinite(turn_error) &&
         std::abs(turn_error) <= config_.corner.max_turn_angle;
}

std::size_t TurnManeuverController::trim_attempts() const {
  return trim_attempts_;
}

double TurnManeuverController::trim_target_x() const { return trim_target_x_; }

TurnManeuverDecision
TurnManeuverController::transition(TurnManeuverPhase phase) {
  phase_ = phase;
  return {phase_, {}, TurnManeuverResult::RUNNING};
}

TurnManeuverDecision
TurnManeuverController::terminal(TurnManeuverPhase phase,
                                 TurnManeuverResult result) {
  phase_ = phase;
  active_ = false;
  return {phase_, {}, result};
}

CornerShuntContext TurnManeuverController::live_context(
    const TurnManeuverInput &input) const {
  CornerShuntContext context = captured_;
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

CornerShuntPose TurnManeuverController::world_pose(double local_x,
                                                   double local_y,
                                                   double yaw) const {
  const double cosine = std::cos(captured_.geometry_heading);
  const double sine = std::sin(captured_.geometry_heading);
  return {captured_.junction_x + cosine * local_x - sine * local_y,
          captured_.junction_y + sine * local_x + cosine * local_y, yaw};
}

bool TurnManeuverController::context_valid(
    const CornerShuntContext &context) const {
  return FinitePose(context.pose) && std::isfinite(context.junction_x) &&
         std::isfinite(context.junction_y) &&
         std::isfinite(context.geometry_heading) &&
         std::isfinite(context.arrival_heading) &&
         std::isfinite(context.target_heading) &&
         enabled_for(NormalizeShuntAngle(context.target_heading -
                                         context.geometry_heading));
}

bool TurnManeuverController::direct_entry_allowed(
    const CornerShuntContext &context) const {
  const double yaw_error = std::abs(NormalizeShuntAngle(
      context.pose.yaw - context.geometry_heading));
  return arc_controller_.pose_is_safe(context, context.pose) &&
         (context.reverse_arrival ||
          yaw_error <= std::max(0.0, config_.direct_entry_yaw_tolerance)) &&
         direct_turn_feasible(context);
}

bool TurnManeuverController::direct_turn_feasible(
    const CornerShuntContext &start) const {
  CornerShuntController controller(config_.corner);
  CornerShuntContext context = start;
  context.applied_twist = {};
  context.measured_twist = {};
  context.measured_yaw_rate_valid = false;
  controller.begin(0.0);
  CornerShuntTwist applied;
  const double step = 1.0 / std::max(1.0, config_.corner.control_frequency);
  const int maximum_steps = static_cast<int>(std::ceil(
      std::max(1.0, config_.corner.max_turn_duration) / step));
  for (int index = 0; index < maximum_steps; ++index) {
    const auto decision = controller.update(context, index * step);
    if (decision.complete)
      return true;
    if (decision.blocked)
      return false;
    applied = LimitedCommand(config_.corner, applied, decision.command, step);
    context.applied_twist = applied;
    context.pose = Integrate(context.pose, applied, step);
  }
  return false;
}

bool TurnManeuverController::find_trim_target(
    const CornerShuntContext &context) {
  const auto local = local_pose(context.pose);
  const double step = std::max(0.005, config_.trim_search_step);
  double distance = std::max(step, trim_search_distance_ + step);
  distance = std::max(distance, local.x - config_.preferred_entry_x);
  const double maximum = std::max(step, config_.trim_max_distance);
  for (; distance <= maximum + 1.0e-9; distance += step) {
    ++trim_candidates_checked_;
    CornerShuntContext candidate = captured_;
    candidate.pose = world_pose(local.x - distance, 0.0,
                                captured_.geometry_heading);
    candidate.arrival_heading = captured_.geometry_heading;
    candidate.reverse_arrival = false;
    CornerShuntSafetyReport report;
    if (!arc_controller_.pose_is_safe(candidate, candidate.pose, &report)) {
      ++trim_footprint_rejections_;
      report.source = "trim_candidate_pose";
      safety_report_ = report;
      has_safety_report_ = true;
      continue;
    }
    if (!direct_turn_feasible(candidate)) {
      ++trim_turn_rejections_;
      continue;
    }
    trim_search_distance_ = distance;
    trim_target_x_ = local.x - distance;
    trim_direction_ = trim_target_x_ < local.x ? -1 : 1;
    trim_direction_started_at_ = phase_started_at_;
    trim_direction_start_x_ = local.x;
    trim_completion_samples_ = 0;
    ++trim_attempts_;
    reset_trim_progress(context, phase_started_at_);
    return true;
  }
  return false;
}

bool TurnManeuverController::trim_target_reached(
    const CornerShuntContext &context) const {
  const auto local = local_pose(context.pose);
  return std::abs(local.x - trim_target_x_) <=
             std::max(0.0, config_.trim_position_tolerance) &&
         std::abs(local.y) <=
             std::max(0.0, config_.trim_lateral_tolerance) &&
         std::abs(local.yaw) <= std::max(0.0, config_.trim_yaw_tolerance);
}

CornerShuntTwist TurnManeuverController::trim_command(
    const CornerShuntContext &context) const {
  const auto local = local_pose(context.pose);
  const double longitudinal_error = trim_target_x_ - local.x;
  const bool reverse = trim_direction_ < 0 ||
                       (trim_direction_ == 0 && longitudinal_error < 0.0);
  const double speed = std::max(0.005, config_.trim_speed);
  const double yaw_error = NormalizeShuntAngle(captured_.geometry_heading -
                                               context.pose.yaw);
  const double lateral_term = config_.trim_lateral_gain * local.y;
  double angular = config_.trim_heading_gain * yaw_error -
                   config_.trim_yaw_rate_damping *
                       context.measured_twist.angular_z;
  angular += reverse ? lateral_term : -lateral_term;
  const double limit = std::max(0.0, config_.trim_max_angular_speed);
  return {reverse ? -speed : speed,
          ClampShunt(angular, -limit, limit)};
}

bool TurnManeuverController::trim_direction_change_allowed(
    const CornerShuntContext &context, double now_s) const {
  if (trim_direction_ == 0)
    return true;
  const auto local = local_pose(context.pose);
  const double error = trim_target_x_ - local.x;
  const int desired_direction = error < 0.0 ? -1 : 1;
  if (desired_direction == trim_direction_)
    return false;
  const double hysteresis = std::max(0.0, config_.trim_reverse_hysteresis);
  const double position_tolerance =
      std::max(0.0, config_.trim_position_tolerance);
  if (std::abs(error) <= position_tolerance + hysteresis)
    return false;
  if (now_s - trim_direction_started_at_ <
      std::max(0.0, config_.trim_min_direction_hold))
    return false;
  return std::abs(local.x - trim_direction_start_x_) >=
         std::max(0.0, config_.trim_min_direction_distance);
}

bool TurnManeuverController::command_is_safe(
    const CornerShuntContext &context,
    const CornerShuntTwist &command,
    CornerShuntSafetyReport *report) const {
  CornerShuntSafetyReport current_report;
  const bool current_safe =
      arc_controller_.pose_is_safe(context, context.pose, &current_report);
  bool envelope_recovery = false;
  if (!current_safe) {
    const auto local = local_pose(context.pose);
    // A measured overshoot can leave the footprint outside the lane before
    // the controller gets a chance to react. Only the validated reverse trim
    // may escape that envelope; all other commands remain fail-closed.
    envelope_recovery =
        command.linear_x < -1.0e-9 &&
        current_report.failure ==
            CornerShuntSafetyFailure::FOOTPRINT_OUTSIDE_ENVELOPE &&
        trim_target_x_ < local.x;
    if (!envelope_recovery) {
      if (report != nullptr)
        *report = current_report;
      return false;
    }
  }
  auto pose = context.pose;
  auto applied = context.applied_twist;
  const double horizon = std::max(0.05, config_.corner.prediction_horizon);
  const double frequency = std::max(1.0, config_.corner.control_frequency);
  const int samples = std::max(1, static_cast<int>(std::ceil(horizon * frequency)));
  const double step = horizon / samples;
  for (int index = 0; index < samples; ++index) {
    applied = LimitedCommand(config_.corner, applied, command, step);
    pose = Integrate(pose, applied, step);
    CornerShuntSafetyReport sample_report;
    if (!arc_controller_.pose_is_safe(context, pose, &sample_report)) {
      if (envelope_recovery &&
          sample_report.failure ==
              CornerShuntSafetyFailure::FOOTPRINT_OUTSIDE_ENVELOPE)
        continue;
      if (report != nullptr)
        *report = sample_report;
      if (report != nullptr)
        report->prediction_sample = index;
      return false;
    }
  }
  return true;
}

void TurnManeuverController::attach_trim_diagnostics(
    TurnManeuverDecision *decision) const {
  decision->trim_attempts = trim_attempts_;
  decision->trim_candidates_checked = trim_candidates_checked_;
  decision->trim_footprint_rejections = trim_footprint_rejections_;
  decision->trim_turn_rejections = trim_turn_rejections_;
  decision->trim_direction = trim_direction_;
  decision->trim_direction_switches = trim_direction_switches_;
  decision->trim_target_local_x = trim_target_x_;
  decision->has_safety_report = has_safety_report_;
  decision->safety_report = safety_report_;
}

void TurnManeuverController::reset_trim_progress(
    const CornerShuntContext &context, double now_s) {
  const auto local = local_pose(context.pose);
  progress_started_at_ = now_s;
  progress_pose_ = context.pose;
  progress_longitudinal_error_ =
      std::abs(trim_target_x_ - local.x);
  progress_lateral_error_ = std::abs(local.y);
  progress_yaw_error_ = std::abs(local.yaw);
}

bool TurnManeuverController::trim_stalled(const CornerShuntContext &context,
                                          double now_s) {
  const auto local = local_pose(context.pose);
  const double longitudinal_error =
      std::abs(trim_target_x_ - local.x);
  const double lateral_error = std::abs(local.y);
  const double yaw_error = std::abs(local.yaw);
  const bool longitudinal_progress =
      progress_longitudinal_error_ - longitudinal_error >=
      std::max(0.0, config_.trim_progress_distance);
  const bool lateral_progress =
      progress_lateral_error_ - lateral_error >=
      std::max(0.0, config_.trim_progress_distance);
  const bool yaw_progress =
      progress_yaw_error_ - yaw_error >=
      std::max(0.0, config_.trim_progress_yaw);
  if (longitudinal_progress || lateral_progress || yaw_progress) {
    progress_longitudinal_error_ = longitudinal_error;
    progress_lateral_error_ = lateral_error;
    progress_yaw_error_ = yaw_error;
    progress_started_at_ = now_s;
    progress_pose_ = context.pose;
    return false;
  }
  return now_s - progress_started_at_ >=
         std::max(0.1, config_.trim_no_progress_timeout);
}

void TurnManeuverController::start_arc(const CornerShuntContext &context,
                                       double now_s) {
  captured_.arrival_heading = context.pose.yaw;
  arc_controller_.reset();
  arc_controller_.begin(now_s);
  arc_started_ = true;
  phase_ = TurnManeuverPhase::ARC;
  phase_started_at_ = now_s;
}

TurnManeuverDecision
TurnManeuverController::update(const TurnManeuverInput &input, double now_s) {
  if (!active_)
    return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
  if (input.cancel)
    return terminal(TurnManeuverPhase::IDLE, TurnManeuverResult::CANCELED);
  if (!input.motion_allowed || !input.localization_valid ||
      input.obstacle_blocked) {
    if (!paused_)
      paused_at_ = now_s;
    paused_ = true;
    arc_controller_.reset();
    arc_started_ = false;
    phase_ = TurnManeuverPhase::PAUSED;
    return {phase_, {}, TurnManeuverResult::RUNNING};
  }
  if (paused_) {
    started_at_ += std::max(0.0, now_s - paused_at_);
    paused_ = false;
    phase_ = TurnManeuverPhase::ENTRY_CHECK;
    phase_started_at_ = now_s;
  }
  if (now_s - started_at_ > config_.corner.max_turn_duration)
    return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);

  if (phase_ == TurnManeuverPhase::CAPTURE) {
    if (!context_valid(input.context))
      return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
    captured_ = input.context;
    capture_valid_ = true;
    phase_started_at_ = now_s;
    return transition(TurnManeuverPhase::ENTRY_CHECK);
  }
  if (!capture_valid_)
    return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);

  const auto context = live_context(input);
  if (phase_ == TurnManeuverPhase::ENTRY_CHECK) {
    if (direct_entry_allowed(context)) {
      start_arc(context, now_s);
      return {phase_, {}, TurnManeuverResult::RUNNING};
    }
    phase_started_at_ = now_s;
    return transition(TurnManeuverPhase::TRIM_SEARCH);
  }
  if (phase_ == TurnManeuverPhase::TRIM_SEARCH) {
    const bool limit_reached =
        trim_attempts_ >= static_cast<std::size_t>(
                             std::max(1, config_.max_trim_attempts));
    if (limit_reached || !find_trim_target(context)) {
      auto decision = terminal(TurnManeuverPhase::NEED_REPLAN,
                               TurnManeuverResult::NEED_REPLAN);
      decision.trim_limit_reached = limit_reached;
      attach_trim_diagnostics(&decision);
      return decision;
    }
    const auto local = local_pose(context.pose);
    phase_started_at_ = now_s;
    progress_started_at_ = now_s;
    progress_pose_ = context.pose;
    phase_ = trim_target_x_ < local.x ? TurnManeuverPhase::REVERSE_TRIM
                                      : TurnManeuverPhase::FORWARD_TRIM;
    TurnManeuverDecision decision{phase_, {}, TurnManeuverResult::RUNNING};
    decision.trim_target_selected = true;
    decision.trim_target_local_x = trim_target_x_;
    attach_trim_diagnostics(&decision);
    return decision;
  }
  if (phase_ == TurnManeuverPhase::REVERSE_TRIM ||
      phase_ == TurnManeuverPhase::FORWARD_TRIM) {
    if (trim_target_reached(context)) {
      ++trim_completion_samples_;
      if (trim_completion_samples_ >=
          std::max(1, config_.trim_completion_samples)) {
        phase_started_at_ = now_s;
        return transition(TurnManeuverPhase::TRIM_SETTLE);
      }
      TurnManeuverDecision decision{phase_, {},
                                    TurnManeuverResult::RUNNING};
      attach_trim_diagnostics(&decision);
      return decision;
    }
    trim_completion_samples_ = 0;
    if (trim_stalled(context, now_s)) {
      phase_started_at_ = now_s;
      auto decision = transition(TurnManeuverPhase::TRIM_SEARCH);
      attach_trim_diagnostics(&decision);
      return decision;
    }
    if (trim_direction_change_allowed(context, now_s)) {
      if (trim_direction_switches_ >= static_cast<std::size_t>(
                                      std::max(0, config_.max_trim_direction_switches))) {
        auto decision = terminal(TurnManeuverPhase::NEED_REPLAN,
                                 TurnManeuverResult::NEED_REPLAN);
        decision.trim_limit_reached = true;
        attach_trim_diagnostics(&decision);
        return decision;
      }
      const auto local = local_pose(context.pose);
      trim_direction_ = trim_target_x_ - local.x < 0.0 ? -1 : 1;
      ++trim_direction_switches_;
      trim_direction_started_at_ = now_s;
      trim_direction_start_x_ = local.x;
      reset_trim_progress(context, now_s);
      phase_ = trim_direction_ < 0 ? TurnManeuverPhase::REVERSE_TRIM
                                   : TurnManeuverPhase::FORWARD_TRIM;
      TurnManeuverDecision decision{phase_, {},
                                    TurnManeuverResult::RUNNING};
      attach_trim_diagnostics(&decision);
      return decision;
    }
    const auto command = trim_command(context);
    CornerShuntSafetyReport report;
    if (!command_is_safe(context, command, &report)) {
      phase_started_at_ = now_s;
      report.source = "trim_command";
      report.attempted_command = command;
      auto decision = transition(TurnManeuverPhase::TRIM_SEARCH);
      decision.has_safety_report = true;
      decision.safety_report = report;
      safety_report_ = report;
      has_safety_report_ = true;
      return decision;
    }
    phase_ = trim_direction_ < 0 ? TurnManeuverPhase::REVERSE_TRIM
                                 : TurnManeuverPhase::FORWARD_TRIM;
    TurnManeuverDecision decision{phase_, command,
                                  TurnManeuverResult::RUNNING};
    attach_trim_diagnostics(&decision);
    return decision;
  }
  if (phase_ == TurnManeuverPhase::TRIM_SETTLE) {
    if (now_s - phase_started_at_ < std::max(0.0, config_.trim_settle_duration))
      return {phase_, {}, TurnManeuverResult::RUNNING};
    captured_.arrival_heading = context.pose.yaw;
    phase_started_at_ = now_s;
    return transition(TurnManeuverPhase::ENTRY_CHECK);
  }
  if (phase_ == TurnManeuverPhase::ARC) {
    if (!arc_started_)
      start_arc(context, now_s);
    const auto arc = arc_controller_.update(context, now_s);
    if (arc.complete) {
      phase_started_at_ = now_s;
      return transition(TurnManeuverPhase::EXIT_ALIGN);
    }
    if (arc.blocked) {
      arc_controller_.reset();
      arc_started_ = false;
      phase_started_at_ = now_s;
      auto decision = transition(TurnManeuverPhase::TRIM_SEARCH);
      decision.has_safety_report =
          arc.safety_report.failure != CornerShuntSafetyFailure::NONE;
      decision.safety_report = arc.safety_report;
      safety_report_ = arc.safety_report;
      has_safety_report_ = decision.has_safety_report;
      return decision;
    }
    return {TurnManeuverPhase::ARC, arc.command,
            TurnManeuverResult::RUNNING};
  }
  if (phase_ == TurnManeuverPhase::EXIT_ALIGN)
    return terminal(TurnManeuverPhase::COMPLETE,
                    TurnManeuverResult::COMPLETED);
  if (phase_ == TurnManeuverPhase::NEED_REPLAN)
    return terminal(phase_, TurnManeuverResult::NEED_REPLAN);
  if (phase_ == TurnManeuverPhase::FAULT)
    return terminal(phase_, TurnManeuverResult::FAULT);
  return terminal(TurnManeuverPhase::FAULT, TurnManeuverResult::FAULT);
}

} // namespace visual_navigation
