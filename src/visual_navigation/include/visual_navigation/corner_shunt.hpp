#ifndef VISUAL_NAVIGATION__CORNER_SHUNT_HPP_
#define VISUAL_NAVIGATION__CORNER_SHUNT_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <thread>
#include <vector>

#include "navigation_safety/checker.hpp"
#include "visual_navigation/rate_limiter.hpp"

namespace visual_navigation {

enum class CornerShuntPhase {
  IDLE,
  STOP,
  CHECK,
  ARC,
  ROTATE,
  CREEP,
  REVERSE,
  COMPLETE,
  BLOCKED
};

struct CornerShuntConfig {
  bool enabled{true};
  double road_width{0.20};
  double road_margin{0.002};
  double road_back{0.60};
  double road_forward{0.56};
  double vehicle_length{0.1320};
  double vehicle_width{0.1320};
  double base_offset_x{0.030};
  double base_offset_y{0.0};
  double stop_duration{0.18};
  double check_duration{0.05};
  double step_duration{0.20};
  double target_yaw_tolerance{0.08};
  double target_x{-0.0075};
  double target_y{0.1225};
  double target_x_tolerance{0.050};
  double target_y_tolerance{0.015};
  double max_turn_angle{2.20};
  double max_turn_duration{60.00};
  double prediction_horizon{0.20};
  double plan_linear_speed{0.020};
  double plan_reverse_speed{-0.020};
  double plan_angular_speed{0.24};
  double plan_clearance{0.001};
  double entry_base_x{-0.015};
  double closed_loop_min_speed{0.008};
  double closed_loop_max_angular_speed{0.40};
  double closed_loop_bearing_gain{1.40};
  double closed_loop_final_yaw_gain{0.60};
  double closed_loop_entry_heading_gain{1.40};
  double closed_loop_entry_lateral_gain{2.00};
  double closed_loop_yaw_rate_damping{0.35};
  double closed_loop_yaw_rate_filter_s{0.08};
  double closed_loop_exit_max_angular_speed{0.12};
  double no_progress_timeout{3.0};
  double no_progress_distance{0.003};
  double no_progress_yaw{0.03};
  double control_frequency{30.0};
  double linear_acceleration{0.25};
  double linear_deceleration{0.80};
  double angular_acceleration{1.80};
  double angular_deceleration{3.00};
  int max_plan_count{8};
};

struct CornerShuntPose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CornerShuntTwist {
  double linear_x{0.0};
  double angular_z{0.0};
};

enum class CornerShuntSafetyFailure {
  NONE,
  FOOTPRINT_OUTSIDE_ENVELOPE,
  FOOTPRINT_INTERSECTS_INNER_CORNER
};

struct CornerShuntSafetyReport {
  CornerShuntSafetyFailure failure{CornerShuntSafetyFailure::NONE};
  const char *source{"pose_check"};
  int prediction_sample{-1};
  int footprint_corner{-1};
  CornerShuntPose checked_pose;
  CornerShuntPose corner_local;
  CornerShuntTwist attempted_command;
  double overlap_m{0.0};
};

inline const char *CornerShuntSafetyFailureName(
    CornerShuntSafetyFailure failure) {
  switch (failure) {
  case CornerShuntSafetyFailure::FOOTPRINT_OUTSIDE_ENVELOPE:
    return "footprint_outside_envelope";
  case CornerShuntSafetyFailure::FOOTPRINT_INTERSECTS_INNER_CORNER:
    return "footprint_intersects_inner_corner";
  case CornerShuntSafetyFailure::NONE:
    return "none";
  }
  return "unknown";
}

struct CornerShuntContext {
  CornerShuntPose pose;
  CornerShuntTwist applied_twist;
  CornerShuntTwist measured_twist;
  bool measured_yaw_rate_valid{false};
  double junction_x{0.0};
  double junction_y{0.0};
  // Road-frame heading used for junction coordinates and footprint checks.
  double geometry_heading{0.0};
  // Vehicle heading at the completed brake stop, used for residual turn error.
  double arrival_heading{0.0};
  double target_heading{0.0};
  bool reverse_arrival{false};
};

struct CornerShuntDecision {
  CornerShuntPhase phase{CornerShuntPhase::IDLE};
  CornerShuntTwist command;
  bool complete{false};
  bool blocked{false};
  CornerShuntSafetyReport safety_report;

  CornerShuntDecision() = default;
  CornerShuntDecision(CornerShuntPhase phase_value,
                      CornerShuntTwist command_value, bool complete_value,
                      bool blocked_value,
                      CornerShuntSafetyReport report_value = {})
      : phase(phase_value), command(command_value), complete(complete_value),
        blocked(blocked_value), safety_report(report_value) {}
};

inline const char *CornerShuntStatus(CornerShuntPhase phase) {
  switch (phase) {
  case CornerShuntPhase::STOP:
    return "TURN_STOP";
  case CornerShuntPhase::CHECK:
    return "TURN_CHECK";
  case CornerShuntPhase::ARC:
    return "TURN_ARC";
  case CornerShuntPhase::ROTATE:
    return "TURN_ROTATE";
  case CornerShuntPhase::CREEP:
    return "TURN_CREEP";
  case CornerShuntPhase::REVERSE:
    return "TURN_REVERSE";
  case CornerShuntPhase::COMPLETE:
    return "PATH_ALIGNED";
  case CornerShuntPhase::BLOCKED:
    return "FAULT_TURN_BLOCKED";
  case CornerShuntPhase::IDLE:
    break;
  }
  return "TURN_CHECK";
}

inline double NormalizeShuntAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double ClampShunt(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

class CornerShuntController {
public:
  explicit CornerShuntController(CornerShuntConfig config = {})
      : config_(config) {}

  void configure(const CornerShuntConfig &config) { config_ = config; }

  void reset() {
    active_ = false;
    phase_ = CornerShuntPhase::IDLE;
    started_at_ = 0.0;
    phase_started_at_ = 0.0;
    planned_commands_.clear();
    planned_command_index_ = 0;
    plan_count_ = 0;
    plan_future_ = {};
    command_cycles_remaining_ = 0;
    last_command_ = {};
    progress_initialized_ = false;
    progress_started_at_ = 0.0;
    progress_pose_ = {};
    yaw_rate_feedback_initialized_ = false;
    filtered_yaw_rate_ = 0.0;
    last_yaw_rate_feedback_at_ = 0.0;
    corner_entry_committed_ = false;
  }

  void begin(double now_s) {
    active_ = true;
    phase_ = CornerShuntPhase::STOP;
    started_at_ = now_s;
    phase_started_at_ = now_s;
    planned_commands_.clear();
    planned_command_index_ = 0;
    plan_count_ = 0;
    plan_future_ = {};
    command_cycles_remaining_ = 0;
    last_command_ = {};
    progress_initialized_ = false;
    progress_started_at_ = now_s;
    progress_pose_ = {};
    yaw_rate_feedback_initialized_ = false;
    filtered_yaw_rate_ = 0.0;
    last_yaw_rate_feedback_at_ = now_s;
    corner_entry_committed_ = false;
  }

  bool active() const { return active_; }

  std::size_t planned_command_count() const { return planned_commands_.size(); }

  int plan_count() const { return plan_count_; }

  bool enabled_for(double turn_error) const {
    return config_.enabled && std::isfinite(turn_error) &&
           std::abs(turn_error) <= std::max(0.0, config_.max_turn_angle);
  }

  CornerShuntDecision update(const CornerShuntContext &context, double now_s) {
    if (!active_)
      begin(now_s);
    update_yaw_rate_feedback(context, now_s);
    if (now_s - started_at_ > std::max(0.0, config_.max_turn_duration))
      return block();
    if (motion_stalled(context, now_s)) {
      planned_commands_.clear();
      planned_command_index_ = 0;
      command_cycles_remaining_ = 0;
      last_command_ = {};
      progress_started_at_ = now_s;
      progress_pose_ = context.pose;
      if (!context.reverse_arrival) {
        ++plan_count_;
        if (plan_count_ >= std::max(1, config_.max_plan_count))
          return block();
      } else if (plan_count_ >= std::max(1, config_.max_plan_count)) {
        return block();
      }
      return set_phase(CornerShuntPhase::CHECK, now_s, {});
    }
    if (target_reached(context)) {
      active_ = false;
      phase_ = CornerShuntPhase::COMPLETE;
      return {phase_, {}, true, false};
    }

    const double phase_elapsed = std::max(0.0, now_s - phase_started_at_);
    if (phase_ == CornerShuntPhase::STOP) {
      if (phase_elapsed < std::max(0.0, config_.stop_duration))
        return {phase_, {}, false, false};
      return set_phase(CornerShuntPhase::CHECK, now_s, {});
    }
    if (phase_ == CornerShuntPhase::CHECK) {
      if (phase_elapsed < std::max(0.0, config_.check_duration))
        return {phase_, {}, false, false};
      if (!context.reverse_arrival) {
        if (!pose_is_safe(context, context.pose))
          return block();
        return closed_loop_action(context, now_s);
      }
      return choose_action(context, now_s);
    }
    if (phase_ == CornerShuntPhase::BLOCKED)
      return block();

    if (!context.reverse_arrival)
      return closed_loop_action(context, now_s);

    if (command_cycles_remaining_ > 1 &&
        command_is_safe(context, last_command_,
                        1.0 / std::max(1.0, config_.control_frequency))) {
      --command_cycles_remaining_;
      return {phase_, last_command_, false, false};
    }
    return choose_action(context, now_s);
  }

  bool pose_is_safe(const CornerShuntContext &context,
                    const CornerShuntPose &pose,
                    CornerShuntSafetyReport *report = nullptr) const {
    const double cos_heading = std::cos(context.geometry_heading);
    const double sin_heading = std::sin(context.geometry_heading);
    const double direction = turn_sign(context);
    const double half = std::max(0.0, 0.5 * config_.road_width);
    const double margin = ClampShunt(config_.road_margin, 0.0, half - 1.0e-6);
    const double safe_half = half - margin;
    const double back = std::max(0.0, config_.road_back) - margin;
    const double forward = std::max(0.0, config_.road_forward) - margin;
    const double local_x =
        cos_heading * (pose.x - context.junction_x) +
        sin_heading * (pose.y - context.junction_y);
    const double local_y =
        (-sin_heading * (pose.x - context.junction_x) +
         cos_heading * (pose.y - context.junction_y)) *
        direction;
    const double local_yaw =
        NormalizeShuntAngle(pose.yaw - context.geometry_heading) * direction;
    const navigation_safety::Pose local_pose{{local_x, local_y}, local_yaw};
    const navigation_safety::Footprint footprint{
        config_.vehicle_length, config_.vehicle_width, config_.base_offset_x,
        direction * config_.base_offset_y, 0.0};
    const std::vector<navigation_safety::Rectangle> free_regions{{
        -back, -safe_half, safe_half, forward}};
    const navigation_safety::Rectangle inner_corner{
        -back, safe_half, -safe_half, forward};
    const std::vector<navigation_safety::Polygon> blocked_regions{
        navigation_safety::rectangle_polygon(inner_corner)};
    navigation_safety::Report safety_report;
    const bool safe = navigation_safety::pose_is_safe(
        local_pose, footprint, free_regions, blocked_regions, {},
        &safety_report);
    if (safe) {
      return true;
    }
    if (report != nullptr) {
      report->failure = safety_report.failure ==
                                navigation_safety::Failure::OUTSIDE_FREE_SPACE
                            ? CornerShuntSafetyFailure::FOOTPRINT_OUTSIDE_ENVELOPE
                            : CornerShuntSafetyFailure::
                                  FOOTPRINT_INTERSECTS_INNER_CORNER;
      report->checked_pose = pose;
      report->footprint_corner = safety_report.footprint_corner;
      report->overlap_m = safety_report.overlap;
      const auto corners = navigation_safety::footprint_corners(
          local_pose, footprint);
      if (report->footprint_corner >= 0 &&
          static_cast<std::size_t>(report->footprint_corner) < corners.size()) {
        const auto &corner =
            corners[static_cast<std::size_t>(report->footprint_corner)];
        report->corner_local = {corner.x, corner.y, 0.0};
      }
    }
    return safe;
  }

private:
  enum class PlanAvailability { PENDING, READY, FAILED };

  CornerShuntDecision choose_action(const CornerShuntContext &context,
                                    double now_s) {
    if (planned_command_index_ < planned_commands_.size()) {
      const auto command = planned_commands_[planned_command_index_];
      if (!command_is_safe(context, command)) {
        planned_commands_.clear();
        planned_command_index_ = 0;
        return set_phase(CornerShuntPhase::CHECK, now_s, {});
      }

      ++planned_command_index_;
      return set_phase(command_phase(command), now_s, command);
    }

    // Stop before planning from a residual motion state. The next CHECK cycle
    // sees the hard-zero command published by RunCornerShunt and plans from it.
    if (phase_ != CornerShuntPhase::CHECK)
      return set_phase(CornerShuntPhase::CHECK, now_s, {});

    const auto availability = ensure_plan(context);
    if (availability == PlanAvailability::PENDING)
      return set_phase(CornerShuntPhase::CHECK, now_s, {});
    if (availability == PlanAvailability::FAILED)
      return block();

    const auto command = planned_commands_[planned_command_index_];
    ++planned_command_index_;
    return set_phase(command_phase(command), now_s, command);
  }

  CornerShuntDecision closed_loop_action(const CornerShuntContext &context,
                                         double now_s) {
    const auto local = local_turn_pose(context, context.pose);
    const double direction = turn_sign(context);
    const double base_x =
        local.x + config_.base_offset_x * std::cos(local.yaw) -
        direction * config_.base_offset_y * std::sin(local.yaw);
    const double forward_speed = std::max(0.005, config_.plan_linear_speed);
    const double minimum_speed =
        ClampShunt(config_.closed_loop_min_speed, 0.005, forward_speed);
    const double yaw_rate_local = filtered_yaw_rate_ * direction;
    if (base_x >= config_.entry_base_x)
      corner_entry_committed_ = true;

    double omega_local = 0.0;
    double steering_error = local.yaw;
    bool aligning_exit = false;
    if (!corner_entry_committed_) {
      omega_local = -config_.closed_loop_entry_heading_gain * local.yaw -
                    config_.closed_loop_entry_lateral_gain * local.y -
                    config_.closed_loop_yaw_rate_damping * yaw_rate_local;
    } else {
      const double target_scale =
          ClampShunt(std::abs(turn_error(context)) / (0.5 * 3.14159265358979323846),
                     0.45, 1.40);
      const double target_x = config_.target_x * target_scale;
      const double target_y = config_.target_y * target_scale;
      // Preserve the measured arrival residual while evaluating progress from
      // the actual current pose: target-arrival minus arrival-to-current.
      const double arrival_turn_error = std::abs(NormalizeShuntAngle(
          context.target_heading - context.arrival_heading));
      const double arrival_progress =
          NormalizeShuntAngle(context.pose.yaw - context.arrival_heading) *
          turn_sign(context);
      const double final_yaw_error = NormalizeShuntAngle(
          arrival_turn_error - arrival_progress);
      aligning_exit =
          local.y >= target_y - std::max(0.03, config_.target_y_tolerance);
      if (aligning_exit) {
        steering_error = final_yaw_error;
        omega_local =
            config_.closed_loop_bearing_gain * final_yaw_error +
            config_.closed_loop_entry_lateral_gain * (local.x - target_x) -
            config_.closed_loop_yaw_rate_damping * yaw_rate_local;
      } else {
        const double bearing =
            std::atan2(target_y - local.y, target_x - local.x);
        const double bearing_error =
            NormalizeShuntAngle(bearing - local.yaw);
        steering_error = bearing_error;
        omega_local = config_.closed_loop_bearing_gain * bearing_error +
                      config_.closed_loop_final_yaw_gain * final_yaw_error -
                      config_.closed_loop_yaw_rate_damping * yaw_rate_local;
      }
    }

    const double maximum_omega =
        std::max(0.05, config_.closed_loop_max_angular_speed);
    if (corner_entry_committed_ && !aligning_exit) {
      // Once the axle has entered the junction, never return to the entry
      // controller or counter-steer toward the incoming road.
      omega_local = ClampShunt(omega_local, 0.0, maximum_omega);
    } else if (aligning_exit) {
      const double exit_limit = std::min(
        maximum_omega,
        std::max(0.0, config_.closed_loop_exit_max_angular_speed));
      omega_local = ClampShunt(omega_local, -exit_limit, exit_limit);
    } else {
      omega_local = ClampShunt(omega_local, -maximum_omega, maximum_omega);
    }
    double heading_scale = ClampShunt(
        std::cos(std::min(std::abs(steering_error),
                          0.5 * 3.14159265358979323846)),
        minimum_speed / forward_speed, 1.0);
    if (aligning_exit && std::abs(steering_error) > 0.20)
      heading_scale = minimum_speed / forward_speed;
    const CornerShuntTwist command{forward_speed * heading_scale,
                                   direction * omega_local};
    CornerShuntSafetyReport report;
    if (!command_is_safe(context, command, -1.0, &report)) {
      report.source = "corner_arc_command";
      report.attempted_command = command;
      return block(report);
    }
    return set_phase(command_phase(command), now_s, command);
  }

  PlanAvailability ensure_plan(const CornerShuntContext &context) {
    if (planned_command_index_ < planned_commands_.size())
      return PlanAvailability::READY;

    if (plan_future_.valid()) {
      if (plan_future_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready)
        return PlanAvailability::PENDING;
      planned_commands_ = plan_future_.get();
      planned_command_index_ = 0;
      return planned_commands_.empty() ? PlanAvailability::FAILED
                                       : PlanAvailability::READY;
    }

    if (plan_count_ >= std::max(1, config_.max_plan_count))
      return PlanAvailability::FAILED;
    ++plan_count_;
    const auto config = config_;
    std::packaged_task<std::vector<CornerShuntTwist>()> task(
        [config, context]() {
          CornerShuntController planner(config);
          return planner.compute_plan(context);
        });
    plan_future_ = task.get_future();
    std::thread(std::move(task)).detach();
    return PlanAvailability::PENDING;
  }

  std::vector<CornerShuntTwist>
  compute_plan(const CornerShuntContext &context) const;

  CornerShuntPhase command_phase(const CornerShuntTwist &command) const {
    if (command.linear_x < -1.0e-9)
      return CornerShuntPhase::REVERSE;
    if (std::abs(command.linear_x) <= 1.0e-9)
      return CornerShuntPhase::ROTATE;
    if (std::abs(command.angular_z) <= 1.0e-9)
      return CornerShuntPhase::CREEP;
    return CornerShuntPhase::ARC;
  }

  bool motion_stalled(const CornerShuntContext &context, double now_s) {
    if (!progress_initialized_) {
      progress_initialized_ = true;
      progress_started_at_ = now_s;
      progress_pose_ = context.pose;
      return false;
    }
    const double distance =
        std::hypot(context.pose.x - progress_pose_.x,
                   context.pose.y - progress_pose_.y);
    const double yaw = std::abs(
        NormalizeShuntAngle(context.pose.yaw - progress_pose_.yaw));
    if (distance >= std::max(0.0, config_.no_progress_distance) ||
        yaw >= std::max(0.0, config_.no_progress_yaw)) {
      progress_started_at_ = now_s;
      progress_pose_ = context.pose;
      return false;
    }
    const bool commanding_motion =
        phase_ == CornerShuntPhase::ARC ||
        phase_ == CornerShuntPhase::ROTATE ||
        phase_ == CornerShuntPhase::CREEP ||
        phase_ == CornerShuntPhase::REVERSE;
    return commanding_motion && config_.no_progress_timeout > 0.0 &&
           now_s - progress_started_at_ >= config_.no_progress_timeout;
  }

  CornerShuntDecision set_phase(CornerShuntPhase phase, double now_s,
                                CornerShuntTwist command) {
    phase_ = phase;
    phase_started_at_ = now_s;
    last_command_ = command;
    const bool motion_phase =
        phase == CornerShuntPhase::ARC || phase == CornerShuntPhase::ROTATE ||
        phase == CornerShuntPhase::CREEP || phase == CornerShuntPhase::REVERSE;
    command_cycles_remaining_ =
        motion_phase
            ? std::max(1, static_cast<int>(std::llround(
                              std::max(0.01, config_.step_duration) *
                              std::max(1.0, config_.control_frequency))))
            : 0;
    return {phase_, command, false, false};
  }

  CornerShuntDecision block(
      const CornerShuntSafetyReport &report = CornerShuntSafetyReport{}) {
    phase_ = CornerShuntPhase::BLOCKED;
    last_command_ = {};
    return {phase_, {}, false, true, report};
  }

  CornerShuntPose local_turn_pose(const CornerShuntContext &context,
                                  CornerShuntPose pose) const {
    const double cos_heading = std::cos(context.geometry_heading);
    const double sin_heading = std::sin(context.geometry_heading);
    const double dx = pose.x - context.junction_x;
    const double dy = pose.y - context.junction_y;
    const double sign = turn_sign(context);
    return {cos_heading * dx + sin_heading * dy,
            (-sin_heading * dx + cos_heading * dy) * sign,
            NormalizeShuntAngle(pose.yaw - context.geometry_heading) * sign};
  }

  bool command_is_safe(const CornerShuntContext &context,
                       CornerShuntTwist command,
                       double horizon_s = -1.0,
                       CornerShuntSafetyReport *report = nullptr) const {
    if (std::abs(command.linear_x) <= 1.0e-9 &&
        std::abs(command.angular_z) <= 1.0e-9)
      return true;
    if (!pose_is_safe(context, context.pose, report))
      return false;

    const double horizon = horizon_s >= 0.0
                               ? horizon_s
                               : std::max(0.0, config_.prediction_horizon);
    const int samples =
        std::max(1, static_cast<int>(std::llround(
                        horizon * std::max(1.0, config_.control_frequency))));
    auto pose = context.pose;
    auto applied = context.applied_twist;
    for (int index = 0; index < samples; ++index) {
      const double elapsed = horizon / static_cast<double>(samples);
      applied = limited_command(applied, command, elapsed);
      pose = integrate(pose, applied, elapsed);
      if (!pose_is_safe(context, pose, report)) {
        if (report != nullptr)
          report->prediction_sample = index;
        return false;
      }
    }
    return true;
  }

  CornerShuntPose integrate(CornerShuntPose pose, CornerShuntTwist command,
                            double elapsed_s) const {
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

  CornerShuntTwist limited_command(CornerShuntTwist current,
                                   CornerShuntTwist target,
                                   double elapsed_s) const {
    current.linear_x =
        LimitRate(target.linear_x, current.linear_x,
                  config_.linear_acceleration, config_.linear_deceleration,
                  elapsed_s);
    if (std::abs(target.linear_x) > 1.0e-9 &&
        std::abs(current.linear_x) < config_.closed_loop_min_speed) {
      current.linear_x =
          std::copysign(config_.closed_loop_min_speed, target.linear_x);
    } else if (std::abs(target.linear_x) <= 1.0e-9 &&
               std::abs(current.linear_x) < config_.closed_loop_min_speed) {
      current.linear_x = 0.0;
    }
    current.angular_z =
        LimitRate(target.angular_z, current.angular_z,
                  config_.angular_acceleration, config_.angular_deceleration,
                  elapsed_s);
    return current;
  }

  bool target_reached(const CornerShuntContext &context) const {
    return target_pose_reached(context, context.pose);
  }

  bool target_pose_reached(const CornerShuntContext &context,
                           const CornerShuntPose &pose) const {
    if (!pose_is_safe(context, pose))
      return false;
    const double yaw_error =
        std::abs(NormalizeShuntAngle(context.target_heading - pose.yaw));
    if (yaw_error > std::max(0.0, config_.target_yaw_tolerance))
      return false;
    const auto local = local_turn_pose(context, pose);
    if (!context.reverse_arrival && local.yaw < 0.0)
      return false;
    const double target_scale = ClampShunt(std::abs(turn_error(context)) /
                                               (0.5 * 3.14159265358979323846),
                                           0.45, 1.40);
    const double target_x = config_.target_x * target_scale;
    const double target_y = config_.target_y * target_scale;
    return local.y + std::max(0.0, config_.target_y_tolerance) >= target_y &&
           std::abs(local.x - target_x) <=
               std::max(0.0, config_.target_x_tolerance) * target_scale;
  }

  double turn_error(const CornerShuntContext &context) const {
    return NormalizeShuntAngle(context.target_heading -
                               context.geometry_heading);
  }

  double turn_sign(const CornerShuntContext &context) const {
    return turn_error(context) >= 0.0 ? 1.0 : -1.0;
  }

  void update_yaw_rate_feedback(const CornerShuntContext &context,
                                double now_s) {
    if (!context.measured_yaw_rate_valid ||
        !std::isfinite(context.measured_twist.angular_z)) {
      yaw_rate_feedback_initialized_ = false;
      filtered_yaw_rate_ = context.applied_twist.angular_z;
      last_yaw_rate_feedback_at_ = now_s;
      return;
    }
    if (!yaw_rate_feedback_initialized_) {
      filtered_yaw_rate_ = context.measured_twist.angular_z;
      yaw_rate_feedback_initialized_ = true;
      last_yaw_rate_feedback_at_ = now_s;
      return;
    }
    const double elapsed = std::max(0.0, now_s - last_yaw_rate_feedback_at_);
    const double filter_time =
      std::max(0.0, config_.closed_loop_yaw_rate_filter_s);
    const double alpha = filter_time <= 0.0
      ? 1.0 : elapsed / (filter_time + elapsed);
    filtered_yaw_rate_ += alpha *
      (context.measured_twist.angular_z - filtered_yaw_rate_);
    last_yaw_rate_feedback_at_ = now_s;
  }

  CornerShuntConfig config_;
  bool active_{false};
  CornerShuntPhase phase_{CornerShuntPhase::IDLE};
  double started_at_{0.0};
  double phase_started_at_{0.0};
  std::vector<CornerShuntTwist> planned_commands_;
  std::size_t planned_command_index_{0};
  int plan_count_{0};
  int command_cycles_remaining_{0};
  std::future<std::vector<CornerShuntTwist>> plan_future_;
  CornerShuntTwist last_command_;
  bool yaw_rate_feedback_initialized_{false};
  double filtered_yaw_rate_{0.0};
  double last_yaw_rate_feedback_at_{0.0};
  bool corner_entry_committed_{false};
  bool progress_initialized_{false};
  double progress_started_at_{0.0};
  CornerShuntPose progress_pose_;
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__CORNER_SHUNT_HPP_
