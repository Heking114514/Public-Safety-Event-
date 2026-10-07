#ifndef VISUAL_NAVIGATION__CORNER_SHUNT_HPP_
#define VISUAL_NAVIGATION__CORNER_SHUNT_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <limits>
#include <thread>
#include <vector>

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
  double road_margin{0.008};
  double road_back{0.60};
  double road_forward{0.56};
  double vehicle_length{0.1485};
  double vehicle_width{0.1250};
  double wheel_track{0.1466};
  double base_offset_x{0.030};
  double base_offset_y{0.0};
  double stop_duration{0.18};
  double check_duration{0.05};
  double step_duration{0.20};
  double target_yaw_tolerance{0.08};
  double target_x{-0.0075};
  double target_y{0.1225};
  double target_x_tolerance{0.025};
  double target_y_tolerance{0.015};
  double max_turn_angle{2.20};
  double max_turn_duration{60.00};
  double prediction_horizon{0.20};
  double prediction_step{0.04};
  double plan_linear_speed{0.020};
  double plan_reverse_speed{-0.020};
  double plan_angular_speed{0.24};
  double plan_clearance{0.001};
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

struct CornerShuntContext {
  CornerShuntPose pose;
  CornerShuntTwist applied_twist;
  double junction_x{0.0};
  double junction_y{0.0};
  double incoming_heading{0.0};
  double target_heading{0.0};
  bool reverse_arrival{false};
};

struct CornerShuntDecision {
  CornerShuntPhase phase{CornerShuntPhase::IDLE};
  CornerShuntTwist command;
  bool complete{false};
  bool blocked{false};
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
    return "TURN_BLOCKED";
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
    if (now_s - started_at_ > std::max(0.0, config_.max_turn_duration))
      return block();
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
      return choose_action(context, now_s);
    }
    if (phase_ == CornerShuntPhase::BLOCKED)
      return block();

    if (command_cycles_remaining_ > 1 &&
        command_is_safe(context, last_command_,
                        1.0 / std::max(1.0, config_.control_frequency))) {
      --command_cycles_remaining_;
      return {phase_, last_command_, false, false};
    }
    return choose_action(context, now_s);
  }

  bool pose_is_safe(const CornerShuntContext &context,
                    const CornerShuntPose &pose) const {
    const auto world_corners = footprint_corners(pose);
    std::array<CornerShuntPose, 4> corners{};
    const double cos_heading = std::cos(context.incoming_heading);
    const double sin_heading = std::sin(context.incoming_heading);
    const double direction = turn_sign(context);
    for (std::size_t index = 0; index < world_corners.size(); ++index) {
      const double dx = world_corners[index].x - context.junction_x;
      const double dy = world_corners[index].y - context.junction_y;
      corners[index] = {cos_heading * dx + sin_heading * dy,
                        (-sin_heading * dx + cos_heading * dy) * direction,
                        0.0};
    }

    const double half = std::max(0.0, 0.5 * config_.road_width);
    const double margin = ClampShunt(config_.road_margin, 0.0, half - 1.0e-6);
    const double safe_half = half - margin;
    const double back = std::max(0.0, config_.road_back) - margin;
    const double forward = std::max(0.0, config_.road_forward) - margin;
    for (const auto &corner : corners) {
      if (corner.x < -back || corner.x > safe_half || corner.y < -safe_half ||
          corner.y > forward)
        return false;
    }

    // The L-shaped road is its outer rectangle minus the upper-left block.
    // A separating-axis test rejects any positive-area overlap with that block;
    // touching x=-safe_half or y=safe_half remains valid road boundary contact.
    const std::array<CornerShuntPose, 4> blocked{{
        {-back, safe_half, 0.0},
        {-safe_half, safe_half, 0.0},
        {-safe_half, forward, 0.0},
        {-back, forward, 0.0},
    }};
    const double edge_x = corners[1].x - corners[0].x;
    const double edge_y = corners[1].y - corners[0].y;
    const double side_x = corners[3].x - corners[0].x;
    const double side_y = corners[3].y - corners[0].y;
    const std::array<CornerShuntPose, 4> axes{{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {edge_x, edge_y, 0.0},
        {side_x, side_y, 0.0},
    }};
    constexpr double kTouchTolerance = 1.0e-9;
    for (const auto &axis : axes) {
      double vehicle_min = std::numeric_limits<double>::infinity();
      double vehicle_max = -std::numeric_limits<double>::infinity();
      double blocked_min = std::numeric_limits<double>::infinity();
      double blocked_max = -std::numeric_limits<double>::infinity();
      for (const auto &corner : corners) {
        const double projection = corner.x * axis.x + corner.y * axis.y;
        vehicle_min = std::min(vehicle_min, projection);
        vehicle_max = std::max(vehicle_max, projection);
      }
      for (const auto &corner : blocked) {
        const double projection = corner.x * axis.x + corner.y * axis.y;
        blocked_min = std::min(blocked_min, projection);
        blocked_max = std::max(blocked_max, projection);
      }
      const double overlap = std::min(vehicle_max, blocked_max) -
                             std::max(vehicle_min, blocked_min);
      if (overlap <= kTouchTolerance)
        return true;
    }
    return false;
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

  CornerShuntDecision block() {
    phase_ = CornerShuntPhase::BLOCKED;
    last_command_ = {};
    return {phase_, {}, false, true};
  }

  CornerShuntPose local_turn_pose(const CornerShuntContext &context,
                                  CornerShuntPose pose) const {
    const double cos_heading = std::cos(context.incoming_heading);
    const double sin_heading = std::sin(context.incoming_heading);
    const double dx = pose.x - context.junction_x;
    const double dy = pose.y - context.junction_y;
    const double sign = turn_sign(context);
    return {cos_heading * dx + sin_heading * dy,
            (-sin_heading * dx + cos_heading * dy) * sign,
            NormalizeShuntAngle(pose.yaw - context.incoming_heading) * sign};
  }

  bool command_is_safe(const CornerShuntContext &context,
                       CornerShuntTwist command,
                       double horizon_s = -1.0) const {
    if (std::abs(command.linear_x) <= 1.0e-9 &&
        std::abs(command.angular_z) <= 1.0e-9)
      return true;
    if (!pose_is_safe(context, context.pose))
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
      if (!pose_is_safe(context, pose))
        return false;
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
    return {
        LimitRate(target.linear_x, current.linear_x,
                  config_.linear_acceleration, config_.linear_deceleration,
                  elapsed_s),
        LimitRate(target.angular_z, current.angular_z,
                  config_.angular_acceleration, config_.angular_deceleration,
                  elapsed_s),
    };
  }

  std::array<CornerShuntPose, 4>
  footprint_corners(const CornerShuntPose &pose) const {
    const double half_length = 0.5 * std::max(0.0, config_.vehicle_length);
    const double half_width = 0.5 * std::max(0.0, config_.vehicle_width);
    const CornerShuntPose center{config_.base_offset_x, config_.base_offset_y,
                                 0.0};
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    std::array<CornerShuntPose, 4> corners{};
    const std::array<CornerShuntPose, 4> local{{
        {center.x - half_length, center.y - half_width, 0.0},
        {center.x + half_length, center.y - half_width, 0.0},
        {center.x + half_length, center.y + half_width, 0.0},
        {center.x - half_length, center.y + half_width, 0.0},
    }};
    for (std::size_t index = 0; index < local.size(); ++index) {
      corners[index].x =
          pose.x + cos_yaw * local[index].x - sin_yaw * local[index].y;
      corners[index].y =
          pose.y + sin_yaw * local[index].x + cos_yaw * local[index].y;
    }
    return corners;
  }

  bool point_inside_safe_road(const CornerShuntContext &context,
                              CornerShuntPose point) const {
    const double cos_heading = std::cos(context.incoming_heading);
    const double sin_heading = std::sin(context.incoming_heading);
    const double dx = point.x - context.junction_x;
    const double dy = point.y - context.junction_y;
    const double local_x = cos_heading * dx + sin_heading * dy;
    double local_y = -sin_heading * dx + cos_heading * dy;
    local_y *= turn_sign(context);

    const double half = std::max(0.0, 0.5 * config_.road_width);
    const double margin = ClampShunt(config_.road_margin, 0.0, half - 1.0e-6);
    const double safe_half = half - margin;
    const double back = std::max(0.0, config_.road_back) - margin;
    const double forward = std::max(0.0, config_.road_forward) - margin;
    const bool in_entry = -back <= local_x && local_x <= safe_half &&
                          -safe_half <= local_y && local_y <= safe_half;
    const bool in_exit = -safe_half <= local_x && local_x <= safe_half &&
                         -safe_half <= local_y && local_y <= forward;
    return in_entry || in_exit;
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
                               context.incoming_heading);
  }

  double turn_sign(const CornerShuntContext &context) const {
    return turn_error(context) >= 0.0 ? 1.0 : -1.0;
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
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__CORNER_SHUNT_HPP_
