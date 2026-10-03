#ifndef VISUAL_NAVIGATION__CORNER_SHUNT_HPP_
#define VISUAL_NAVIGATION__CORNER_SHUNT_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

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
  double vehicle_width{0.1535};
  double wheel_track{0.1466};
  double base_offset_x{0.050};
  double base_offset_y{0.0};
  double stop_duration{0.18};
  double check_duration{0.05};
  double arc_duration{0.19};
  double rotate_duration{0.19};
  double creep_duration{0.19};
  double reverse_duration{0.19};
  double start_x{-0.070};
  double start_x_tolerance{0.005};
  double target_yaw_tolerance{0.08};
  double target_x{-0.0075};
  double target_y{0.1225};
  double target_x_tolerance{0.025};
  double target_y_tolerance{0.015};
  double max_turn_angle{2.20};
  double max_turn_duration{60.00};
  double prediction_horizon{0.20};
  double prediction_step{0.04};
  double arc_linear_speed{0.010};
  double arc_angular_speed{0.24};
  double late_turn_progress{0.83};
  double late_linear_speed{0.014};
  double late_angular_speed{0.10};
  double rotate_angular_speed{0.20};
  double creep_linear_speed{0.010};
  double reverse_linear_speed{-0.008};
  double reverse_angular_speed{-0.10};
  int max_reverse_count{80};
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
  double junction_x{0.0};
  double junction_y{0.0};
  double incoming_heading{0.0};
  double target_heading{0.0};
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
    reverse_count_ = 0;
    last_command_ = {};
  }

  void begin(double now_s) {
    active_ = true;
    phase_ = CornerShuntPhase::STOP;
    started_at_ = now_s;
    phase_started_at_ = now_s;
    reverse_count_ = 0;
    last_command_ = {};
  }

  bool active() const { return active_; }

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

    const double duration = phase_duration(phase_);
    if (phase_elapsed < duration && command_is_safe(context, last_command_))
      return {phase_, last_command_, false, false};
    return choose_action(context, now_s);
  }

  bool pose_is_safe(const CornerShuntContext &context,
                    const CornerShuntPose &pose) const {
    const auto corners = footprint_corners(pose);
    for (std::size_t index = 0; index < corners.size(); ++index) {
      const auto start = corners[index];
      const auto end = corners[(index + 1) % corners.size()];
      for (int step = 0; step <= 18; ++step) {
        const double amount = static_cast<double>(step) / 18.0;
        const CornerShuntPose point{start.x + (end.x - start.x) * amount,
                                    start.y + (end.y - start.y) * amount, 0.0};
        if (!point_inside_safe_road(context, point))
          return false;
      }
    }
    CornerShuntPose center;
    for (const auto &corner : corners) {
      center.x += corner.x / static_cast<double>(corners.size());
      center.y += corner.y / static_cast<double>(corners.size());
    }
    return point_inside_safe_road(context, center);
  }

private:
  CornerShuntDecision choose_action(const CornerShuntContext &context,
                                    double now_s) {
    const auto local = local_turn_pose(context, context.pose);
    if (local.yaw < 0.12 * std::abs(turn_error(context)) &&
        local.x > config_.start_x + std::max(0.0, config_.start_x_tolerance) &&
        reverse_count_ < std::max(0, config_.max_reverse_count)) {
      const CornerShuntTwist command{std::min(config_.reverse_linear_speed,
                                              -0.006),
                                     0.0};
      if (command_is_safe(context, command))
        return set_phase(CornerShuntPhase::REVERSE, now_s, command);
    }

    Candidate best;
    for (const auto &candidate : arc_commands(context)) {
      if (!candidate.valid)
        continue;
      if (candidate.phase == CornerShuntPhase::REVERSE &&
          reverse_count_ >= std::max(0, config_.max_reverse_count))
        continue;
      if (!command_is_safe(context, candidate.command))
        continue;
      const double score =
          command_score(context, candidate.command) + candidate.penalty;
      if (!best.valid || score < best.score)
        best = {candidate.phase, candidate.command, score, true};
    }
    if (best.valid) {
      if (best.phase == CornerShuntPhase::REVERSE)
        ++reverse_count_;
      return set_phase(best.phase, now_s, best.command);
    }

    if (reverse_count_ < std::max(0, config_.max_reverse_count)) {
      for (const auto &candidate : reverse_commands(context)) {
        if (!candidate.valid)
          continue;
        if (!command_is_safe(context, candidate.command))
          continue;
        ++reverse_count_;
        return set_phase(candidate.phase, now_s, candidate.command);
      }
    }
    return block();
  }

  CornerShuntDecision set_phase(CornerShuntPhase phase, double now_s,
                                CornerShuntTwist command) {
    phase_ = phase;
    phase_started_at_ = now_s;
    last_command_ = command;
    return {phase_, command, false, false};
  }

  CornerShuntDecision block() {
    phase_ = CornerShuntPhase::BLOCKED;
    last_command_ = {};
    return {phase_, {}, false, true};
  }

  double phase_duration(CornerShuntPhase phase) const {
    switch (phase) {
    case CornerShuntPhase::ARC:
      return std::max(0.0, config_.arc_duration);
    case CornerShuntPhase::ROTATE:
      return std::max(0.0, config_.rotate_duration);
    case CornerShuntPhase::CREEP:
      return std::max(0.0, config_.creep_duration);
    case CornerShuntPhase::REVERSE:
      return std::max(0.0, config_.reverse_duration);
    default:
      break;
    }
    return 0.0;
  }

  struct Candidate {
    CornerShuntPhase phase{CornerShuntPhase::IDLE};
    CornerShuntTwist command;
    double score{std::numeric_limits<double>::infinity()};
    bool valid{false};
    double penalty{0.0};
  };

  struct ProfileTarget {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  std::array<Candidate, 16>
  arc_commands(const CornerShuntContext &context) const {
    const double sign = turn_sign(context);
    const double arc_linear = std::max(0.0, config_.arc_linear_speed);
    const double arc_angular = std::max(0.0, config_.arc_angular_speed);
    const double rotate_angular = std::max(0.0, config_.rotate_angular_speed);
    const double late_linear = std::max(0.0, config_.late_linear_speed);
    const double late_angular = std::max(0.0, config_.late_angular_speed);
    const auto planned = planned_arc_command(context);
    return {{
        {CornerShuntPhase::ARC, planned, 0.0, true, 0.00},
        {CornerShuntPhase::ARC, {0.20 * arc_linear, arc_angular * sign}, 0.0,
         true, 0.01},
        {CornerShuntPhase::ARC, {0.40 * arc_linear, arc_angular * sign}, 0.0,
         true, 0.01},
        {CornerShuntPhase::ARC, {0.60 * arc_linear, arc_angular * sign}, 0.0,
         true, 0.01},
        {CornerShuntPhase::ARC,
         {arc_linear, std::max(0.0, 0.83 * arc_angular) * sign},
         0.0,
         true,
         0.02},
        {CornerShuntPhase::ARC,
         {late_linear, late_angular * sign},
         0.0,
         true,
         0.02},
        {CornerShuntPhase::ARC,
         {late_linear, std::max(0.0, 0.70 * late_angular) * sign},
         0.0,
         true,
         0.02},
        {CornerShuntPhase::ARC,
         {std::max(late_linear, 0.016),
          std::max(0.0, 0.50 * late_angular) * sign},
         0.0,
         true,
         0.03},
        {CornerShuntPhase::ARC,
         {0.5 * (arc_linear + late_linear),
          std::max(0.0, 0.80 * late_angular) * sign},
         0.0,
         true,
         0.03},
        {CornerShuntPhase::ROTATE, {0.0, rotate_angular * sign}, 0.0, true,
         0.05},
        {CornerShuntPhase::ROTATE,
         {0.0, std::max(0.0, 0.60 * rotate_angular) * sign},
         0.0,
         true,
         0.05},
        {CornerShuntPhase::CREEP, {std::max(0.0, config_.creep_linear_speed), 0.0},
         0.0, true, 0.04},
        {CornerShuntPhase::CREEP, {std::max(late_linear, 0.020), 0.0}, 0.0,
         true, 0.04},
        {CornerShuntPhase::ARC,
         {std::max(late_linear, 0.024),
          std::max(0.0, 0.50 * late_angular) * sign},
         0.0,
         true,
         0.04},
        {CornerShuntPhase::REVERSE,
         {std::min(config_.reverse_linear_speed, -0.006),
          config_.reverse_angular_speed * sign},
         0.0,
         true,
         0.16},
        {CornerShuntPhase::REVERSE,
         {std::min(config_.reverse_linear_speed, -0.006), 0.0},
         0.0,
         true,
         0.20},
    }};
  }

  std::array<Candidate, 2>
  reverse_commands(const CornerShuntContext &context) const {
    const double sign = turn_sign(context);
    return {{
        {CornerShuntPhase::REVERSE,
         {std::min(config_.reverse_linear_speed, -0.006),
          config_.reverse_angular_speed * sign},
         0.0,
         true,
        },
        {CornerShuntPhase::REVERSE,
         {std::min(config_.reverse_linear_speed, -0.006), 0.0},
         0.0,
         true},
    }};
  }

  CornerShuntTwist planned_arc_command(const CornerShuntContext &context) const {
    const double progress = turn_progress(context);
    const double sign = turn_sign(context);
    const double arc_linear = std::max(0.0, config_.arc_linear_speed);
    const double arc_angular = std::max(0.0, config_.arc_angular_speed);
    const double late_linear = std::max(0.0, config_.late_linear_speed);
    const double late_angular = std::max(0.0, config_.late_angular_speed);
    const double late_progress = ClampShunt(config_.late_turn_progress, 0.0, 1.0);

    if (progress < 50.0 / 90.0)
      return {arc_linear, arc_angular * sign};
    if (progress < late_progress)
      return {arc_linear, std::max(0.0, 0.83 * arc_angular) * sign};
    if (progress < 80.0 / 90.0)
      return {late_linear, late_angular * sign};
    if (progress < 0.96)
      return {std::max(late_linear, 0.016),
              std::max(0.0, 0.50 * late_angular) * sign};
    return {0.5 * (arc_linear + late_linear),
            std::max(0.0, 0.80 * late_angular) * sign};
  }

  ProfileTarget profile_target(const CornerShuntContext &context) const {
    const auto local = local_turn_pose(context, context.pose);
    const double target_yaw = std::max(0.10, std::abs(turn_error(context)));
    const double target_scale =
        ClampShunt(target_yaw / (0.5 * 3.14159265358979323846), 0.45, 1.40);
    if (local.yaw < target_yaw * 50.0 / 90.0)
      return {-0.0550 * target_scale, 0.0150 * target_scale,
              target_yaw * 50.0 / 90.0};
    if (local.yaw < target_yaw * 60.0 / 90.0)
      return {-0.0380 * target_scale, 0.0300 * target_scale,
              target_yaw * 60.0 / 90.0};
    if (local.yaw < target_yaw * 65.0 / 90.0)
      return {-0.0320 * target_scale, 0.0400 * target_scale,
              target_yaw * 65.0 / 90.0};
    if (local.yaw < target_yaw * 70.0 / 90.0)
      return {-0.0240 * target_scale, 0.0550 * target_scale,
              target_yaw * 70.0 / 90.0};
    if (local.yaw < target_yaw * 80.0 / 90.0)
      return {-0.0060 * target_scale, 0.1000 * target_scale,
              target_yaw * 80.0 / 90.0};
    return {config_.target_x * target_scale, config_.target_y * target_scale,
            target_yaw};
  }

  double command_score(const CornerShuntContext &context,
                       CornerShuntTwist command) const {
    const auto predicted = predict(context.pose, command);
    const auto local = local_turn_pose(context, predicted);
    const auto current = local_turn_pose(context, context.pose);
    const auto target = profile_target(context);
    const double yaw_remaining = std::max(0.0, target.yaw - local.yaw);
    const double y_remaining = target.y - local.y;
    const double x_error = local.x - target.x;
    double score = 2.0 * yaw_remaining * yaw_remaining +
                   45.0 * x_error * x_error + 30.0 * y_remaining * y_remaining;
    if (local.yaw < current.yaw - 1.0e-3)
      score += 0.50;
    return score;
  }

  CornerShuntPose predict(CornerShuntPose pose,
                          CornerShuntTwist command) const {
    const double horizon = std::max(0.04, config_.prediction_horizon);
    const double step = std::max(0.01, config_.prediction_step);
    const int samples =
        std::max(1, static_cast<int>(std::ceil(horizon / step)));
    for (int index = 0; index < samples; ++index)
      pose = integrate(pose, command, horizon / static_cast<double>(samples));
    return pose;
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
                       CornerShuntTwist command) const {
    if (std::abs(command.linear_x) <= 1.0e-9 &&
        std::abs(command.angular_z) <= 1.0e-9)
      return true;
    if (!pose_is_safe(context, context.pose))
      return false;

    const double horizon = std::max(0.0, config_.prediction_horizon);
    const double step = std::max(0.01, config_.prediction_step);
    const int samples =
        std::max(1, static_cast<int>(std::ceil(horizon / step)));
    auto pose = context.pose;
    for (int index = 0; index < samples; ++index) {
      pose = integrate(pose, command, horizon / static_cast<double>(samples));
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
    const double yaw_error = std::abs(
        NormalizeShuntAngle(context.target_heading - pose.yaw));
    if (yaw_error > std::max(0.0, config_.target_yaw_tolerance))
      return false;
    const auto local = local_turn_pose(context, pose);
    if (local.yaw < 0.0)
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

  CornerShuntPose body_center_pose(CornerShuntPose pose) const {
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    pose.x += cos_yaw * config_.base_offset_x - sin_yaw * config_.base_offset_y;
    pose.y += sin_yaw * config_.base_offset_x + cos_yaw * config_.base_offset_y;
    return pose;
  }

  double turn_error(const CornerShuntContext &context) const {
    return NormalizeShuntAngle(context.target_heading -
                               context.incoming_heading);
  }

  double turn_sign(const CornerShuntContext &context) const {
    return turn_error(context) >= 0.0 ? 1.0 : -1.0;
  }

  double turn_progress(const CornerShuntContext &context) const {
    const double target = std::max(0.05, std::abs(turn_error(context)));
    const double progress = std::abs(
        NormalizeShuntAngle(context.pose.yaw - context.incoming_heading));
    return ClampShunt(progress / target, 0.0, 1.0);
  }

  CornerShuntConfig config_;
  bool active_{false};
  CornerShuntPhase phase_{CornerShuntPhase::IDLE};
  double started_at_{0.0};
  double phase_started_at_{0.0};
  int reverse_count_{0};
  CornerShuntTwist last_command_;
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__CORNER_SHUNT_HPP_
