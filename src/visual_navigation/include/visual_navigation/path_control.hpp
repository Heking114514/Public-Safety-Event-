#ifndef VISUAL_NAVIGATION__PATH_CONTROL_HPP_
#define VISUAL_NAVIGATION__PATH_CONTROL_HPP_

#include <algorithm>
#include <cmath>

namespace visual_navigation
{

struct PlanarPoint
{
  double x{0.0};
  double y{0.0};
};

struct PathProjection
{
  double remaining{0.0};
  double cross_track{0.0};
  bool valid{false};
};

inline PathProjection ProjectOntoPathSegment(
  double start_x, double start_y, double target_x, double target_y,
  double current_x, double current_y)
{
  if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(target_x) || !std::isfinite(target_y) ||
    !std::isfinite(current_x) || !std::isfinite(current_y))
  {
    return {};
  }

  const double delta_x = target_x - start_x;
  const double delta_y = target_y - start_y;
  const double length = std::hypot(delta_x, delta_y);
  if (length <= 1.0e-9) {
    return {};
  }
  const double unit_x = delta_x / length;
  const double unit_y = delta_y / length;
  const double current_from_start_x = current_x - start_x;
  const double current_from_start_y = current_y - start_y;
  const double progress =
    unit_x * current_from_start_x + unit_y * current_from_start_y;
  return {
    length - progress,
    unit_x * current_from_start_y - unit_y * current_from_start_x,
    true};
}

inline bool WaypointReached(
  double distance, double radial_tolerance, const PathProjection & projection,
  double pass_longitudinal_tolerance, double pass_lateral_tolerance)
{
  if (!std::isfinite(distance) || !std::isfinite(radial_tolerance) ||
    !std::isfinite(pass_longitudinal_tolerance) ||
    !std::isfinite(pass_lateral_tolerance))
  {
    return false;
  }
  const double radial = std::max(0.0, radial_tolerance);
  if (distance <= radial) {
    return true;
  }
  return projection.valid &&
    projection.remaining <= std::max(0.0, pass_longitudinal_tolerance) &&
    std::abs(projection.cross_track) <=
    std::max(radial, std::max(0.0, pass_lateral_tolerance));
}

inline bool WaypointNeedsRecovery(
  const PathProjection & projection, double pass_longitudinal_tolerance,
  double pass_lateral_tolerance)
{
  if (!projection.valid || !std::isfinite(pass_longitudinal_tolerance) ||
    !std::isfinite(pass_lateral_tolerance))
  {
    return false;
  }
  return projection.remaining <= std::max(0.0, pass_longitudinal_tolerance) &&
    std::abs(projection.cross_track) > std::max(0.0, pass_lateral_tolerance);
}

inline double EndpointApproachDistance(
  double euclidean_distance, const PathProjection & projection)
{
  if (!std::isfinite(euclidean_distance)) {
    return 0.0;
  }
  const double distance = std::max(0.0, euclidean_distance);
  if (!projection.valid || !std::isfinite(projection.remaining)) {
    return distance;
  }
  return std::min(distance, std::max(0.0, projection.remaining));
}

inline PlanarPoint BasePositionFromTrackingPoint(
  double tracking_x, double tracking_y, double yaw, double reference_yaw,
  double tracking_offset_x, double tracking_offset_y)
{
  if (!std::isfinite(tracking_x) || !std::isfinite(tracking_y) ||
    !std::isfinite(yaw) || !std::isfinite(reference_yaw) ||
    !std::isfinite(tracking_offset_x) || !std::isfinite(tracking_offset_y))
  {
    return {};
  }
  const double current_offset_x =
    std::cos(yaw) * tracking_offset_x - std::sin(yaw) * tracking_offset_y;
  const double current_offset_y =
    std::sin(yaw) * tracking_offset_x + std::cos(yaw) * tracking_offset_y;
  const double reference_offset_x =
    std::cos(reference_yaw) * tracking_offset_x -
    std::sin(reference_yaw) * tracking_offset_y;
  const double reference_offset_y =
    std::sin(reference_yaw) * tracking_offset_x +
    std::cos(reference_yaw) * tracking_offset_y;
  return {
    tracking_x - current_offset_x + reference_offset_x,
    tracking_y - current_offset_y + reference_offset_y};
}

inline double StanleyPathError(
  double heading_error, double cross_track_error, double gain,
  double speed, double softening_speed, double maximum_correction)
{
  if (!std::isfinite(heading_error) || !std::isfinite(cross_track_error) ||
    !std::isfinite(gain) || !std::isfinite(speed) ||
    !std::isfinite(softening_speed) || !std::isfinite(maximum_correction))
  {
    return 0.0;
  }
  const double denominator = std::max(
    1.0e-6, std::abs(speed) + std::max(0.0, softening_speed));
  const double limit = std::max(0.0, maximum_correction);
  const double unbounded_correction =
    std::atan2(std::max(0.0, gain) * cross_track_error, denominator);
  const double correction = std::max(-limit, std::min(limit, unbounded_correction));
  return std::remainder(
    heading_error - correction, 2.0 * 3.14159265358979323846);
}

inline bool FinalPositionCaptured(
  bool already_captured, bool final_waypoint, double distance, double tolerance)
{
  return already_captured ||
    (final_waypoint && std::isfinite(distance) && std::isfinite(tolerance) &&
    tolerance > 0.0 && distance <= tolerance);
}

inline bool FinalPositionCanComplete(
  bool position_captured, double current_distance, double radial_tolerance)
{
  return position_captured && std::isfinite(current_distance) &&
    std::isfinite(radial_tolerance) && radial_tolerance > 0.0 &&
    current_distance <= radial_tolerance;
}

inline bool ShouldBeginWaypointBrake(
  bool braking, bool brake_completed, bool waypoint_reached, bool stop_required)
{
  return !braking && !brake_completed && waypoint_reached && stop_required;
}

inline double TurnPositionHoldSpeed(
  double current_x, double current_y, double current_yaw,
  double anchor_x, double anchor_y, double gain,
  double deadband, double maximum_speed)
{
  if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
    !std::isfinite(current_yaw) || !std::isfinite(anchor_x) ||
    !std::isfinite(anchor_y) || !std::isfinite(gain) ||
    !std::isfinite(deadband) || !std::isfinite(maximum_speed))
  {
    return 0.0;
  }

  const double error_x = anchor_x - current_x;
  const double error_y = anchor_y - current_y;
  const double forward_error =
    std::cos(current_yaw) * error_x + std::sin(current_yaw) * error_y;
  const double active_error = std::max(
    0.0, std::abs(forward_error) - std::max(0.0, deadband));
  const double command = std::max(0.0, gain) * active_error;
  return std::copysign(
    std::min(std::max(0.0, maximum_speed), command), forward_error);
}

inline bool TurnHasSettled(
  double absolute_yaw_rate, double seconds_below_threshold,
  double settle_yaw_rate, double settle_dwell)
{
  if (!std::isfinite(absolute_yaw_rate) ||
    !std::isfinite(seconds_below_threshold) ||
    !std::isfinite(settle_yaw_rate) || !std::isfinite(settle_dwell))
  {
    return false;
  }
  return absolute_yaw_rate <= std::max(0.0, settle_yaw_rate) &&
    seconds_below_threshold >= std::max(0.0, settle_dwell);
}

inline double CrossTrackSpeedLimit(
  double requested_speed, double absolute_cross_track_error,
  double slowdown_start, double slowdown_full, double minimum_speed)
{
  if (!std::isfinite(requested_speed) || !std::isfinite(absolute_cross_track_error) ||
    !std::isfinite(slowdown_start) || !std::isfinite(slowdown_full) ||
    !std::isfinite(minimum_speed))
  {
    return 0.0;
  }
  const double requested = std::max(0.0, requested_speed);
  const double minimum = std::min(requested, std::max(0.0, minimum_speed));
  const double start = std::max(0.0, slowdown_start);
  const double full = std::max(start + 1.0e-6, slowdown_full);
  if (absolute_cross_track_error <= start)
    return requested;
  if (absolute_cross_track_error >= full)
    return minimum;
  const double progress = (absolute_cross_track_error - start) / (full - start);
  return requested + progress * (minimum - requested);
}

inline double WaypointApproachSpeedLimit(
  double speed_after_cross_track_limit, double requested_speed,
  double linear_gain, double distance)
{
  if (!std::isfinite(speed_after_cross_track_limit) ||
    !std::isfinite(requested_speed) || !std::isfinite(linear_gain) ||
    !std::isfinite(distance))
  {
    return 0.0;
  }

  const double approach_limit =
    std::max(0.0, linear_gain) * std::max(0.0, distance);
  return std::max(
    0.0, std::min(speed_after_cross_track_limit,
      std::min(std::max(0.0, requested_speed), approach_limit)));
}

inline double BrakingDistance(
  double absolute_speed, double effective_deceleration,
  double control_delay, double safety_margin)
{
  if (!std::isfinite(absolute_speed) || !std::isfinite(effective_deceleration) ||
    !std::isfinite(control_delay) || !std::isfinite(safety_margin) ||
    effective_deceleration <= 0.0)
  {
    return 0.0;
  }

  const double speed = std::max(0.0, absolute_speed);
  return speed * speed / (2.0 * effective_deceleration) +
    speed * std::max(0.0, control_delay) + std::max(0.0, safety_margin);
}

inline double BrakingSpeedLimit(
  double requested_speed, double remaining_distance,
  double effective_deceleration, double control_delay, double safety_margin)
{
  if (!std::isfinite(requested_speed) || !std::isfinite(remaining_distance) ||
    !std::isfinite(effective_deceleration) || !std::isfinite(control_delay) ||
    !std::isfinite(safety_margin) || effective_deceleration <= 0.0)
  {
    return 0.0;
  }

  const double deceleration = effective_deceleration;
  const double delay = std::max(0.0, control_delay);
  const double usable_distance = std::max(
    0.0, remaining_distance - std::max(0.0, safety_margin));
  // Invert d = v^2/(2a) + v*t so the configured delay remains in the profile.
  const double delay_speed = deceleration * delay;
  const double safe_speed = -delay_speed + std::sqrt(
    delay_speed * delay_speed + 2.0 * deceleration * usable_distance);
  return std::min(std::max(0.0, requested_speed), std::max(0.0, safe_speed));
}

inline double BrakingFeedbackSpeedLimit(
  double profile_speed_limit, double absolute_measured_speed,
  double remaining_distance, double effective_deceleration,
  double control_delay, double safety_margin, double distance_feedback_gain)
{
  if (!std::isfinite(profile_speed_limit) ||
    !std::isfinite(absolute_measured_speed) ||
    !std::isfinite(remaining_distance) ||
    !std::isfinite(distance_feedback_gain))
  {
    return 0.0;
  }
  const double profile_limit = std::max(0.0, profile_speed_limit);
  const double stopping_distance = BrakingDistance(
    absolute_measured_speed, effective_deceleration, control_delay, safety_margin);
  const double excess_stopping_distance =
    stopping_distance - std::max(0.0, remaining_distance);
  if (excess_stopping_distance <= 0.0)
    return profile_limit;

  const double feedback_limit = std::max(
    0.0, absolute_measured_speed -
    std::max(0.0, distance_feedback_gain) * excess_stopping_distance);
  return std::min(profile_limit, feedback_limit);
}

inline bool WaypointStopSatisfied(
  double absolute_speed, double seconds_below_threshold,
  double total_stop_seconds, double speed_threshold, double settle_dwell,
  double minimum_stop_time, double timeout)
{
  if (!std::isfinite(total_stop_seconds) || !std::isfinite(timeout))
    return false;
  if (total_stop_seconds >= std::max(0.0, timeout))
    return true;
  if (!std::isfinite(absolute_speed) ||
    !std::isfinite(seconds_below_threshold) ||
    !std::isfinite(speed_threshold) || !std::isfinite(settle_dwell) ||
    !std::isfinite(minimum_stop_time))
  {
    return false;
  }
  return total_stop_seconds >= std::max(0.0, minimum_stop_time) &&
    absolute_speed <= std::max(0.0, speed_threshold) &&
    seconds_below_threshold >= std::max(0.0, settle_dwell);
}

inline double CompensatePathTurnDeadband(
  double requested_angular_speed, double path_error, double yaw_rate,
  double activation_error, double response_yaw_rate,
  double minimum_angular_speed, double maximum_angular_speed)
{
  if (!std::isfinite(requested_angular_speed) || !std::isfinite(path_error) ||
    !std::isfinite(yaw_rate) || !std::isfinite(activation_error) ||
    !std::isfinite(response_yaw_rate) || !std::isfinite(minimum_angular_speed) ||
    !std::isfinite(maximum_angular_speed))
  {
    return 0.0;
  }

  const double maximum = std::max(0.0, maximum_angular_speed);
  const double minimum = std::min(maximum, std::max(0.0, minimum_angular_speed));
  const double command = std::max(-maximum, std::min(maximum, requested_angular_speed));
  if (std::abs(path_error) < std::max(0.0, activation_error) ||
    std::abs(command) >= minimum)
  {
    return command;
  }

  // Preserve a small opposing command used to brake an existing turn. Boost
  // only while the requested correction and path error point the same way.
  if (std::abs(command) > 1.0e-12 && command * path_error < 0.0)
    return command;
  const bool responding_toward_path = yaw_rate * path_error > 0.0 &&
    std::abs(yaw_rate) >= std::max(0.0, response_yaw_rate);
  if (responding_toward_path)
    return command;

  return std::copysign(minimum, std::abs(command) > 1.0e-12 ? command : path_error);
}

inline bool ShouldRotateInPlace(
  bool currently_rotating, double absolute_heading_error,
  double absolute_yaw_rate, double enter_threshold, double exit_threshold,
  double settle_yaw_rate)
{
  if (!std::isfinite(absolute_heading_error) || !std::isfinite(absolute_yaw_rate))
    return true;
  const double enter = std::max(0.0, enter_threshold);
  const double exit = std::min(enter, std::max(0.0, exit_threshold));
  const double settled_rate = std::max(0.0, settle_yaw_rate);
  return currently_rotating ?
    absolute_heading_error > exit || absolute_yaw_rate > settled_rate :
    absolute_heading_error >= enter;
}

inline double RotationEntryThreshold(
  bool path_alignment_completed, double initial_threshold,
  double reentry_threshold)
{
  if (!std::isfinite(initial_threshold) || !std::isfinite(reentry_threshold))
    return 0.0;
  const double initial = std::max(0.0, initial_threshold);
  return path_alignment_completed ?
    std::max(initial, reentry_threshold) : initial;
}

inline double EnforceMinimumTurnSpeed(
  double command, double error, double minimum_speed, double maximum_speed)
{
  if (!std::isfinite(command) || !std::isfinite(error))
    return 0.0;
  const double maximum = std::max(0.0, maximum_speed);
  const double minimum = std::min(maximum, std::max(0.0, minimum_speed));
  command = std::max(-maximum, std::min(maximum, command));
  if (std::abs(error) <= 1e-12 || std::abs(command) >= minimum)
    return command;
  return std::copysign(minimum, std::abs(command) > 1e-12 ? command : error);
}

inline double SelectMinimumTurnSpeed(
  double absolute_error, double precision_threshold,
  double precision_minimum_speed, double coarse_minimum_speed)
{
  const double coarse = std::max(0.0, coarse_minimum_speed);
  const double precision = std::min(coarse, std::max(0.0, precision_minimum_speed));
  if (!std::isfinite(absolute_error))
    return coarse;
  return absolute_error <= std::max(0.0, precision_threshold) ? precision : coarse;
}

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__PATH_CONTROL_HPP_
