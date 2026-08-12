#ifndef VISUAL_NAVIGATION__PATH_CONTROL_HPP_
#define VISUAL_NAVIGATION__PATH_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace visual_navigation
{

struct PlanarPoint
{
  double x{0.0};
  double y{0.0};
};

struct PolylineLookahead
{
  bool valid{false};
  PlanarPoint projection{};
  PlanarPoint point{};
  double segment_heading{0.0};
  double cross_track_error{0.0};
  double distance{0.0};
};

inline PolylineLookahead ComputePolylineLookahead(
  const std::vector<PlanarPoint> &points,
  double current_x, double current_y, double lookahead_distance)
{
  PolylineLookahead result;
  if (points.size() < 2 || !std::isfinite(current_x) ||
    !std::isfinite(current_y) || !std::isfinite(lookahead_distance) ||
    lookahead_distance < 0.0)
  {
    return result;
  }

  constexpr double kEpsilon = 1.0e-9;
  double closest_distance_squared = std::numeric_limits<double>::infinity();
  std::size_t closest_segment = points.size();
  double closest_fraction = 0.0;
  double closest_length = 0.0;

  for (std::size_t index = 0; index + 1 < points.size(); ++index)
  {
    const PlanarPoint &start = points[index];
    const PlanarPoint &end = points[index + 1];
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
      !std::isfinite(end.x) || !std::isfinite(end.y))
    {
      return result;
    }

    const double delta_x = end.x - start.x;
    const double delta_y = end.y - start.y;
    const double length_squared = delta_x * delta_x + delta_y * delta_y;
    if (length_squared <= kEpsilon)
      continue;

    const double fraction = std::max(0.0, std::min(1.0,
      ((current_x - start.x) * delta_x + (current_y - start.y) * delta_y) /
      length_squared));
    const double projection_x = start.x + fraction * delta_x;
    const double projection_y = start.y + fraction * delta_y;
    const double error_x = current_x - projection_x;
    const double error_y = current_y - projection_y;
    const double distance_squared = error_x * error_x + error_y * error_y;
    if (distance_squared < closest_distance_squared)
    {
      closest_distance_squared = distance_squared;
      closest_segment = index;
      closest_fraction = fraction;
      closest_length = std::sqrt(length_squared);
    }
  }

  if (closest_segment >= points.size() - 1)
    return result;

  const PlanarPoint &closest_start = points[closest_segment];
  const PlanarPoint &closest_end = points[closest_segment + 1];
  const double closest_delta_x = closest_end.x - closest_start.x;
  const double closest_delta_y = closest_end.y - closest_start.y;
  const double closest_heading = std::atan2(closest_delta_y, closest_delta_x);
  result.projection = {
    closest_start.x + closest_fraction * closest_delta_x,
    closest_start.y + closest_fraction * closest_delta_y};
  result.cross_track_error =
    std::cos(closest_heading) * (current_y - result.projection.y) -
    std::sin(closest_heading) * (current_x - result.projection.x);

  std::size_t segment = closest_segment;
  double position = closest_fraction * closest_length;
  double remaining = lookahead_distance;
  while (true)
  {
    const PlanarPoint &start = points[segment];
    const PlanarPoint &end = points[segment + 1];
    const double delta_x = end.x - start.x;
    const double delta_y = end.y - start.y;
    const double length = std::hypot(delta_x, delta_y);
    if (length <= kEpsilon)
    {
      if (segment + 1 >= points.size() - 1)
        break;
      ++segment;
      position = 0.0;
      continue;
    }

    const double available = std::max(0.0, length - position);
    if (remaining <= available + kEpsilon || segment + 1 >= points.size() - 1)
    {
      const double travel = std::min(available, std::max(0.0, remaining));
      const double fraction = std::min(1.0, (position + travel) / length);
      result.point = {
        start.x + fraction * delta_x,
        start.y + fraction * delta_y};
      result.segment_heading = std::atan2(delta_y, delta_x);
      result.distance = lookahead_distance - std::max(0.0, remaining - travel);
      result.valid = true;
      return result;
    }

    remaining -= available;
    ++segment;
    position = 0.0;
  }

  return result;
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

inline double PathSegmentProgress(
  double start_x, double start_y, double target_x, double target_y,
  double current_x, double current_y)
{
  if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
    !std::isfinite(target_x) || !std::isfinite(target_y) ||
    !std::isfinite(current_x) || !std::isfinite(current_y))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double segment_x = target_x - start_x;
  const double segment_y = target_y - start_y;
  const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
  if (segment_length_squared <= 1.0e-12)
    return std::numeric_limits<double>::quiet_NaN();

  return ((current_x - start_x) * segment_x +
    (current_y - start_y) * segment_y) / segment_length_squared;
}

inline bool WaypointReached(
  double distance, double tolerance, double segment_progress,
  double cross_track_error = std::numeric_limits<double>::quiet_NaN(),
  double passed_waypoint_cross_track_tolerance = 0.0)
{
  const bool inside_tolerance =
    std::isfinite(distance) && std::isfinite(tolerance) &&
    tolerance > 0.0 && distance <= tolerance;
  const bool passed_target =
    std::isfinite(segment_progress) && segment_progress >= 1.0 &&
    std::isfinite(cross_track_error) &&
    std::isfinite(passed_waypoint_cross_track_tolerance) &&
    passed_waypoint_cross_track_tolerance >= 0.0 &&
    std::abs(cross_track_error) <= passed_waypoint_cross_track_tolerance;
  return inside_tolerance || passed_target;
}

inline double WaypointApproachDistance(
  double euclidean_distance, double segment_progress, double segment_length)
{
  if (!std::isfinite(euclidean_distance))
    return 0.0;

  const double fallback_distance = std::max(0.0, euclidean_distance);
  if (!std::isfinite(segment_progress) || !std::isfinite(segment_length) ||
    segment_length <= 1.0e-6)
  {
    return fallback_distance;
  }

  const double along_track_remaining =
    std::max(0.0, (1.0 - segment_progress) * segment_length);
  return std::min(fallback_distance, along_track_remaining);
}

inline bool FinalPositionCaptured(
  bool already_captured, bool final_waypoint, double distance, double tolerance,
  double segment_progress = std::numeric_limits<double>::quiet_NaN(),
  double cross_track_error = std::numeric_limits<double>::quiet_NaN(),
  double passed_waypoint_cross_track_tolerance = 0.0)
{
  return already_captured ||
    (final_waypoint && WaypointReached(
      distance, tolerance, segment_progress, cross_track_error,
      passed_waypoint_cross_track_tolerance));
}

inline double TurnSpeedForError(
  double absolute_error, double settle_tolerance, double slow_threshold,
  double minimum_speed, double maximum_speed)
{
  if (!std::isfinite(absolute_error) || !std::isfinite(settle_tolerance) ||
    !std::isfinite(slow_threshold) || !std::isfinite(minimum_speed) ||
    !std::isfinite(maximum_speed))
  {
    return 0.0;
  }
  const double maximum = std::max(0.0, maximum_speed);
  const double minimum = std::min(maximum, std::max(0.0, minimum_speed));
  const double settle = std::max(0.0, settle_tolerance);
  const double slow = std::max(settle + 1.0e-6, slow_threshold);
  if (absolute_error <= settle)
    return 0.0;
  if (absolute_error >= slow)
    return maximum;
  const double ratio = (absolute_error - settle) / (slow - settle);
  return minimum + ratio * (maximum - minimum);
}

inline bool ShouldBrakeTurn(
  double directed_error, double absolute_yaw_rate,
  double settle_tolerance, double brake_horizon)
{
  if (!std::isfinite(directed_error) || !std::isfinite(absolute_yaw_rate) ||
    !std::isfinite(settle_tolerance) || !std::isfinite(brake_horizon))
  {
    return true;
  }
  const double braking_error = std::max(0.0, settle_tolerance) +
    std::max(0.0, brake_horizon) * std::max(0.0, absolute_yaw_rate);
  return directed_error <= braking_error;
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
