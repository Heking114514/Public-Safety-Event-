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

inline double FinalApproachSpeedLimit(
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
