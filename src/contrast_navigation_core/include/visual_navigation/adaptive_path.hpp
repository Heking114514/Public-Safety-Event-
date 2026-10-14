#ifndef VISUAL_NAVIGATION__ADAPTIVE_PATH_HPP_
#define VISUAL_NAVIGATION__ADAPTIVE_PATH_HPP_

#include <algorithm>
#include <cmath>

namespace visual_navigation
{

inline double AdaptiveErrorBlend(
  double absolute_error, double start, double full)
{
  if (!std::isfinite(absolute_error) || !std::isfinite(start) ||
    !std::isfinite(full))
  {
    return 0.0;
  }
  const double lower = std::max(0.0, start);
  const double upper = std::max(lower + 1.0e-6, full);
  if (absolute_error <= lower) {
    return 0.0;
  }
  if (absolute_error >= upper) {
    return 1.0;
  }
  const double fraction = (absolute_error - lower) / (upper - lower);
  return fraction * fraction * (3.0 - 2.0 * fraction);
}

// Gradually overcome the wheel deadband when the measured yaw rate has not
// responded. Fade the boost out smoothly as the chassis starts correcting.
inline double AdaptivePathDeadband(
  double requested_angular_speed, double path_error, double yaw_rate,
  double activation_error, double response_yaw_rate,
  double minimum_angular_speed, double maximum_angular_speed)
{
  const double maximum = std::max(0.0, maximum_angular_speed);
  const double command = std::clamp(requested_angular_speed, -maximum, maximum);
  if (command * path_error <= 0.0) {
    return command;
  }
  const double error_blend = AdaptiveErrorBlend(
    std::abs(path_error), activation_error, 2.0 * activation_error);
  const double response_blend = yaw_rate * path_error > 0.0
    ? AdaptiveErrorBlend(
        std::abs(yaw_rate), response_yaw_rate, 2.0 * response_yaw_rate)
    : 0.0;
  const double minimum = std::min(maximum, std::max(0.0, minimum_angular_speed));
  const double boost = minimum * error_blend * (1.0 - response_blend);
  return std::copysign(std::max(std::abs(command), boost), command);
}

// Increase the allowable correction smoothly as either heading or lateral
// error grows. The base ratio remains in force around the path, while a large
// error can use a larger differential command without introducing a step in w.
inline double AdaptivePathAngularSpeed(
  double requested_angular_speed, double linear_speed,
  double heading_error, double cross_track_error,
  double base_angular_ratio, double maximum_angular_ratio,
  double heading_start, double heading_full,
  double cross_track_start, double cross_track_full,
  double braking_blend = 0.0)
{
  if (!std::isfinite(requested_angular_speed) ||
    !std::isfinite(linear_speed) || !std::isfinite(heading_error) ||
    !std::isfinite(cross_track_error) || !std::isfinite(base_angular_ratio) ||
    !std::isfinite(maximum_angular_ratio) || !std::isfinite(braking_blend))
  {
    return 0.0;
  }
  const double base = std::max(0.0, base_angular_ratio);
  const double maximum = std::max(base, maximum_angular_ratio);
  const double heading_blend = AdaptiveErrorBlend(
    std::abs(heading_error), heading_start, heading_full);
  const double lateral_blend = AdaptiveErrorBlend(
    std::abs(cross_track_error), cross_track_start, cross_track_full);
  const double blend = std::max(heading_blend, lateral_blend);
  const double brake = std::max(0.0, std::min(1.0, braking_blend));
  const double ratio = base + (maximum - base) * blend * (1.0 - brake);
  const double limit = ratio * std::abs(linear_speed);
  return std::max(-limit, std::min(limit, requested_angular_speed));
}

// Slow down progressively while the correction demand grows. This gives the
// larger angular command time to move the chassis back onto the lane and keeps
// the forward command from masking a persistent heading error.
inline double AdaptiveCorrectionSpeedLimit(
  double requested_speed, double heading_error, double cross_track_error,
  double heading_start, double heading_full,
  double cross_track_start, double cross_track_full,
  double minimum_speed)
{
  if (!std::isfinite(requested_speed) || !std::isfinite(heading_error) ||
    !std::isfinite(cross_track_error) || !std::isfinite(minimum_speed))
  {
    return 0.0;
  }
  const double requested = std::max(0.0, requested_speed);
  const double minimum = std::min(requested, std::max(0.0, minimum_speed));
  const double blend = std::max(
    AdaptiveErrorBlend(std::abs(heading_error), heading_start, heading_full),
    AdaptiveErrorBlend(
      std::abs(cross_track_error), cross_track_start, cross_track_full));
  return requested + blend * (minimum - requested);
}

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__ADAPTIVE_PATH_HPP_
