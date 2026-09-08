#ifndef ARENA_PATH_PLANNER__REQUEST_VALIDATION_HPP_
#define ARENA_PATH_PLANNER__REQUEST_VALIDATION_HPP_

#include "arena_path_planner/planner.hpp"

#include <cmath>
#include <string>

namespace arena_path_planner
{

inline bool QuaternionToYaw(
  double x, double y, double z, double w, double & yaw,
  std::string * rejection_reason = nullptr)
{
  const auto reject = [rejection_reason](const char * reason) {
      if (rejection_reason != nullptr) {
        *rejection_reason = reason;
      }
      return false;
    };
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    !std::isfinite(w))
  {
    return reject("orientation contains a non-finite value");
  }
  const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return reject("orientation quaternion has no valid norm");
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  yaw = std::atan2(2.0 * (w * z + x * y),
                   1.0 - 2.0 * (y * y + z * z));
  if (!std::isfinite(yaw)) {
    return reject("orientation cannot be converted to a finite yaw");
  }
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

inline bool PointIsInsideArena(
  const Point & point, const PlannerConfig & config,
  std::string * rejection_reason = nullptr)
{
  const auto reject = [rejection_reason](const char * reason) {
      if (rejection_reason != nullptr) {
        *rejection_reason = reason;
      }
      return false;
    };
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return reject("coordinates contain a non-finite value");
  }
  if (point.x < 0.0 || point.y < 0.0 || point.x >= config.width ||
    point.y >= config.height)
  {
    return reject("coordinates lie outside the arena");
  }
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

}  // namespace arena_path_planner

#endif  // ARENA_PATH_PLANNER__REQUEST_VALIDATION_HPP_
