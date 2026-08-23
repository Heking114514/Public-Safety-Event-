#ifndef FUSED_ODOMETRY__SENSOR_INPUT_VALIDATION_HPP_
#define FUSED_ODOMETRY__SENSOR_INPUT_VALIDATION_HPP_

#include <cstdint>
#include <cmath>
#include <string>

namespace fused_odometry
{

enum class StampValidation
{
  kAccepted,
  kUnset,
  kNonMonotonic,
  kStale,
  kFuture
};

// Pure timestamp policy shared by every sensor callback. ROS logging and state
// mutation stay in the node, so this rule can be tested without spinning ROS.
inline StampValidation ValidateMeasurementStamp(
  std::int64_t stamp_ns, bool previous_valid, std::int64_t previous_stamp_ns,
  std::int64_t now_ns, double max_age_s, double future_tolerance_s)
{
  if (stamp_ns <= 0)
    return StampValidation::kUnset;
  if (previous_valid && stamp_ns <= previous_stamp_ns)
    return StampValidation::kNonMonotonic;
  if (now_ns <= 0)
    return StampValidation::kAccepted;
  const double age_s = static_cast<double>(now_ns - stamp_ns) * 1.0e-9;
  if (!std::isfinite(age_s) || age_s > max_age_s)
    return StampValidation::kStale;
  if (age_s < -future_tolerance_s)
    return StampValidation::kFuture;
  return StampValidation::kAccepted;
}

inline bool FramePairMatches(
  const std::string & actual_parent, const std::string & actual_child,
  const std::string & expected_parent, const std::string & expected_child)
{
  return actual_parent == expected_parent && actual_child == expected_child;
}

inline bool FiniteAndWithin(double value, double maximum_abs_value)
{
  return std::isfinite(value) && std::isfinite(maximum_abs_value) &&
    maximum_abs_value >= 0.0 && std::abs(value) <= maximum_abs_value;
}

}  // namespace fused_odometry

#endif  // FUSED_ODOMETRY__SENSOR_INPUT_VALIDATION_HPP_
