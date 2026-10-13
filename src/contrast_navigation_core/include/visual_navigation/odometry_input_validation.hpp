#ifndef VISUAL_NAVIGATION__ODOMETRY_INPUT_VALIDATION_HPP_
#define VISUAL_NAVIGATION__ODOMETRY_INPUT_VALIDATION_HPP_

#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <string>

namespace visual_navigation
{

// Navigation does not perform TF conversion. Both frame names therefore need
// to be present and match the configured map/base pair before a pose can
// affect control. Empty frame names are not treated as an implicit wildcard.
inline bool OdometryFramePairMatches(
  const std::string & frame_id, const std::string & child_frame_id,
  const std::string & expected_frame_id,
  const std::string & expected_child_frame_id)
{
  return !frame_id.empty() && !child_frame_id.empty() &&
    !expected_frame_id.empty() && !expected_child_frame_id.empty() &&
    frame_id == expected_frame_id && child_frame_id == expected_child_frame_id;
}

inline bool OdometryPoseIsFinite(const nav_msgs::msg::Odometry & message)
{
  const auto & position = message.pose.pose.position;
  const auto & orientation = message.pose.pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    return false;
  }
  const double squared_norm = orientation.x * orientation.x +
    orientation.y * orientation.y + orientation.z * orientation.z +
    orientation.w * orientation.w;
  return std::isfinite(squared_norm) && squared_norm > 1.0e-12;
}

inline bool OdometryVelocityIsFinite(const nav_msgs::msg::Odometry & message)
{
  // Navigation only consumes the longitudinal velocity. Leave unrelated
  // twist fields alone: some odometry sources intentionally mark unused
  // components as NaN, and that must not make a usable pose stop the run.
  return std::isfinite(message.twist.twist.linear.x);
}

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__ODOMETRY_INPUT_VALIDATION_HPP_
