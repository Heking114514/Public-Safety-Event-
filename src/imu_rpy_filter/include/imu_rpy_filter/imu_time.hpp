#pragma once

#include <cmath>
#include <cstdint>

namespace imu_rpy_filter {

// IMU samples with a valid timestamp but an unusable interval are dropped
// without poisoning the filter state. Advancing the baseline here lets the
// next regularly timed sample resume the filter instead of being compared to
// the old timestamp forever.
constexpr double kMaxImuDtSeconds = 0.1;

inline bool valid_ros_time_representation(std::int32_t seconds,
                                          std::uint32_t nanoseconds) {
  return seconds >= 0 && nanoseconds < 1000000000U;
}

inline bool update_imu_time_baseline(double stamp, double &last_stamp,
                                     double &dt) {
  if (!std::isfinite(stamp) || stamp <= 0.0) {
    return false;
  }

  dt = stamp - last_stamp;
  // Preserve the monotonic timestamp contract for delayed/out-of-order
  // samples. A large positive gap is recoverable and should rebase, but a
  // backwards clock sample must not move the baseline backwards.
  if (!std::isfinite(dt) || dt <= 0.0) {
    return false;
  }
  last_stamp = stamp;
  return dt <= kMaxImuDtSeconds;
}

}  // namespace imu_rpy_filter
