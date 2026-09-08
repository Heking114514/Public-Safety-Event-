#include "wheel_odometry/mcu_clock_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace wheel_odometry
{

void McuClockMapper::reset()
{
  valid_ = false;
  elapsed_mcu_ms_ = 0U;
  next_calibration_mcu_ms_ = kCalibrationWindowMs;
  rate_ = 1.0;
}

int64_t McuClockMapper::map(uint32_t mcu_time_ms, int64_t reception_time_ns)
{
  if (!valid_) {
    valid_ = true;
    last_mcu_ms_ = mcu_time_ms;
    last_mapped_ns_ = reception_time_ns;
    anchor_reception_ns_ = reception_time_ns;
    elapsed_mcu_ms_ = 0U;
    next_calibration_mcu_ms_ = kCalibrationWindowMs;
    return reception_time_ns;
  }

  const uint32_t elapsed = mcu_time_ms - last_mcu_ms_;
  if (elapsed == 0U || elapsed >= kHalfRange) {
    // A non-wrap regression means the MCU restarted or the stream was reset.
    valid_ = false;
    return map(mcu_time_ms, reception_time_ns);
  }

  elapsed_mcu_ms_ += elapsed;
  if (elapsed_mcu_ms_ >= next_calibration_mcu_ms_) {
    const double ros_elapsed_ns = static_cast<double>(
      reception_time_ns - anchor_reception_ns_);
    if (std::isfinite(ros_elapsed_ns) && ros_elapsed_ns > 0.0) {
      const double observed_rate = ros_elapsed_ns /
        (static_cast<double>(elapsed_mcu_ms_) * 1.0e6);
      if (std::isfinite(observed_rate)) {
        rate_ = std::clamp(
          (1.0 - kRateBlend) * rate_ + kRateBlend * observed_rate,
          kMinRate, kMaxRate);
      }
    }
    next_calibration_mcu_ms_ = elapsed_mcu_ms_ + kCalibrationWindowMs;
  }

  const int64_t mapped_delta_ns = static_cast<int64_t>(
    static_cast<double>(elapsed) * 1.0e6 * rate_);
  int64_t mapped_ns = last_mapped_ns_ + mapped_delta_ns;
  if (mapped_ns > reception_time_ns) {
    mapped_ns = reception_time_ns;
  }
  last_mcu_ms_ = mcu_time_ms;
  last_mapped_ns_ = mapped_ns;
  return mapped_ns;
}

}  // namespace wheel_odometry
