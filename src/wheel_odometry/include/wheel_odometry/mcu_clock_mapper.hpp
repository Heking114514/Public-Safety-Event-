#pragma once

#include <cstdint>

namespace wheel_odometry
{

// Maps the MCU's wrapping millisecond clock into ROS time. Rate calibration
// starts after five seconds and keeps the original anchor as a long baseline,
// preventing millisecond quantization from biasing consecutive short windows.
class McuClockMapper
{
public:
  int64_t map(uint32_t mcu_time_ms, int64_t reception_time_ns);
  void reset();

private:
  static constexpr uint32_t kHalfRange = 0x80000000U;
  static constexpr uint32_t kCalibrationWindowMs = 5000U;
  static constexpr double kMinRate = 0.999;
  static constexpr double kMaxRate = 1.001;
  static constexpr double kRateBlend = 0.20;

  bool valid_{false};
  uint32_t last_mcu_ms_{0};
  int64_t last_mapped_ns_{0};
  int64_t anchor_reception_ns_{0};
  uint64_t elapsed_mcu_ms_{0};
  uint64_t next_calibration_mcu_ms_{kCalibrationWindowMs};
  double rate_{1.0};
};

}  // namespace wheel_odometry
