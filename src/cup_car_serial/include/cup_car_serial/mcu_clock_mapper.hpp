#ifndef CUP_CAR_SERIAL__MCU_CLOCK_MAPPER_HPP_
#define CUP_CAR_SERIAL__MCU_CLOCK_MAPPER_HPP_

#include <cstdint>

namespace cup_car_serial
{

// Maps the wrapping MCU millisecond clock into the ROS clock at reception.
// The mapped stamp follows the MCU cadence, while a slow rate calibration
// removes long-run clock drift without following serial transport jitter.
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
  uint32_t last_mcu_time_ms_{0};
  int64_t last_mapped_time_ns_{0};
  int64_t anchor_reception_time_ns_{0};
  uint64_t elapsed_mcu_time_ms_{0};
  uint64_t next_calibration_time_ms_{kCalibrationWindowMs};
  double rate_{1.0};
};

}  // namespace cup_car_serial

#endif  // CUP_CAR_SERIAL__MCU_CLOCK_MAPPER_HPP_
