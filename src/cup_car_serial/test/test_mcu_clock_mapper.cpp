#include <gtest/gtest.h>

#include "cup_car_serial/mcu_clock_mapper.hpp"

namespace cup_car_serial
{

TEST(McuClockMapper, TracksSlowMcuClockWithoutAccumulatingTransportOffset)
{
  McuClockMapper mapper;
  constexpr int64_t start = 10'000'000'000LL;
  mapper.map(1000U, start);
  int64_t last_mapped = start;
  int64_t mapped = start;
  int64_t reception = start;
  for (int seconds = 5; seconds <= 3600; seconds += 5) {
    const auto mcu_time = static_cast<uint32_t>(
      1000.0 + seconds * 1000.0 * (1.0 - 50.0e-6));
    reception = start + static_cast<int64_t>(seconds) * 1'000'000'000LL;
    mapped = mapper.map(mcu_time, reception);
    EXPECT_GT(mapped, last_mapped);
    EXPECT_LE(mapped, reception);
    last_mapped = mapped;
  }
  const double age = static_cast<double>(reception - mapped) / 1.0e9;
  EXPECT_LT(age, 0.02);
}

TEST(McuClockMapper, ReanchorsAfterCounterRegression)
{
  McuClockMapper mapper;
  constexpr int64_t first = 20'000'000'000LL;
  mapper.map(5000U, first);
  const int64_t second = mapper.map(5100U, first + 100'000'000LL);
  const int64_t restarted = mapper.map(10U, first + 200'000'000LL);
  EXPECT_GE(restarted, second);
  EXPECT_EQ(restarted, first + 200'000'000LL);
}

}  // namespace cup_car_serial
