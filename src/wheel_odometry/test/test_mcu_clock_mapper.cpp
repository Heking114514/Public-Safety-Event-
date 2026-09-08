#include "gtest/gtest.h"

#include "wheel_odometry/mcu_clock_mapper.hpp"

namespace wheel_odometry
{

TEST(McuClockMapper, TracksSlowMcuClockWithoutAccumulatingStaleOffset)
{
  McuClockMapper mapper;
  constexpr int64_t start = 10'000'000'000LL;
  mapper.map(1000, start);
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

TEST(McuClockMapper, ReanchorsAfterMcuClockRegression)
{
  McuClockMapper mapper;
  constexpr int64_t first = 20'000'000'000LL;
  mapper.map(5000, first);
  const int64_t second = mapper.map(5100, first + 100'000'000LL);
  const int64_t restarted = mapper.map(10, first + 200'000'000LL);
  EXPECT_GE(restarted, second);
  EXPECT_EQ(restarted, first + 200'000'000LL);
}

}  // namespace wheel_odometry
