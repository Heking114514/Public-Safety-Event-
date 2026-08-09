#include <gtest/gtest.h>

#include <limits>

#include "visual_navigation/rate_limiter.hpp"

TEST(RateLimiter, LimitsAccelerationAndDeceleration)
{
  EXPECT_DOUBLE_EQ(visual_navigation::LimitRate(1.0, 0.0, 0.4, 0.8, 0.5), 0.2);
  EXPECT_DOUBLE_EQ(visual_navigation::LimitRate(0.0, 1.0, 0.4, 0.8, 0.5), 0.6);
}

TEST(RateLimiter, HandlesSignedAngularCommands)
{
  EXPECT_DOUBLE_EQ(visual_navigation::LimitRate(-1.0, 0.5, 2.0, 2.0, 0.25), 0.0);
  EXPECT_DOUBLE_EQ(visual_navigation::LimitRate(1.0, -0.5, 2.0, 2.0, 0.25), 0.0);
}

TEST(RateLimiter, RejectsInvalidTimeStep)
{
  EXPECT_DOUBLE_EQ(visual_navigation::LimitRate(1.0, 0.2, 1.0, 1.0, 0.0), 0.2);
  EXPECT_DOUBLE_EQ(
    visual_navigation::LimitRate(
      std::numeric_limits<double>::quiet_NaN(), 0.2, 1.0, 1.0, 0.1),
    0.2);
}
