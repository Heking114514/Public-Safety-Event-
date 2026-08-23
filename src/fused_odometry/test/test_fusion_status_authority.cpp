#include <chrono>

#include "fused_odometry/fusion_status_authority.hpp"
#include "gtest/gtest.h"

TEST(FusionStatusAuthority, VisualStatesAreSharedAcrossDownstreamNodes)
{
  using Authority = fused_odometry::FusionStatusAuthority;
  const auto start = Authority::Clock::now();
  Authority authority;

  authority.update("FULL", start);
  EXPECT_TRUE(authority.visual_correction_allowed(
    start + std::chrono::milliseconds(100), 0.5));

  authority.update("DEGRADED_NO_IMU", start);
  EXPECT_TRUE(authority.visual_correction_allowed(
    start + std::chrono::milliseconds(100), 0.5));

  authority.update("DEGRADED_NO_VISION", start);
  EXPECT_FALSE(authority.visual_correction_allowed(
    start + std::chrono::milliseconds(100), 0.5));
}

TEST(FusionStatusAuthority, StaleOrFutureStatusIsNotAuthoritative)
{
  using Authority = fused_odometry::FusionStatusAuthority;
  const auto start = Authority::Clock::now();
  Authority authority;
  authority.update("FULL", start);

  EXPECT_FALSE(authority.visual_correction_allowed(
    start + std::chrono::seconds(1), 0.5));
  EXPECT_FALSE(authority.visual_correction_allowed(
    start - std::chrono::milliseconds(1), 0.5));
}
