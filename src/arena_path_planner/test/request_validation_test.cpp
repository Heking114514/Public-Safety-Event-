#include "arena_path_planner/request_validation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace arena_path_planner
{
namespace
{

TEST(RequestValidation, NormalizesQuaternionBeforeComputingYaw)
{
  double yaw = 0.0;
  std::string reason;
  ASSERT_TRUE(QuaternionToYaw(0.0, 0.0, 2.0, 2.0, yaw, &reason));
  EXPECT_NEAR(yaw, 1.5707963267948966, 1.0e-12);
  EXPECT_TRUE(reason.empty());
}

TEST(RequestValidation, RejectsInvalidQuaternions)
{
  double yaw = 0.0;
  std::string reason;
  EXPECT_FALSE(QuaternionToYaw(0.0, 0.0, 0.0, 0.0, yaw, &reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_FALSE(QuaternionToYaw(
    0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 1.0, yaw, &reason));
  EXPECT_FALSE(QuaternionToYaw(
    0.0, 0.0, std::numeric_limits<double>::infinity(), 1.0, yaw, &reason));
}

TEST(RequestValidation, RejectsNonFiniteAndExtremeArenaCoordinates)
{
  PlannerConfig config;
  config.width = 6.0;
  config.height = 6.0;
  std::string reason;
  EXPECT_TRUE(PointIsInsideArena({2.0, 3.0}, config, &reason));
  EXPECT_FALSE(PointIsInsideArena({1.0e300, 3.0}, config, &reason));
  EXPECT_FALSE(PointIsInsideArena(
    {std::numeric_limits<double>::quiet_NaN(), 3.0}, config, &reason));
  EXPECT_FALSE(PointIsInsideArena(
    {2.0, std::numeric_limits<double>::infinity()}, config, &reason));
}

}  // namespace
}  // namespace arena_path_planner
