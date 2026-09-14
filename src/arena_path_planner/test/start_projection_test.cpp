#include "arena_path_planner/planner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace arena_path_planner
{
namespace
{

std::string ConfigPath()
{
  return ament_index_cpp::get_package_share_directory("arena_path_planner") +
         "/config/arena_map.yaml";
}

TEST(StartProjection, RecoversRecordedLaunchLaneDriftWithinEightCentimetres)
{
  const ArenaPlanner planner(ArenaPlanner::LoadConfig(ConfigPath()));
  const Pose recorded_start{{1.58032, 4.25334}, -0.5 * std::acos(-1.0)};
  EXPECT_TRUE(planner.IsFree(recorded_start.position));
  ASSERT_FALSE(planner.PoseIsFree(recorded_start.position, recorded_start.yaw));

  Pose projected;
  ASSERT_TRUE(planner.ProjectToNearestFreePose(recorded_start, 0.08, projected));
  EXPECT_TRUE(planner.IsFree(projected.position));
  EXPECT_TRUE(planner.PoseIsFree(projected.position, projected.yaw));
  EXPECT_LE(Distance(recorded_start.position, projected.position), 0.08);
  EXPECT_GT(Distance(recorded_start.position, projected.position), 0.0);
  EXPECT_DOUBLE_EQ(projected.yaw, recorded_start.yaw);
}

TEST(StartProjection, MillimetreLaunchLaneOffsetDoesNotCreateFakeRoadTurn)
{
  const ArenaPlanner planner(ArenaPlanner::LoadConfig(ConfigPath()));
  const Pose live_start{{1.60256, 4.25344}, -1.54977};

  const PlanResult result = planner.Plan(
    live_start, planner.config().default_targets,
    planner.config().default_labels, "layer1");

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GE(result.points.size(), 2U);
  EXPECT_NEAR(result.points.front().x, live_start.position.x, 1.0e-12);
  EXPECT_NEAR(result.points.front().y, live_start.position.y, 1.0e-12);
  EXPECT_GT(Distance(result.points[0], result.points[1]), 0.10);
  EXPECT_TRUE(planner.RouteTurnsAreAllowed(result.points));
}

TEST(StartProjection, DoesNotHideLargePoseErrors)
{
  const ArenaPlanner planner(ArenaPlanner::LoadConfig(ConfigPath()));
  const Pose inside_block{{1.60, 3.60}, -0.5 * std::acos(-1.0)};
  Pose projected;
  EXPECT_FALSE(planner.ProjectToNearestFreePose(inside_block, 0.08, projected));
  EXPECT_FALSE(planner.ProjectToNearestFreePose(
      {{std::numeric_limits<double>::quiet_NaN(), 0.0}, 0.0}, 0.08, projected));
}

}  // namespace
}  // namespace arena_path_planner
