#include <cmath>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "navigation_mission_manager/events.hpp"
#include "navigation_mission_manager/mission.hpp"
#include "navigation_mission_manager/obstacles.hpp"
#include "navigation_mission_manager/recovery_session.hpp"

namespace nmm = navigation_mission_manager;

namespace
{

nmm::MissionLayout layout()
{
  return nmm::MissionLayout::from_file(
    std::string(MISSION_MANAGER_SOURCE_DIR) + "/../new_map_planner/config/arena.yaml");
}

nmm::EventData event(
  const std::string & id = "a", double x = 1.0, double confidence = 0.9)
{
  return {id, "camera", "fire", confidence, x, 2.0, 0.0, "", "", 0};
}

}  // namespace

TEST(MissionModel, AdvancesOnlyFromConfirmedProgress)
{
  nmm::StagedMission mission(layout());
  mission.start();
  const auto layer1 = mission.plan_spec();
  EXPECT_EQ(layer1.mode, "layer1");
  EXPECT_GT(mission.commit(layer1, layer1.target_ids, {"ROAD_TOP_LEFT"}, true), 0U);

  const auto layer2 = mission.plan_spec();
  EXPECT_EQ(layer2.mode, "layer2");
  EXPECT_GT(mission.commit(layer2, {}, layer2.required_lane_ids, true), 0U);

  const auto layer3 = mission.plan_spec();
  EXPECT_EQ(layer3.mode, "layer3");
  mission.commit(layer3, {}, layer3.required_lane_ids, true);
  EXPECT_TRUE(mission.finished());
}

TEST(MissionModel, PartialRouteKeepsSameStage)
{
  nmm::StagedMission mission(layout());
  mission.start({"1", "2"});
  const auto spec = mission.plan_spec();
  mission.commit(spec, {"1"}, {"ROAD_TOP_LEFT"}, false);
  EXPECT_EQ(mission.plan_spec().target_ids, std::vector<std::string>({"2"}));
}

TEST(MissionModel, TaskCoordinatesUseNavigationMapFrame)
{
  const auto arena = layout();
  EXPECT_NEAR(arena.task_positions.at("1").first, 0.2, 1.0e-12);
  EXPECT_NEAR(arena.task_positions.at("1").second, -0.5, 1.0e-12);
  EXPECT_NEAR(arena.task_positions.at("5").first, 4.2, 1.0e-12);
  EXPECT_NEAR(arena.task_positions.at("5").second, -1.5, 1.0e-12);
}

TEST(MissionModel, RfidSupernodesUseStablePhysicalPointIds)
{
  const auto arena = layout();
  ASSERT_EQ(arena.supernode_task_ids.at("s1"), "1");
  EXPECT_EQ(arena.supernode_task_ids.size(), 12U);
}

TEST(EventRegistry, RejectsWeakAndDuplicateCandidates)
{
  nmm::EventRegistry registry(0.7, 0.35);
  EXPECT_FALSE(registry.offer(event("weak", 1.0, 0.6)));
  EXPECT_TRUE(registry.offer(event()));
  EXPECT_FALSE(registry.offer(event()));
  EXPECT_FALSE(registry.offer(event("b", 1.2)));
  EXPECT_TRUE(registry.offer(event("c", 1.5)));
  EXPECT_EQ(registry.mark_attempt().attempts, 1);
  EXPECT_EQ(registry.mark_reported().candidate_id, "a");
}

TEST(Obstacles, ProjectsAndMergesObservations)
{
  const auto polygon = nmm::front_obstacle_polygon(
    {1.0, 2.0, 0.0}, 0.5, 0.1, 0.2, 0.1);
  const auto center = nmm::polygon_center(polygon);
  EXPECT_NEAR(center.first, 1.65, 1.0e-12);
  EXPECT_NEAR(center.second, 2.0, 1.0e-12);

  std::vector<nmm::Polygon2d> obstacles{
    {{0.0, 0.0}, {0.1, 0.0}, {0.1, 0.1}, {0.0, 0.1}}};
  const nmm::Polygon2d replacement{
    {0.1, 0.0}, {0.2, 0.0}, {0.2, 0.1}, {0.1, 0.1}};
  nmm::merge_obstacle(obstacles, replacement, 0.2);
  EXPECT_EQ(obstacles, std::vector<nmm::Polygon2d>({replacement}));
}

TEST(Obstacles, SnapshotRevisionChangesOnlyWhenLaneSetChanges)
{
  std::set<std::string> lanes;
  EXPECT_TRUE(lanes.empty());
  lanes.insert("L1");
  EXPECT_EQ(lanes.count("L1"), 1U);
  EXPECT_EQ(lanes.count("L2"), 0U);
}
TEST(Obstacles, RetentionRequiresTimestampAndPositiveDuration)
{
  EXPECT_FALSE(nmm::obstacle_memory_expired(std::nullopt, 20.0, 10.0));
  EXPECT_FALSE(nmm::obstacle_memory_expired(10.0, 20.0, 0.0));
  EXPECT_FALSE(nmm::obstacle_memory_expired(10.0, 19.9, 10.0));
  EXPECT_TRUE(nmm::obstacle_memory_expired(10.0, 20.0, 10.0));
}

TEST(RecoveryProtocol, RejectsRetiredAndMismatchedControllerMessages)
{
  nmm::RecoverySession session;
  EXPECT_FALSE(session.begin(0, 1, 1));
  EXPECT_FALSE(session.begin(2, 0, 1));
  ASSERT_TRUE(session.begin(2, 1, 8));
  const auto generation = session.generation;
  EXPECT_TRUE(session.matches(2, 1, 8));
  EXPECT_FALSE(session.matches(2, 1, 9));
  EXPECT_FALSE(session.matches(3, 1, 8));
  EXPECT_FALSE(session.begin(2, 2, 9));
  session.reset();
  EXPECT_GT(session.generation, generation);
  EXPECT_FALSE(session.matches(2, 1, 8));
  EXPECT_FALSE(session.begin(2, 1, 8));
  EXPECT_TRUE(session.begin(3, 2, 9));
}

TEST(RecoveryProtocol, BlocksBorrowingOrExitingEitherDirectionOfPhysicalLane)
{
  const std::set<std::string> blocked{"ROAD_TOP_LEFT"};
  EXPECT_TRUE(nmm::recovery_borrows_blocked_lane({"ROAD_TOP_LEFT"}, "L2", blocked));
  EXPECT_TRUE(nmm::recovery_borrows_blocked_lane({}, "ROAD_TOP_LEFT", blocked));
  EXPECT_FALSE(nmm::recovery_borrows_blocked_lane({"L1"}, "L2", blocked));
}

TEST(MissionModel, BlockedRoadIsSkippedWithoutCoverageOrTaskTunnelCompletion)
{
  nmm::StagedMission mission(layout());
  mission.start({"1"});
  const auto road = mission.layout().road_lane_ids.front();
  const auto tunnel = mission.layout().tunnel_lane_ids.front();
  mission.skip_blocked_roads({road, tunnel});
  EXPECT_EQ(mission.skipped_roads(), std::set<std::string>({road}));
  EXPECT_EQ(mission.confirmed_lanes().count(road), 0U);
  EXPECT_EQ(mission.progress_count(), 0U);
  EXPECT_EQ(mission.remaining_tasks(), std::vector<std::string>({"1"}));
  EXPECT_EQ(mission.remaining_tunnels().size(), mission.layout().tunnel_lane_ids.size());
  EXPECT_EQ(mission.remaining_roads().size(), mission.layout().road_lane_ids.size() - 1);
  const auto tasks = mission.plan_spec();
  mission.commit(tasks, {"1"}, {}, false);
  EXPECT_EQ(mission.current_stage(), "layer2");
  EXPECT_EQ(mission.remaining_tunnels().size(), mission.layout().tunnel_lane_ids.size());
  mission.reset();
  EXPECT_TRUE(mission.skipped_roads().empty());
}
