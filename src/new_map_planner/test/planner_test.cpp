#include "new_map_planner/collision.hpp"
#include "new_map_planner/config.hpp"
#include "new_map_planner/geometry.hpp"
#include "new_map_planner/graph.hpp"
#include "new_map_planner/maneuver.hpp"
#include "new_map_planner/planner.hpp"
#include "new_map_planner/transform.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace {

new_map_planner::PlannerConfig OfficialConfig() {
  const std::string map_file =
      std::string(NEW_MAP_PLANNER_SOURCE_DIR) + "/config/arena.yaml";
  const std::string vehicle_file =
      ament_index_cpp::get_package_share_directory("vehicle_config") +
      "/config/vehicle_physical.yaml";
  return new_map_planner::LoadConfig(map_file, vehicle_file);
}

new_map_planner::PlanRequest HomeRequest(const std::string &mode) {
  new_map_planner::PlanRequest request;
  request.start_map = {{0.0, 0.0}, 0.0};
  request.mode = mode;
  request.return_home = true;
  return request;
}

new_map_planner::Polygon
ArenaRectangleInMap(const new_map_planner::PlannerConfig &config,
                    double minimum_x, double minimum_y, double maximum_x,
                    double maximum_y) {
  new_map_planner::Polygon polygon;
  for (const new_map_planner::Point point :
       {new_map_planner::Point{minimum_x, minimum_y},
        new_map_planner::Point{maximum_x, minimum_y},
        new_map_planner::Point{maximum_x, maximum_y},
        new_map_planner::Point{minimum_x, maximum_y}}) {
    polygon.points.push_back(
        new_map_planner::ArenaToMap(point, config.transform));
  }
  return polygon;
}

} // namespace

TEST(Config, OfficialMapHasExplicitTopology) {
  const auto config = OfficialConfig();
  EXPECT_EQ(config.map_id, "official_arena_v1");
  EXPECT_EQ(config.nodes.size(), 20U);
  EXPECT_EQ(config.lanes.size(), 29U);
  EXPECT_EQ(config.tasks.size(), 12U);
  EXPECT_EQ(config.default_inspection_lanes.size(), 28U);
  EXPECT_EQ(config.rfid_supernodes.size(), 12U);
}

TEST(Transform, LaunchPoseIsMapOrigin) {
  const auto config = OfficialConfig();
  const new_map_planner::Pose map_pose{{0.0, 0.0}, 0.0};
  const auto arena_pose =
      new_map_planner::MapToArena(map_pose, config.transform);
  EXPECT_NEAR(arena_pose.position.x, 1.6, 1.0e-9);
  EXPECT_NEAR(arena_pose.position.y, 4.3, 1.0e-9);
  EXPECT_NEAR(arena_pose.yaw, -0.5 * new_map_planner::kPi, 1.0e-9);
  const auto round_trip =
      new_map_planner::ArenaToMap(arena_pose, config.transform);
  EXPECT_NEAR(round_trip.position.x, 0.0, 1.0e-9);
  EXPECT_NEAR(round_trip.position.y, 0.0, 1.0e-9);
  EXPECT_NEAR(round_trip.yaw, 0.0, 1.0e-9);
}

TEST(Planner, ExactTwelveTargetRouteIsExecutableAndReturnsHome) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  const auto result = planner.plan(HomeRequest("targets"));

  ASSERT_TRUE(result.success)
      << result.message << " deferred=" << result.deferred_target_ids.size();
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.collision_checked);
  EXPECT_EQ(result.visit_order.size(), 12U);
  auto ordered = result.visit_order;
  std::sort(ordered.begin(), ordered.end());
  EXPECT_EQ(std::unique(ordered.begin(), ordered.end()), ordered.end());
  ASSERT_FALSE(result.navigation_path.empty());
  ASSERT_EQ(result.waypoint_supernode_ids.size(), result.navigation_path.size());
  for (int index = 1; index <= 12; ++index) {
    EXPECT_NE(std::find(result.waypoint_supernode_ids.begin(),
                        result.waypoint_supernode_ids.end(),
                        "s" + std::to_string(index)),
              result.waypoint_supernode_ids.end());
  }
  EXPECT_NEAR(result.navigation_path.front().position.x, 0.0, 1.0e-6);
  EXPECT_NEAR(result.navigation_path.front().position.y, 0.0, 1.0e-6);
  EXPECT_NEAR(result.navigation_path.back().position.x, 0.0, 1.0e-6);
  EXPECT_NEAR(result.navigation_path.back().position.y, 0.0, 1.0e-6);
  for (const auto &maneuver : result.maneuvers) {
    EXPECT_TRUE(maneuver.collision_checked);
  }
}

TEST(Planner, RouteStartingAtJunctionCarriesExplicitTurnAuthorization) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = new_map_planner::ArenaToMap(
      {{1.6, 4.1}, -0.5 * new_map_planner::kPi}, config.transform);
  request.mode = "targets";
  request.target_ids = {"12"};
  request.return_home = false;

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.turn_waypoints.empty());
  EXPECT_TRUE(result.turn_waypoints.front());
  ASSERT_FALSE(result.maneuvers.empty());
  EXPECT_EQ(result.maneuvers.front().type,
            new_map_planner::ManeuverType::START_ALIGN);
  EXPECT_TRUE(result.maneuvers.front().collision_checked);
}

TEST(Planner, LayerOneTargetsNextPendingTaskInOrder) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("layer1");
  request.target_ids = {"1", "2", "3"};
  request.allow_partial = true;

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.complete);
  ASSERT_EQ(result.visit_order.size(), 1U);
  EXPECT_EQ(result.visit_order.front(), "1");
  EXPECT_EQ(result.ordinary_nodes_since_task, 0);
}

TEST(Maneuver, LaunchJunctionHasBothValidatedExits) {
  const auto config = OfficialConfig();
  const new_map_planner::RoadGraph graph(config);
  const new_map_planner::CollisionChecker collision(config);
  const new_map_planner::ManeuverLibrary maneuvers(config, collision);
  const int launch = graph.node_index("launch");
  const int junction = graph.node_index("launch_junction");
  const auto &launch_outgoing = graph.outgoing(launch);
  ASSERT_EQ(launch_outgoing.size(), 1U);
  const auto &incoming = graph.directed_lane(launch_outgoing.front());
  ASSERT_EQ(incoming.to, junction);
  int valid_turns = 0;
  for (int index : graph.outgoing(junction)) {
    const auto &outgoing = graph.directed_lane(index);
    if (outgoing.heading == incoming.heading) {
      continue;
    }
    const auto sweep =
        maneuvers.turn_sweep(junction, incoming.heading, outgoing.heading);
    bool all_poses_free = true;
    for (std::size_t pose_index = 0; pose_index < sweep.size(); ++pose_index) {
      if (!maneuvers.turn_pose_is_free(sweep[pose_index], {})) {
        all_poses_free = false;
        ADD_FAILURE() << "blocked exit heading=" << outgoing.heading
                      << " sample=" << pose_index << " pose=("
                      << sweep[pose_index].position.x << ", "
                      << sweep[pose_index].position.y << ", "
                      << sweep[pose_index].yaw << ")";
        break;
      }
    }
    if (all_poses_free && maneuvers.transition_is_valid(
                              junction, incoming.heading, outgoing, {})) {
      ++valid_turns;
    }
  }
  EXPECT_EQ(valid_turns, 2);
}

TEST(Maneuver, TopLeftTaskCanTurnTowardArena) {
  const auto config = OfficialConfig();
  const new_map_planner::RoadGraph graph(config);
  const new_map_planner::CollisionChecker collision(config);
  const new_map_planner::ManeuverLibrary maneuvers(config, collision);
  const int junction = graph.node_index("x1_y4");
  const int top_lane = graph.lane_index("ROAD_TOP_LEFT");
  const int down_lane = graph.lane_index("ROAD_X1_Y3");
  const new_map_planner::DirectedLane *incoming = nullptr;
  const new_map_planner::DirectedLane *outgoing = nullptr;
  for (const auto &directed : graph.graph().directed_lanes) {
    if (directed.lane == top_lane && directed.to == junction) {
      incoming = &directed;
    }
    if (directed.lane == down_lane && directed.from == junction) {
      outgoing = &directed;
    }
  }
  ASSERT_NE(incoming, nullptr);
  ASSERT_NE(outgoing, nullptr);
  const auto sweep =
      maneuvers.turn_sweep(junction, incoming->heading, outgoing->heading);
  for (std::size_t index = 0; index < sweep.size(); ++index) {
    EXPECT_TRUE(maneuvers.turn_pose_is_free(sweep[index], {}))
        << "sample=" << index << " pose=(" << sweep[index].position.x << ", "
        << sweep[index].position.y << ", " << sweep[index].yaw << ")";
  }
  EXPECT_TRUE(maneuvers.transition_is_valid(junction, incoming->heading,
                                            *outgoing, {}));
}

TEST(Maneuver, EveryConfiguredQuarterTurnHasAValidatedSweep) {
  const auto config = OfficialConfig();
  const new_map_planner::RoadGraph graph(config);
  const new_map_planner::CollisionChecker collision(config);
  const new_map_planner::ManeuverLibrary maneuvers(config, collision);
  for (const auto &incoming : graph.graph().directed_lanes) {
    for (int index : graph.outgoing(incoming.to)) {
      const auto &outgoing = graph.directed_lane(index);
      const int turn = new_map_planner::TurnQuarterSteps(
          incoming.heading, outgoing.heading);
      if (turn != 1 && turn != -1) {
        continue;
      }
      EXPECT_TRUE(maneuvers.transition_is_valid(
          incoming.to, incoming.heading, outgoing, {}))
          << "node=" << config.nodes[incoming.to].id
          << " incoming=" << config.lanes[incoming.lane].id
          << " outgoing=" << config.lanes[outgoing.lane].id;
    }
  }
}

TEST(Planner, CompleteRoadCoverageUsesExplicitManeuvers) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  const auto result = planner.plan(HomeRequest("coverage"));

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.collision_checked);
  EXPECT_EQ(result.covered_lane_ids.size(),
            config.default_inspection_lanes.size());
  EXPECT_TRUE(result.deferred_lane_ids.empty());
  EXPECT_TRUE(std::any_of(
      result.maneuvers.begin(), result.maneuvers.end(),
      [](const new_map_planner::Maneuver &maneuver) {
        return maneuver.type == new_map_planner::ManeuverType::TURN_LEFT ||
               maneuver.type == new_map_planner::ManeuverType::TURN_RIGHT;
      }));
  for (std::size_t index = 0; index < result.traversals.size(); ++index) {
    EXPECT_FALSE(result.traversals[index].reverse)
        << "index=" << index << " lane="
        << result.traversals[index].directed_lane;
  }
  EXPECT_TRUE(std::none_of(
      result.maneuvers.begin(), result.maneuvers.end(),
      [](const new_map_planner::Maneuver &maneuver) {
        return maneuver.type == new_map_planner::ManeuverType::REVERSE;
      }));
}

TEST(Planner, CoverageSearchIsDeterministic) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  const auto request = HomeRequest("coverage");

  const auto first = planner.plan(request);
  const auto second = planner.plan(request);

  ASSERT_TRUE(first.success) << first.message;
  ASSERT_TRUE(second.success) << second.message;
  ASSERT_EQ(first.traversals.size(), second.traversals.size());
  for (std::size_t index = 0; index < first.traversals.size(); ++index) {
    EXPECT_EQ(first.traversals[index].directed_lane,
              second.traversals[index].directed_lane);
    EXPECT_EQ(first.traversals[index].reverse,
              second.traversals[index].reverse);
  }
}

TEST(Planner, RunsTaskTunnelAndRoadStagesFromMeasuredEndPoses) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);

  std::vector<std::string> remaining_targets = config.default_targets;
  std::vector<std::string> layer1_covered;
  auto current_pose = HomeRequest("layer1").start_map;
  int ordinary_count = 1;
  for (std::size_t attempt = 0; !remaining_targets.empty() && attempt < 16;
       ++attempt) {
    new_map_planner::PlanRequest layer1_request;
    layer1_request.start_map = current_pose;
    layer1_request.mode = "layer1";
    layer1_request.target_ids = remaining_targets;
    layer1_request.allow_partial = true;
    layer1_request.ordinary_nodes_since_task = ordinary_count;
    const auto layer1 = planner.plan(layer1_request);
    ASSERT_TRUE(layer1.success) << layer1.message;
    ASSERT_EQ(layer1.visit_order.size(), 1U);
    EXPECT_EQ(layer1.visit_order.front(), remaining_targets.front());
    ASSERT_FALSE(layer1.navigation_path.empty());
    int path_count = ordinary_count;
    for (const auto &traversal : layer1.traversals) {
      if (traversal.directed_lane < 0 || traversal.reverse) {
        continue;
      }
      const int node =
          planner.road_graph().directed_lane(traversal.directed_lane).to;
      const bool task_node =
          std::any_of(config.tasks.begin(), config.tasks.end(),
                      [node](const new_map_planner::Task &task) {
                        return task.node == node;
                      });
      path_count = task_node ? 0 : path_count + 1;
      EXPECT_LE(path_count, 2);
    }
    current_pose = layer1.navigation_path.back();
    ordinary_count = layer1.ordinary_nodes_since_task;
    layer1_covered.insert(layer1_covered.end(), layer1.covered_lane_ids.begin(),
                          layer1.covered_lane_ids.end());
    remaining_targets.erase(std::remove(remaining_targets.begin(),
                                        remaining_targets.end(),
                                        layer1.visit_order.front()),
                            remaining_targets.end());
    EXPECT_EQ(layer1.complete, remaining_targets.empty());
  }
  ASSERT_TRUE(remaining_targets.empty());
  EXPECT_EQ(config.default_targets.size(), 12U);
  EXPECT_GT(std::hypot(current_pose.position.x, current_pose.position.y), 0.1);

  new_map_planner::PlanRequest layer2_request;
  layer2_request.start_map = current_pose;
  layer2_request.mode = "layer2";
  layer2_request.ordinary_nodes_since_task = ordinary_count;
  for (const std::string &id : layer1_covered) {
    const int lane = planner.road_graph().lane_index(id);
    if (!config.lanes[lane].tunnel) {
      layer2_request.completed_lane_ids.push_back(id);
    }
  }
  layer2_request.allow_partial = false;
  const auto layer2 = planner.plan(layer2_request);
  ASSERT_TRUE(layer2.success) << layer2.message;
  ASSERT_TRUE(layer2.complete);
  for (const auto &lane : config.lanes) {
    if (lane.inspection && lane.tunnel) {
      EXPECT_NE(std::find(layer2.covered_lane_ids.begin(),
                          layer2.covered_lane_ids.end(), lane.id),
                layer2.covered_lane_ids.end())
          << lane.id;
    }
  }
  ASSERT_FALSE(layer2.navigation_path.empty());

  new_map_planner::PlanRequest layer3_request;
  layer3_request.start_map = layer2.navigation_path.back();
  layer3_request.mode = "layer3";
  layer3_request.ordinary_nodes_since_task = layer2.ordinary_nodes_since_task;
  layer3_request.completed_lane_ids = layer1_covered;
  layer3_request.completed_lane_ids.insert(
      layer3_request.completed_lane_ids.end(), layer2.covered_lane_ids.begin(),
      layer2.covered_lane_ids.end());
  layer3_request.allow_partial = false;
  const auto layer3 = planner.plan(layer3_request);
  ASSERT_TRUE(layer3.success) << layer3.message;
  ASSERT_TRUE(layer3.complete);
  ASSERT_FALSE(layer3.navigation_path.empty());
  EXPECT_NEAR(layer3.navigation_path.back().position.x, 0.0, 1.0e-6);
  EXPECT_NEAR(layer3.navigation_path.back().position.y, 0.0, 1.0e-6);
  for (const auto &traversal : layer3.traversals) {
    if (traversal.directed_lane < 0) {
      continue;
    }
    const auto &directed =
        planner.road_graph().directed_lane(traversal.directed_lane);
    EXPECT_FALSE(config.lanes[directed.lane].tunnel);
  }

  std::vector<std::string> covered = layer1_covered;
  covered.insert(covered.end(), layer2.covered_lane_ids.begin(),
                 layer2.covered_lane_ids.end());
  covered.insert(covered.end(), layer3.covered_lane_ids.begin(),
                 layer3.covered_lane_ids.end());
  for (const std::string &lane : config.default_inspection_lanes) {
    EXPECT_NE(std::find(covered.begin(), covered.end(), lane), covered.end())
        << lane;
  }
}

TEST(Planner, RejectsLaneFromTheWrongStage) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("layer2");
  request.required_lane_ids = {"ROAD_TOP_LEFT"};

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("non-tunnel stage"), std::string::npos);
}

TEST(Planner, StartOutsideRoadIsRejectedWithoutProjection) {
  auto config = OfficialConfig();
  config.free_regions = {{0.0, 0.0, config.width, config.height}};
  config.static_obstacles.clear();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("targets");
  request.start_map = new_map_planner::ArenaToMap(
      {{0.5, 0.5}, 0.0}, config.transform);
  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason,
            new_map_planner::PlanFailureReason::START_NOT_ON_ROAD);
}

TEST(Planner, StartFootprintCollisionHasStableFailureReason) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("targets");
  request.dynamic_obstacles_map.push_back(
      ArenaRectangleInMap(config, 1.50, 4.20, 1.70, 4.40));

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason,
            new_map_planner::PlanFailureReason::START_POSE_COLLISION);
}

TEST(Planner, StartHeadingMismatchHasStableFailureReason) {
  auto config = OfficialConfig();
  config.free_regions = {{0.0, 0.0, config.width, config.height}};
  config.static_obstacles.clear();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("targets");
  request.start_map = new_map_planner::ArenaToMap(
      {{1.1, 1.1}, -0.25 * new_map_planner::kPi}, config.transform);

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason,
            new_map_planner::PlanFailureReason::START_HEADING_MISMATCH);
}

TEST(Planner, BlockedForwardAnchorHasStableFailureReason) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("targets");
  request.start_map = new_map_planner::ArenaToMap(
      {{1.6, 4.22}, -0.5 * new_map_planner::kPi}, config.transform);
  request.dynamic_obstacles_map.push_back(
      ArenaRectangleInMap(config, 1.55, 4.105, 1.65, 4.125));

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason,
            new_map_planner::PlanFailureReason::START_FORWARD_BLOCKED);
}

TEST(Planner, ReverseOnlyAnchorRequestsRecoveryInsteadOfPlanningReverse) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("targets");
  request.start_map = new_map_planner::ArenaToMap(
      {{1.6, 4.31}, 0.5 * new_map_planner::kPi}, config.transform);

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason,
            new_map_planner::PlanFailureReason::START_REQUIRES_REVERSE);
  EXPECT_TRUE(result.traversals.empty());
}

TEST(Planner, NearTopTaskStartKeepsVehicleHeadingDuringSnap) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = {{0.194, -0.462}, -88.3 * new_map_planner::kPi / 180.0};
  request.mode = "layer1";
  request.target_ids = {"2"};
  request.allow_partial = true;
  request.return_home = false;

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.traversals.empty());
  EXPECT_TRUE(result.traversals.front().partial);
  ASSERT_FALSE(result.arena_path.empty());
  EXPECT_NEAR(
      new_map_planner::NormalizeAngle(result.arena_path.front().yaw -
                                      (-178.3 * new_map_planner::kPi / 180.0)),
      0.0, 0.20);
}

TEST(Planner, RfidCorrectionResidualIsCapturedByTaskNode) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = {
      {0.200041, -0.500001},
      -88.389 * new_map_planner::kPi / 180.0};
  request.mode = "layer1";
  request.target_ids = {"2"};
  request.allow_partial = true;
  request.return_home = false;

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.traversals.empty());
  EXPECT_TRUE(result.traversals.front().partial);
  EXPECT_LT(
      new_map_planner::Distance(
          result.traversals.front().start, result.traversals.front().end),
      config.node_capture_distance);
}

TEST(Planner, RejectsReverseOnlySnapToJunctionAfterObstacleRetreat) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("layer1");
  request.start_map = {{1.2, -0.55}, -0.5 * new_map_planner::kPi};
  request.return_home = false;
  request.allow_partial = true;
  request.dynamic_obstacles_map.push_back(new_map_planner::Polygon{
      {{1.1, -1.093}, {1.3, -1.093}, {1.3, -0.993}, {1.1, -0.993}}});

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.message.empty());
}

TEST(Planner, RejectsPartialCoverageRouteWithoutRequiredLaneProgress) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = {{1.2391626162350184, -1.5005002196722133},
                       new_map_planner::kPi};
  request.mode = "layer2";
  request.required_lane_ids = {"TUNNEL_1", "TUNNEL_2", "TUNNEL_3", "TUNNEL_4"};
  request.allow_partial = true;
  request.dynamic_obstacles_map.push_back(new_map_planner::Polygon{
      {{1.1, -1.093}, {1.3, -1.093}, {1.3, -0.993}, {1.1, -0.993}}});

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.deferred_lane_ids.size(), 4U);
}

TEST(Planner, TargetRouteDoesNotUseRecoveryReverse) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = {{1.239, -1.500}, -179.9 * new_map_planner::kPi / 180.0};
  request.mode = "layer1";
  request.target_ids = {"4"};
  request.allow_partial = true;
  request.return_home = false;
  request.dynamic_obstacles_map.push_back(new_map_planner::Polygon{
      {{1.10, -1.093}, {1.30, -1.093}, {1.30, -0.993},
       {1.10, -0.993}}});

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason,
            new_map_planner::PlanFailureReason::NO_FEASIBLE_ROUTE);
  EXPECT_FALSE(std::any_of(
      result.traversals.begin(), result.traversals.end(),
      [](const new_map_planner::Traversal &traversal) {
        return traversal.reverse;
      }));
}

TEST(Planner, PlansTunnelStageFromForwardAlignedRoad) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = {{2.2, -1.5}, new_map_planner::kPi};
  request.mode = "layer2";
  request.allow_partial = true;
  request.dynamic_obstacles_map.push_back(new_map_planner::Polygon{
      {{1.1, -1.093}, {1.3, -1.093}, {1.3, -0.993}, {1.1, -0.993}}});

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(std::any_of(
      result.traversals.begin(), result.traversals.end(),
      [](const new_map_planner::Traversal &traversal) {
        return traversal.reverse;
      }));
  EXPECT_TRUE(std::any_of(
      result.covered_lane_ids.begin(), result.covered_lane_ids.end(),
      [](const std::string &id) { return id.find("TUNNEL_") == 0; }));
}

TEST(Planner, CoverageRouteDoesNotUseRecoveryReverse) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  new_map_planner::PlanRequest request;
  request.start_map = {{4.200369974461137, -0.46090603325788965},
                       -0.5 * new_map_planner::kPi};
  request.mode = "layer3";
  request.completed_lane_ids = {
      "ROAD_BOTTOM_LEFT", "ROAD_BOTTOM_RIGHT", "ROAD_TOP_LEFT",
      "ROAD_TOP_RIGHT",   "ROAD_X0_Y0",        "ROAD_X0_Y1",
      "ROAD_X0_Y2",       "ROAD_X1_Y1",        "ROAD_X1_Y2",
      "ROAD_X1_Y3",       "ROAD_X2_Y0",        "ROAD_X2_Y2",
      "ROAD_X2_Y3",       "ROAD_X3_Y0",        "ROAD_X3_Y1",
      "ROAD_X3_Y2",       "ROAD_Y2_LEFT",      "ROAD_Y3_RIGHT",
      "TUNNEL_1",         "TUNNEL_2",          "TUNNEL_3",
      "TUNNEL_4"};
  request.allow_partial = true;
  request.return_home = true;
  request.dynamic_obstacles_map.push_back(new_map_planner::Polygon{
      {{1.1, -1.093}, {1.3, -1.093}, {1.3, -0.993}, {1.1, -0.993}}});

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(std::any_of(
      result.traversals.begin(), result.traversals.end(),
      [](const new_map_planner::Traversal &traversal) {
        return traversal.reverse;
      }));
  EXPECT_EQ(std::find(result.reverse_arrival_turn_waypoints.begin(),
                      result.reverse_arrival_turn_waypoints.end(), true),
            result.reverse_arrival_turn_waypoints.end());
}

TEST(Diagnostics, FailureNamesAreStable) {
  EXPECT_STREQ(new_map_planner::PlanFailureName(
                   new_map_planner::PlanFailureReason::START_POSE_COLLISION),
               "START_POSE_COLLISION");
  EXPECT_STREQ(new_map_planner::PlanFailureName(
                   new_map_planner::PlanFailureReason::START_REQUIRES_REVERSE),
               "START_REQUIRES_REVERSE");
  EXPECT_STREQ(new_map_planner::PlanFailureName(
                   new_map_planner::PlanFailureReason::NO_FEASIBLE_ROUTE),
               "NO_FEASIBLE_ROUTE");
}

TEST(Planner, DynamicBlockageDefersOnlyUnreachableCoverage) {
  const auto config = OfficialConfig();
  new_map_planner::MapPlanner planner(config);
  auto request = HomeRequest("coverage");
  request.allow_partial = true;
  request.return_home = false;
  request.dynamic_obstacles_map.push_back(
      ArenaRectangleInMap(config, 1.55, 1.06, 1.65, 1.14));

  const auto begin = std::chrono::steady_clock::now();
  const auto result = planner.plan(request);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - begin)
                                .count();

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.complete);
  EXPECT_TRUE(result.collision_checked);
  EXPECT_NE(std::find(result.deferred_lane_ids.begin(),
                      result.deferred_lane_ids.end(), "TUNNEL_1"),
            result.deferred_lane_ids.end());
  EXPECT_TRUE(std::none_of(
      result.traversals.begin(), result.traversals.end(),
      [](const new_map_planner::Traversal &traversal) {
        return traversal.reverse;
      }));

  new_map_planner::PlanRequest return_request;
  return_request.start_map = result.navigation_path.back();
  return_request.mode = "coverage";
  return_request.completed_lane_ids = config.default_inspection_lanes;
  return_request.dynamic_obstacles_map = request.dynamic_obstacles_map;
  return_request.return_home = true;
  const auto return_route = planner.plan(return_request);
  ASSERT_TRUE(return_route.success) << return_route.message;
  EXPECT_TRUE(return_route.complete);
  EXPECT_LT(elapsed_ms, 2000.0);
}
