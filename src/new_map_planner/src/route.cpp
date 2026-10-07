#include "new_map_planner/route.hpp"

#include "new_map_planner/geometry.hpp"
#include "new_map_planner/transform.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace new_map_planner {

RouteCompiler::RouteCompiler(const PlannerConfig &config,
                             const RoadGraph &graph,
                             const CollisionChecker &collision,
                             const ManeuverLibrary &maneuvers)
    : config_(config), graph_(graph), collision_(collision),
      maneuvers_(maneuvers) {}

bool RouteCompiler::compile(const Pose &start_arena,
                            const std::vector<Traversal> &traversals,
                            const std::vector<Polygon> &dynamic_obstacles,
                            PlanResult &result) const {
  result.arena_path.clear();
  result.navigation_path.clear();
  result.turn_waypoints.clear();
  result.reverse_arrival_turn_waypoints.clear();
  result.maneuvers.clear();
  result.length = 0.0;
  result.collision_checked = false;
  if (traversals.empty()) {
    result.message = "route contains no executable movement";
    return false;
  }
  if (Distance(start_arena.position, traversals.front().start) > 1.0e-6) {
    result.message = "route does not start at the requested vehicle pose";
    return false;
  }

  result.arena_path.push_back(start_arena);
  result.turn_waypoints.push_back(false);
  result.reverse_arrival_turn_waypoints.push_back(false);
  int vehicle_heading = CardinalHeading(start_arena.yaw);
  int current_lane = -1;
  for (std::size_t index = 0; index < traversals.size(); ++index) {
    const Traversal &traversal = traversals[index];
    if (Distance(traversal.start, traversal.end) <= 1.0e-6 ||
        !collision_.segment_is_free(traversal.start, traversal.end,
                                    dynamic_obstacles)) {
      result.message = "route contains a blocked drive segment";
      return false;
    }
    if (index > 0 &&
        Distance(traversals[index - 1].end, traversal.start) > 1.0e-6) {
      result.message = "route drive segments are disconnected";
      return false;
    }

    int drive_heading =
        CardinalHeading(std::atan2(traversal.end.y - traversal.start.y,
                                   traversal.end.x - traversal.start.x));
    int next_vehicle_heading = drive_heading;
    if (index == 0 && traversal.partial && traversal.directed_lane < 0 &&
        TurnQuarterSteps(vehicle_heading, drive_heading) != 0) {
      result.message =
          "route requires an unsupported initial reverse or lateral approach";
      return false;
    }
    if (traversal.directed_lane >= 0) {
      const DirectedLane &outgoing =
          graph_.directed_lane(traversal.directed_lane);
      drive_heading = outgoing.heading;
      next_vehicle_heading = traversal.reverse
                                 ? (outgoing.heading + 2) % kHeadingCount
                                 : outgoing.heading;
    }

    if (traversal.reverse) {
      if (traversal.directed_lane < 0 || current_lane < 0 ||
          graph_.directed_lane(traversal.directed_lane).lane != current_lane ||
          next_vehicle_heading != vehicle_heading) {
        result.message = "reverse action does not retrace the incoming road";
        return false;
      }
    } else if (current_lane >= 0 && traversal.directed_lane >= 0) {
      const DirectedLane &outgoing =
          graph_.directed_lane(traversal.directed_lane);
      const int turn = TurnQuarterSteps(vehicle_heading, outgoing.heading);
      if (turn == 2) {
        result.message = "route contains a forbidden U-turn";
        return false;
      }
      if (turn != 0) {
        try {
          const bool reverse_arrival =
              index > 0 && traversals[index - 1].reverse;
          if (reverse_arrival) {
            const DirectedLane &reverse_motion =
                graph_.directed_lane(traversals[index - 1].directed_lane);
            result.maneuvers.push_back(maneuvers_.make_reverse_arrival_turn(
                outgoing.from, reverse_motion.heading, vehicle_heading,
                outgoing, dynamic_obstacles));
            result.reverse_arrival_turn_waypoints.back() = true;
          } else {
            DirectedLane incoming;
            incoming.lane = current_lane;
            incoming.to = outgoing.from;
            incoming.heading = vehicle_heading;
            result.maneuvers.push_back(maneuvers_.make_turn(
                incoming.to, incoming, outgoing, dynamic_obstacles));
          }
          result.turn_waypoints.back() = true;
        } catch (const std::runtime_error &exception) {
          result.message = exception.what();
          return false;
        }
      }
    } else if (current_lane < 0 && traversal.directed_lane >= 0) {
      const DirectedLane &outgoing =
          graph_.directed_lane(traversal.directed_lane);
      const int turn = TurnQuarterSteps(vehicle_heading, outgoing.heading);
      if (turn == 2) {
        result.message = "route starts with a forbidden U-turn";
        return false;
      }
      if (turn != 0) {
        if (Distance(traversal.start, config_.nodes[outgoing.from].position) >
            1.0e-6) {
          result.message =
              "route requests an initial turn away from a junction";
          return false;
        }
        try {
          result.maneuvers.push_back(maneuvers_.make_start_turn(
              outgoing.from, vehicle_heading, outgoing, dynamic_obstacles));
          result.turn_waypoints.back() = true;
        } catch (const std::runtime_error &exception) {
          result.message = exception.what();
          return false;
        }
      }
    }

    const double yaw = HeadingYaw(next_vehicle_heading);
    Maneuver drive;
    drive.type =
        traversal.reverse ? ManeuverType::REVERSE : ManeuverType::DRIVE;
    if (traversal.directed_lane >= 0) {
      const DirectedLane &directed =
          graph_.directed_lane(traversal.directed_lane);
      drive.incoming_lane_id = config_.lanes[directed.lane].id;
      drive.outgoing_lane_id = drive.incoming_lane_id;
      drive.node_id = config_.nodes[directed.to].id;
    }
    drive.start = {traversal.start, yaw};
    drive.end = {traversal.end, yaw};
    drive.distance = Distance(traversal.start, traversal.end);
    drive.collision_checked = true;
    result.maneuvers.push_back(std::move(drive));
    result.length += Distance(traversal.start, traversal.end);

    if (Distance(result.arena_path.back().position, traversal.start) > 1.0e-6) {
      result.arena_path.push_back({traversal.start, yaw});
      result.turn_waypoints.push_back(false);
      result.reverse_arrival_turn_waypoints.push_back(false);
    }
    if (!result.turn_waypoints.back()) {
      result.arena_path.back().yaw = yaw;
    }
    result.arena_path.push_back({traversal.end, yaw});
    result.turn_waypoints.push_back(false);
    result.reverse_arrival_turn_waypoints.push_back(false);
    vehicle_heading = next_vehicle_heading;
    if (traversal.directed_lane >= 0) {
      current_lane = graph_.directed_lane(traversal.directed_lane).lane;
    }
  }

  Maneuver finish;
  finish.type = ManeuverType::FINISH;
  finish.start = result.arena_path.back();
  finish.end = result.arena_path.back();
  finish.collision_checked = true;
  result.maneuvers.push_back(std::move(finish));
  result.navigation_path.reserve(result.arena_path.size());
  for (const Pose &pose : result.arena_path) {
    result.navigation_path.push_back(ArenaToMap(pose, config_.transform));
  }
  if (result.turn_waypoints.size() != result.arena_path.size()) {
    result.message = "compiled turn metadata does not match the path";
    return false;
  }
  if (result.reverse_arrival_turn_waypoints.size() !=
      result.arena_path.size()) {
    result.message =
        "compiled reverse-arrival metadata does not match the path";
    return false;
  }
  result.collision_checked = true;
  return true;
}

} // namespace new_map_planner
