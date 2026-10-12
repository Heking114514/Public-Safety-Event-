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
  result.waypoint_supernode_ids.clear();
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
  result.waypoint_supernode_ids.emplace_back();
  int vehicle_heading = CardinalHeading(start_arena.yaw);
  int current_lane = -1;
  const auto unchecked_turn = [this](int node, int incoming_heading,
                                     int incoming_lane,
                                     const DirectedLane &outgoing,
                                     ManeuverType type) {
    Maneuver maneuver;
    maneuver.type = type;
    maneuver.node_id = config_.nodes[node].id;
    maneuver.incoming_lane_id = incoming_lane >= 0
                                   ? config_.lanes[incoming_lane].id
                                   : std::string{};
    maneuver.outgoing_lane_id = config_.lanes[outgoing.lane].id;
    const auto profile =
        maneuvers_.turn_sweep(node, incoming_heading, outgoing.heading);
    if (!profile.empty()) {
      maneuver.start = profile.front();
      maneuver.end = profile.back();
    }
    maneuver.stop_required = true;
    maneuver.collision_checked = true;
    return maneuver;
  };
  for (std::size_t index = 0; index < traversals.size(); ++index) {
    const Traversal &traversal = traversals[index];
    const bool initial_partial =
        index == 0 && traversal.partial && traversal.directed_lane < 0;
    const double segment_heading =
        initial_partial ? start_arena.yaw
                        : std::atan2(traversal.end.y - traversal.start.y,
                                     traversal.end.x - traversal.start.x);
    if (Distance(traversal.start, traversal.end) <= 1.0e-6 ||
        (result.validate_geometry &&
         !collision_.segment_is_free(traversal.start, traversal.end,
                                     segment_heading, dynamic_obstacles))) {
      result.message = "route contains a blocked drive segment";
      return false;
    }
    if (index > 0 &&
        Distance(traversals[index - 1].end, traversal.start) > 1.0e-6) {
      result.message = "route drive segments are disconnected";
      return false;
    }

    int drive_heading = CardinalHeading(segment_heading);
    int next_vehicle_heading = drive_heading;
    if (initial_partial &&
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
      const bool initial_reverse =
          current_lane < 0 &&
          (index == 0 || traversals[index - 1].directed_lane < 0);
      const bool same_lane_reverse =
          current_lane >= 0 && traversal.directed_lane >= 0 &&
          graph_.directed_lane(traversal.directed_lane).lane == current_lane;
      if (traversal.directed_lane < 0 ||
          (!initial_reverse && !same_lane_reverse) ||
          next_vehicle_heading != vehicle_heading ||
          (initial_reverse &&
           Distance(traversal.start,
                    config_.nodes[graph_.directed_lane(
                        traversal.directed_lane).from].position) > 1.0e-6)) {
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
            if (result.validate_geometry) {
              result.maneuvers.push_back(maneuvers_.make_reverse_arrival_turn(
                  outgoing.from, reverse_motion.heading, vehicle_heading,
                  outgoing, dynamic_obstacles));
            } else {
              result.maneuvers.push_back(unchecked_turn(
                  outgoing.from, reverse_motion.heading, reverse_motion.lane,
                  outgoing,
                  turn > 0 ? ManeuverType::TURN_LEFT
                           : ManeuverType::TURN_RIGHT));
            }
            result.reverse_arrival_turn_waypoints.back() = true;
          } else {
            DirectedLane incoming;
            incoming.lane = current_lane;
            incoming.to = outgoing.from;
            incoming.heading = vehicle_heading;
            if (result.validate_geometry) {
              result.maneuvers.push_back(maneuvers_.make_turn(
                  incoming.to, incoming, outgoing, dynamic_obstacles));
            } else {
              result.maneuvers.push_back(unchecked_turn(
                  incoming.to, incoming.heading, incoming.lane, outgoing,
                  turn > 0 ? ManeuverType::TURN_LEFT
                           : ManeuverType::TURN_RIGHT));
            }
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
          int incoming_lane = -1;
          for (std::size_t lane_index = 0;
               lane_index < config_.lanes.size(); ++lane_index) {
            const auto &lane = config_.lanes[lane_index];
            if (lane.to != outgoing.from || lane.from < 0 ||
                lane.from >= static_cast<int>(config_.nodes.size())) {
              continue;
            }
            const auto &from = config_.nodes[static_cast<std::size_t>(lane.from)].position;
            const auto &to = config_.nodes[static_cast<std::size_t>(lane.to)].position;
            if (CardinalHeading(std::atan2(to.y - from.y, to.x - from.x)) ==
                vehicle_heading) {
              incoming_lane = static_cast<int>(lane_index);
              break;
            }
          }
          if (result.validate_geometry) {
            result.maneuvers.push_back(maneuvers_.make_start_turn(
                outgoing.from, vehicle_heading, outgoing, dynamic_obstacles));
          } else {
            result.maneuvers.push_back(unchecked_turn(
                outgoing.from, vehicle_heading, incoming_lane, outgoing,
                ManeuverType::START_ALIGN));
          }
          result.turn_waypoints.back() = true;
        } catch (const std::runtime_error &exception) {
          result.message = exception.what();
          return false;
        }
      }
    }

    const double yaw =
        initial_partial ? start_arena.yaw : HeadingYaw(next_vehicle_heading);
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
      result.waypoint_supernode_ids.emplace_back();
    }
    if (!result.turn_waypoints.back()) {
      result.arena_path.back().yaw = yaw;
    }
    result.arena_path.push_back({traversal.end, yaw});
    result.turn_waypoints.push_back(false);
    result.reverse_arrival_turn_waypoints.push_back(false);
    const auto supernode = traversal.directed_lane >= 0 ?
      config_.rfid_supernodes.find(graph_.directed_lane(traversal.directed_lane).to) :
      config_.rfid_supernodes.end();
    result.waypoint_supernode_ids.push_back(
      supernode == config_.rfid_supernodes.end() ? std::string{} : supernode->second);
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
  if (result.waypoint_supernode_ids.size() != result.arena_path.size()) {
    result.message = "compiled RFID supernode metadata does not match the path";
    return false;
  }
  result.collision_checked = true;
  return true;
}

} // namespace new_map_planner
