#include "new_map_planner/maneuver.hpp"

#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace new_map_planner {
namespace {

PlannerConfig TurnConfig(const PlannerConfig &config) {
  PlannerConfig turn_config = config;
  turn_config.safety_margin = config.turn_safety_margin;
  // The local profile is expressed at base_link after the axle offset is
  // applied in turn_sweep; CollisionChecker therefore uses its normal body
  // centre convention here.
  turn_config.footprint_center_offset_x = 0.0;
  return turn_config;
}

std::vector<Pose> GenerateLocalTurnProfile(const PlannerConfig &config) {
  const auto narrowest = std::min_element(
      config.lanes.begin(), config.lanes.end(),
      [](const Lane &left, const Lane &right) {
        return left.width < right.width;
      });
  if (narrowest == config.lanes.end())
    return {};
  std::vector<Pose> profile;
  // This is a route-level geometry sweep. It deliberately models only the
  // road connection; velocity control and short-horizon safety remain in
  // navigation controller. Keep the sweep inside the junction so every lane
  // transition is checked against the same static footprint.
  const double approach = std::max(0.075, 0.40 * narrowest->width);
  const int samples = 36;
  // A tangent quarter-circle keeps the body heading aligned with its path;
  // unlike an in-place rotation, it does not sweep the inner chassis corner
  // outside the road envelope.
  for (int index = 0; index <= samples; ++index) {
    const double ratio = static_cast<double>(index) / samples;
    const double angle = ratio * 0.5 * kPi;
    profile.push_back({{-approach + approach * std::sin(angle),
                        approach * (1.0 - std::cos(angle))},
                       angle});
  }
  return profile;
}

} // namespace

ManeuverLibrary::ManeuverLibrary(const PlannerConfig &config,
                                 const CollisionChecker &collision)
    : config_(config), turn_config_(TurnConfig(config)),
      collision_(turn_config_), local_left_turn_profile_(
                                   GenerateLocalTurnProfile(config)) {
  (void)collision;
  if (local_left_turn_profile_.empty())
    throw std::runtime_error("closed-loop corner sweep is not feasible");
}

bool ManeuverLibrary::turn_pose_is_free(
    const Pose &pose, const std::vector<Polygon> &dynamic_obstacles) const {
  return collision_.pose_is_free(pose, dynamic_obstacles);
}

std::vector<Pose> ManeuverLibrary::turn_sweep(int node, int incoming_heading,
                                              int outgoing_heading) const {
  const int turn = TurnQuarterSteps(incoming_heading, outgoing_heading);
  if (turn != 1 && turn != -1) {
    return {};
  }
  const double incoming_yaw = HeadingYaw(incoming_heading);
  const Point &junction =
      config_.nodes[static_cast<std::size_t>(node)].position;
  const double cosine = std::cos(incoming_yaw);
  const double sine = std::sin(incoming_yaw);
  std::vector<Pose> profile;
  profile.reserve(local_left_turn_profile_.size());
  for (const Pose &sample : local_left_turn_profile_) {
    // The search profile is measured at the axle midpoint, while all ROS
    // paths use base_link. Convert before rotating the profile into the arena.
    const double local_yaw = static_cast<double>(turn) * sample.yaw;
    const double axle_y = turn > 0 ? sample.position.y : -sample.position.y;
    const double local_x =
        sample.position.x + config_.base_link_offset_x * std::cos(local_yaw);
    const double local_y =
        axle_y + config_.base_link_offset_x * std::sin(local_yaw);
    profile.push_back({
        {
            junction.x + cosine * local_x - sine * local_y,
            junction.y + sine * local_x + cosine * local_y,
        },
        NormalizeAngle(incoming_yaw + local_yaw),
    });
  }
  return profile;
}

bool ManeuverLibrary::transition_is_valid(
    int node, int incoming_heading, const DirectedLane &outgoing,
    const std::vector<Polygon> &dynamic_obstacles) const {
  if (node != outgoing.from) {
    return false;
  }
  const int turn = TurnQuarterSteps(incoming_heading, outgoing.heading);
  if (turn == 0) {
    return true;
  }
  if (turn == 2) {
    return false;
  }
  return collision_.sweep_is_free(
      turn_sweep(node, incoming_heading, outgoing.heading), dynamic_obstacles);
}

Maneuver ManeuverLibrary::make_turn(
    int node, const DirectedLane &incoming, const DirectedLane &outgoing,
    const std::vector<Polygon> &dynamic_obstacles) const {
  if (!transition_is_valid(node, incoming.heading, outgoing,
                           dynamic_obstacles)) {
    throw std::runtime_error(
        "requested road transition has no collision-free maneuver");
  }
  const int turn = TurnQuarterSteps(incoming.heading, outgoing.heading);
  Maneuver maneuver;
  maneuver.type = turn > 0 ? ManeuverType::TURN_LEFT : ManeuverType::TURN_RIGHT;
  maneuver.node_id = config_.nodes[node].id;
  maneuver.incoming_lane_id = config_.lanes[incoming.lane].id;
  maneuver.outgoing_lane_id = config_.lanes[outgoing.lane].id;
  const std::vector<Pose> profile =
      turn_sweep(node, incoming.heading, outgoing.heading);
  maneuver.start = profile.front();
  maneuver.end = profile.back();
  maneuver.stop_required = true;
  maneuver.collision_checked =
      collision_.sweep_is_free(profile, dynamic_obstacles);
  return maneuver;
}

Maneuver ManeuverLibrary::make_start_turn(
    int node, int incoming_heading, const DirectedLane &outgoing,
    const std::vector<Polygon> &dynamic_obstacles) const {
  if (!transition_is_valid(node, incoming_heading, outgoing,
                           dynamic_obstacles)) {
    throw std::runtime_error(
        "requested initial turn has no collision-free maneuver");
  }
  Maneuver maneuver;
  maneuver.type = ManeuverType::START_ALIGN;
  maneuver.node_id = config_.nodes[node].id;
  // Initial turns are compiled before a directed incoming lane exists in the
  // traversal list. Preserve the lane metadata so route-level recovery can
  // identify the road to retrace.
  for (const auto &lane : config_.lanes) {
    if (lane.to != node || lane.from < 0 || lane.from >=
        static_cast<int>(config_.nodes.size())) {
      continue;
    }
    const auto &from = config_.nodes[static_cast<std::size_t>(lane.from)].position;
    const auto &to = config_.nodes[static_cast<std::size_t>(lane.to)].position;
    if (CardinalHeading(std::atan2(to.y - from.y, to.x - from.x)) ==
        incoming_heading) {
      maneuver.incoming_lane_id = lane.id;
      break;
    }
  }
  maneuver.outgoing_lane_id = config_.lanes[outgoing.lane].id;
  const std::vector<Pose> profile =
      turn_sweep(node, incoming_heading, outgoing.heading);
  maneuver.start = profile.front();
  maneuver.end = profile.back();
  maneuver.stop_required = true;
  maneuver.collision_checked =
      collision_.sweep_is_free(profile, dynamic_obstacles);
  return maneuver;
}

Maneuver ManeuverLibrary::make_reverse_arrival_turn(
    int node, int incoming_motion_heading, int vehicle_heading,
    const DirectedLane &outgoing,
    const std::vector<Polygon> &dynamic_obstacles) const {
  if (node != outgoing.from ||
      TurnQuarterSteps(incoming_motion_heading, outgoing.heading) == 2 ||
      (incoming_motion_heading + 2) % kHeadingCount != vehicle_heading) {
    throw std::runtime_error("invalid reverse-arrival turn request");
  }

  const int turn = TurnQuarterSteps(incoming_motion_heading, outgoing.heading);
  if (turn != 1 && turn != -1) {
    throw std::runtime_error(
        "requested reverse-arrival turn has no collision-free maneuver");
  }
  std::vector<Pose> sweep = turn_sweep(node, incoming_motion_heading,
                                       outgoing.heading);
  if (!sweep.empty()) {
    const double start_yaw = HeadingYaw(vehicle_heading);
    const double yaw_delta = NormalizeAngle(HeadingYaw(outgoing.heading) -
                                            start_yaw);
    for (std::size_t index = 0; index < sweep.size(); ++index) {
      const double ratio = sweep.size() <= 1
                               ? 1.0
                               : static_cast<double>(index) /
                                     static_cast<double>(sweep.size() - 1);
      sweep[index].yaw = NormalizeAngle(start_yaw + yaw_delta * ratio);
    }
  }
  if (sweep.empty() ||
      !collision_.sweep_is_free(sweep, dynamic_obstacles)) {
    throw std::runtime_error(
        "requested reverse-arrival turn has no collision-free maneuver");
  }

  Maneuver maneuver;
  maneuver.type = turn > 0 ? ManeuverType::TURN_LEFT : ManeuverType::TURN_RIGHT;
  maneuver.node_id = config_.nodes[static_cast<std::size_t>(node)].id;
  maneuver.outgoing_lane_id = config_.lanes[outgoing.lane].id;
  maneuver.start = sweep.front();
  maneuver.end = sweep.back();
  maneuver.stop_required = true;
  maneuver.collision_checked = true;
  return maneuver;
}

} // namespace new_map_planner
