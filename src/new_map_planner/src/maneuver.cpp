#include "new_map_planner/maneuver.hpp"

#include "new_map_planner/geometry.hpp"
#include "visual_navigation/corner_shunt.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace new_map_planner {
namespace {

PlannerConfig TurnConfig(const PlannerConfig &config) {
  PlannerConfig turn_config = config;
  turn_config.safety_margin = config.turn_safety_margin;
  return turn_config;
}

visual_navigation::CornerShuntPose
Integrate(visual_navigation::CornerShuntPose pose,
          const visual_navigation::CornerShuntTwist &command, double elapsed) {
  if (std::abs(command.angular_z) <= 1.0e-6) {
    pose.x += command.linear_x * elapsed * std::cos(pose.yaw);
    pose.y += command.linear_x * elapsed * std::sin(pose.yaw);
    return pose;
  }
  const double next_yaw = pose.yaw + command.angular_z * elapsed;
  const double radius = command.linear_x / command.angular_z;
  pose.x += radius * (std::sin(next_yaw) - std::sin(pose.yaw));
  pose.y -= radius * (std::cos(next_yaw) - std::cos(pose.yaw));
  pose.yaw = next_yaw;
  return pose;
}

Pose BasePose(const visual_navigation::CornerShuntPose &axle,
              double base_offset) {
  return {{axle.x + base_offset * std::cos(axle.yaw),
           axle.y + base_offset * std::sin(axle.yaw)},
          NormalizeAngle(axle.yaw)};
}

} // namespace

ManeuverLibrary::ManeuverLibrary(const PlannerConfig &config,
                                 const CollisionChecker &collision)
    : config_(config), turn_config_(TurnConfig(config)),
      collision_(turn_config_) {
  (void)collision;
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
  profile.reserve(config_.left_turn_profile.size());
  for (const Pose &sample : config_.left_turn_profile) {
    // The calibrated profile is measured at the axle midpoint, while all ROS
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

  visual_navigation::CornerShuntConfig shunt;
  shunt.road_width = config_.lanes[outgoing.lane].width;
  shunt.road_margin = config_.turn_safety_margin;
  shunt.vehicle_length = config_.vehicle_length;
  shunt.vehicle_width = config_.vehicle_width;
  shunt.wheel_track = config_.wheel_track;
  shunt.base_offset_x = config_.base_link_offset_x;
  visual_navigation::CornerShuntController controller(shunt);

  const Point &junction =
      config_.nodes[static_cast<std::size_t>(node)].position;
  const double incoming_yaw = HeadingYaw(incoming_motion_heading);
  visual_navigation::CornerShuntContext context;
  context.pose = {junction.x + 0.005 * std::cos(incoming_yaw),
                  junction.y + 0.005 * std::sin(incoming_yaw),
                  HeadingYaw(vehicle_heading)};
  context.junction_x = junction.x;
  context.junction_y = junction.y;
  context.incoming_heading = incoming_yaw;
  context.target_heading = HeadingYaw(outgoing.heading);
  context.reverse_arrival = true;

  std::vector<Pose> sweep;
  sweep.push_back(BasePose(context.pose, config_.base_link_offset_x));
  controller.begin(0.0);
  double now = 0.0;
  const double step = 1.0 / shunt.control_frequency;
  for (int iteration = 0; iteration < 1800; ++iteration) {
    const auto decision = controller.update(context, now);
    if (decision.complete)
      break;
    if (decision.blocked)
      sweep.clear();
    if (decision.complete || decision.blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (decision.phase == visual_navigation::CornerShuntPhase::STOP ||
        decision.phase == visual_navigation::CornerShuntPhase::CHECK) {
      context.applied_twist = {};
    } else {
      context.applied_twist.linear_x = visual_navigation::LimitRate(
          decision.command.linear_x, context.applied_twist.linear_x,
          shunt.linear_acceleration, shunt.linear_deceleration, step);
      context.applied_twist.angular_z = visual_navigation::LimitRate(
          decision.command.angular_z, context.applied_twist.angular_z,
          shunt.angular_acceleration, shunt.angular_deceleration, step);
      context.pose = Integrate(context.pose, context.applied_twist, step);
    }
    const Pose base = BasePose(context.pose, config_.base_link_offset_x);
    if (!controller.pose_is_safe(context, context.pose) ||
        !collision_.pose_is_free(base, dynamic_obstacles)) {
      sweep.clear();
      break;
    }
    sweep.push_back(base);
    now += step;
  }
  if (sweep.empty() || std::abs(NormalizeAngle(sweep.back().yaw -
                                               HeadingYaw(outgoing.heading))) >
                           shunt.target_yaw_tolerance) {
    throw std::runtime_error(
        "requested reverse-arrival turn has no collision-free maneuver");
  }

  const int turn = TurnQuarterSteps(incoming_motion_heading, outgoing.heading);
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
