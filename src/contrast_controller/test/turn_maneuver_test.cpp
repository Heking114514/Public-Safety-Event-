#include "navigation_safety/checker.hpp"
#include "vehicle_loopback_sim/motion.hpp"
#include "visual_navigation/rate_limiter.hpp"
#include "visual_navigation/launch_departure.hpp"
#include "visual_navigation/turn_maneuver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kStep = 1.0 / 30.0;
using Phase = visual_navigation::TurnManeuverPhase;
using Result = visual_navigation::TurnManeuverResult;

visual_navigation::TurnManeuverConfig TestConfig() {
  const auto path =
      ament_index_cpp::get_package_share_directory("contrast_controller") +
      "/config/contrast_controller.yaml";
  const auto parameters =
      YAML::LoadFile(path)["contrast_controller"]["ros__parameters"];
  visual_navigation::TurnManeuverConfig config;
  config.enabled = parameters["turn_maneuver_enabled"].as<bool>();
  config.entry_target_x =
      parameters["turn_maneuver_entry_target_x"].as<double>();
  config.entry_target_y =
      parameters["turn_maneuver_entry_target_y"].as<double>();
  config.entry_speed = parameters["turn_maneuver_entry_speed"].as<double>();
  config.creep_linear_speed =
      parameters["turn_maneuver_creep_speed"].as<double>();
  config.turn_linear_speed =
      parameters["turn_maneuver_turn_linear_speed"].as<double>();
  config.turn_angular_speed =
      parameters["turn_maneuver_angular_speed"].as<double>();
  config.turn_min_angular_speed =
      parameters["turn_maneuver_min_angular_speed"].as<double>();
  config.entry_position_tolerance =
      parameters["turn_maneuver_entry_tolerance"].as<double>();
  config.entry_lateral_tolerance =
      parameters["turn_maneuver_entry_lateral_tolerance"].as<double>();
  config.entry_yaw_tolerance =
      parameters["turn_maneuver_entry_yaw_tolerance"].as<double>();
  config.phase_timeout = parameters["turn_maneuver_phase_timeout"].as<double>();
  config.no_progress_timeout =
      parameters["turn_maneuver_no_progress_timeout"].as<double>();
  config.min_position_progress =
      parameters["turn_maneuver_min_position_progress"].as<double>();
  config.min_yaw_progress =
      parameters["turn_maneuver_min_yaw_progress"].as<double>();
  config.creep_distance =
      parameters["turn_maneuver_creep_distance"].as<double>();
  config.entry_heading_gain =
      parameters["turn_maneuver_entry_heading_gain"].as<double>();
  config.entry_lateral_gain =
      parameters["turn_maneuver_entry_lateral_gain"].as<double>();
  config.entry_max_angular_speed =
      parameters["turn_maneuver_entry_max_angular_speed"].as<double>();
  config.exit_lateral_tolerance =
      parameters["turn_maneuver_exit_lateral_tolerance"].as<double>();
  return config;
}

visual_navigation::TurnManeuverInput Input(double x, double y, double yaw,
                                           double direction = 1.0) {
  visual_navigation::TurnManeuverInput input;
  input.context.pose = {x, y, yaw};
  input.context.geometry_heading = 0.0;
  input.context.target_heading = direction * kPi / 2.0;
  input.context.arrival_heading = yaw;
  return input;
}

// Use the same output limiting and axle integration as the production stack.
class TruthModel {
public:
  explicit TruthModel(bool collision_check = false) : check_(collision_check) {
    const auto physical = YAML::LoadFile(
        ament_index_cpp::get_package_share_directory("vehicle_config") +
        "/config/vehicle_physical.yaml");
    base_x_ =
        physical["differential_drive"]["base_link_offset_x_m"].as<double>();
    base_y_ =
        physical["differential_drive"]["base_link_offset_y_m"].as<double>();
    footprint_ = {physical["chassis"]["length_m"].as<double>(),
                  physical["chassis"]["width_m"].as<double>(), 0.0, 0.0, 0.0};
    const auto parameters = YAML::LoadFile(
        ament_index_cpp::get_package_share_directory("visual_navigation") +
        "/config/waypoint_navigation.yaml")["waypoint_navigator"]
                                           ["ros__parameters"];
    minimum_ = parameters["min_linear_command_speed"].as<double>();
    maximum_ = parameters["max_linear_speed"].as<double>();
    linear_up_ = parameters["max_linear_acceleration"].as<double>();
    linear_down_ = parameters["max_linear_deceleration"].as<double>();
    angular_up_ = parameters["max_angular_acceleration"].as<double>();
    angular_down_ = parameters["max_angular_deceleration"].as<double>();
    const auto arena = YAML::LoadFile(
        ament_index_cpp::get_package_share_directory("new_map_planner") +
        "/config/arena.yaml");
    const double ox = arena["frame"]["map_origin_m"][0].as<double>();
    const double oy = arena["frame"]["map_origin_m"][1].as<double>();
    const double yaw =
        arena["frame"]["map_x_heading_deg"].as<double>() * kPi / 180.0;
    const auto polygon = [ox, oy, yaw](const YAML::Node &rectangle) {
      const double x0 = rectangle[0].as<double>(),
                   y0 = rectangle[1].as<double>();
      const double x1 = rectangle[2].as<double>(),
                   y1 = rectangle[3].as<double>();
      navigation_safety::Polygon result;
      for (const auto &p : std::vector<navigation_safety::Point>{
               {x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}) {
        result.points.push_back(
            {std::cos(yaw) * (p.x - ox) + std::sin(yaw) * (p.y - oy),
             -std::sin(yaw) * (p.x - ox) + std::cos(yaw) * (p.y - oy)});
      }
      return result;
    };
    for (const auto &region : arena["arena"]["free_regions"]) {
      const auto converted = polygon(region);
      navigation_safety::Rectangle bounds{
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()};
      for (const auto &p : converted.points) {
        bounds.minimum_x = std::min(bounds.minimum_x, p.x);
        bounds.minimum_y = std::min(bounds.minimum_y, p.y);
        bounds.maximum_x = std::max(bounds.maximum_x, p.x);
        bounds.maximum_y = std::max(bounds.maximum_y, p.y);
      }
      free_.push_back(bounds);
    }
    for (const auto &obstacle : arena["arena"]["obstacles"])
      obstacles_.push_back(polygon(obstacle));
  }

  void Apply(visual_navigation::TurnManeuverInput &input,
             const visual_navigation::TurnManeuverDecision &decision) {
    const auto desired = decision.command;
    if (std::abs(desired.linear_x) <= 1.0e-9 &&
        std::abs(desired.angular_z) <= 1.0e-9) {
      applied_ = {};
    } else {
      const double target =
          std::max(-maximum_, std::min(maximum_, desired.linear_x));
      applied_.linear_x = visual_navigation::LimitRate(
          target, applied_.linear_x, linear_up_, linear_down_, kStep);
      if (std::abs(target) > 1.0e-9 && std::abs(applied_.linear_x) < minimum_)
        applied_.linear_x = std::copysign(minimum_, target);
      applied_.angular_z =
          visual_navigation::LimitRate(desired.angular_z, applied_.angular_z,
                                       angular_up_, angular_down_, kStep);
    }
    vehicle_loopback_sim::Pose2d pose{
        input.context.pose.x, input.context.pose.y, input.context.pose.yaw};
    // Subsample the body sweep more finely than the live 50 Hz loopback.
    for (int sample = 0; sample < 5; ++sample) {
      const auto candidate = vehicle_loopback_sim::integrate_base_link(
          pose, applied_, kStep / 5.0, base_x_, base_y_);
      navigation_safety::Report report;
      if (check_ && !navigation_safety::pose_is_safe(
                        {{candidate.x, candidate.y}, candidate.yaw}, footprint_,
                        free_, obstacles_, {}, &report)) {
        ++collisions;
        applied_ = {};
        break;
      }
      pose = candidate;
    }
    input.context.pose = {pose.x, pose.y, pose.yaw};
    input.context.applied_twist = {applied_.linear_x, applied_.angular_z};
    input.context.measured_twist = input.context.applied_twist;
    input.context.measured_yaw_rate_valid = true;
  }
  int collisions{0};

private:
  bool check_;
  double base_x_, base_y_, minimum_, maximum_, linear_up_, linear_down_,
      angular_up_, angular_down_;
  vehicle_loopback_sim::Twist2d applied_;
  navigation_safety::Footprint footprint_;
  std::vector<navigation_safety::Rectangle> free_;
  std::vector<navigation_safety::Polygon> obstacles_;
};

void AssertComplete(visual_navigation::TurnManeuverInput input, bool guard,
                    bool launch = false) {
  auto config = TestConfig();
  if (launch) config.entry_target_x = 0.11 - input.context.junction_x;
  visual_navigation::TurnManeuverController controller(config);
  TruthModel truth(guard);
  controller.begin(0.0);
  std::vector<Phase> phases;
  visual_navigation::TurnManeuverDecision decision;
  for (int step = 0; step < 900; ++step) {
    decision = controller.update(input, step * kStep);
    if (phases.empty() || phases.back() != decision.phase)
      phases.push_back(decision.phase);
    if (decision.result != Result::RUNNING)
      break;
    truth.Apply(input, decision);
  }
  const std::vector<Phase> expected{Phase::CAPTURE, Phase::APPROACH_ENTRY,
                                    Phase::TURN_60, Phase::DIAGONAL_CREEP,
                                    Phase::TURN_30, Phase::COMPLETE};
  EXPECT_EQ(phases, expected);
  EXPECT_EQ(decision.result, Result::COMPLETED)
      << "pose=" << input.context.pose.x << "," << input.context.pose.y << ","
      << input.context.pose.yaw;
  EXPECT_EQ(truth.collisions, 0);
  EXPECT_NEAR(input.context.pose.yaw, input.context.target_heading,
              config.entry_yaw_tolerance);
  EXPECT_NEAR(input.context.pose.x, input.context.junction_x,
              config.exit_lateral_tolerance);
}
} // namespace

TEST(ContrastTurn, FullLeftAndRightAtLaunchWithRealOutputAndCollision) {
  for (double direction : {-1.0, 1.0}) {
    auto input = Input(0.173, 0.0, 0.0, direction);
    input.context.junction_x = 0.2;
    AssertComplete(input, true);
  }
}

TEST(ContrastTurn, FixedElevenCentimetreLaunchWithCollision) {
  for (double direction : {-1.0, 1.0}) {
    auto input = Input(0.11, 0.0, 0.0, direction);
    input.context.junction_x = 0.2;
    AssertComplete(input, true, true);
  }
}

TEST(ContrastTurn, ForwardEntryAlsoCompletes) {
  AssertComplete(Input(-0.09, 0.0, 0.0), false);
}

TEST(ContrastTurn, ZeroMotionStopsAndRequestsReplan) {
  auto config = TestConfig();
  config.no_progress_timeout = 0.25; // Test-only accelerated clock.
  visual_navigation::TurnManeuverController controller(config);
  auto input = Input(-0.04, 0.0, 0.0);
  controller.begin(0.0);
  controller.update(input, 0.0);
  controller.update(input, kStep);
  auto decision = controller.update(input, 2 * kStep);
  ASSERT_LT(decision.command.linear_x, 0.0);
  decision = controller.update(input, 0.4);
  EXPECT_EQ(decision.result, Result::NEED_REPLAN);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
  EXPECT_FALSE(controller.active());
}

TEST(ContrastTurn, TranslationCannotMaskMissingYawProgress) {
  auto config = TestConfig();
  config.no_progress_timeout = 0.25;
  visual_navigation::TurnManeuverController controller(config);
  auto input = Input(config.entry_target_x, 0.0, 0.0);
  controller.begin(0.0);
  controller.update(input, 0.0);
  controller.update(input, kStep);
  auto decision = controller.update(input, 2 * kStep);
  ASSERT_EQ(decision.phase, Phase::TURN_60);
  input.context.pose.x += 0.06;
  decision = controller.update(input, 0.5);
  EXPECT_EQ(decision.result, Result::NEED_REPLAN);
}

TEST(ContrastTurn, EntryOvershootCannotReverseForever) {
  const auto config = TestConfig();
  visual_navigation::TurnManeuverController controller(config);
  auto input = Input(-0.04, 0.04, 0.0);
  controller.begin(0.0);
  controller.update(input, 0.0);
  controller.update(input, kStep);
  input.context.pose.x =
      config.entry_target_x - 2 * config.entry_position_tolerance;
  const auto decision = controller.update(input, 2 * kStep);
  EXPECT_EQ(decision.result, Result::NEED_REPLAN);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
}

// Route fixtures follow arena.yaml: home=(0,0), first junction=(0.20,0).
TEST(ContrastLaunch, FixedHomeDistanceAndOrdinaryRoutes) {
  visual_navigation::LaunchDeparture launch;
  launch.configure(0.0, 0.0, 0.20, 0.0);
  std::vector<visual_navigation::RouteWaypoint> route(3);
  route[1].x = 0.20;
  route[1].turn_junction = true;
  route[2].x = 0.20;
  route[2].y = -0.50;
  EXPECT_TRUE(launch.applies(route, 1));
  EXPECT_FALSE(launch.applies(route, 0));
  EXPECT_FALSE(launch.applies(route, 2));
  EXPECT_NEAR(launch.remaining(0.04, 0.0), 0.07, 1e-12);
  EXPECT_NEAR(launch.entry_target_x(), -0.09, 1e-12);
  route[0].x = 0.04; // A replan from the outbound lane still targets home+11cm.
  EXPECT_TRUE(launch.applies(route, 1));
  route[0].x = 0.15;
  EXPECT_FALSE(launch.applies(route, 1));
  route[0].x = 1.0;
  route[1].x = 1.2;
  route[2].x = 1.2;
  EXPECT_FALSE(launch.applies(route, 1));
}

TEST(ContrastLaunch, HomeGeometryRotatesWithMapAndRejectsReturn) {
  visual_navigation::LaunchDeparture launch;
  launch.configure(1.0, 2.0, 1.0, 2.20);
  std::vector<visual_navigation::RouteWaypoint> route(3);
  route[0].x = route[1].x = 1.0;
  route[0].y = 2.0;
  route[1].y = route[2].y = 2.20;
  route[1].turn_junction = true;
  route[2].x = 1.50;
  EXPECT_TRUE(launch.applies(route, 1));
  EXPECT_NEAR(launch.remaining(1.0, 2.04), 0.07, 1e-12);
  route[1].reverse_arrival_turn = true;
  EXPECT_FALSE(launch.applies(route, 1));
  route[1].reverse_arrival_turn = false;
  route[1].rfid_supernode = true;
  EXPECT_FALSE(launch.applies(route, 1));
  route[1].rfid_supernode = false;
  std::swap(route[0], route[1]); // Return to home is not outbound departure.
  EXPECT_FALSE(launch.applies(route, 1));
}

TEST(ContrastLaunch, OrdinaryRecoveryRestoresEntryAfterLaunchOverride) {
  auto ordinary = TestConfig();
  auto launch = ordinary;
  launch.entry_target_x = -0.09;
  visual_navigation::TurnManeuverController controller(launch);
  controller.configure(ordinary);
  controller.begin(0.0);
  auto input = Input(-0.09, 0.0, 0.0);
  controller.update(input, 0.0);
  controller.update(input, kStep);
  const auto decision = controller.update(input, 2 * kStep);
  EXPECT_EQ(decision.phase, Phase::APPROACH_ENTRY);
  EXPECT_GT(decision.command.linear_x, 0.0);
}
