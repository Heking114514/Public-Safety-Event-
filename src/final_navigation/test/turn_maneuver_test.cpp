#include "navigation_safety/checker.hpp"
#include "visual_navigation/rate_limiter.hpp"
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
struct Pose2d { double x, y, yaw; };
struct Twist2d { double linear_x{0.0}, angular_z{0.0}; };

Pose2d IntegrateBaseLink(const Pose2d & pose, const Twist2d & twist,
                         double elapsed, double offset_x, double offset_y)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const double wheel_x = pose.x - cosine * offset_x + sine * offset_y;
  const double wheel_y = pose.y - sine * offset_x - cosine * offset_y;
  const double next_yaw = pose.yaw + twist.angular_z * elapsed;
  double next_wheel_x = wheel_x;
  double next_wheel_y = wheel_y;
  if (std::abs(twist.angular_z) <= 1.0e-9) {
    next_wheel_x += twist.linear_x * elapsed * cosine;
    next_wheel_y += twist.linear_x * elapsed * sine;
  } else {
    const double radius = twist.linear_x / twist.angular_z;
    next_wheel_x += radius * (std::sin(next_yaw) - sine);
    next_wheel_y -= radius * (std::cos(next_yaw) - cosine);
  }
  const double next_cosine = std::cos(next_yaw);
  const double next_sine = std::sin(next_yaw);
  return {next_wheel_x + next_cosine * offset_x - next_sine * offset_y,
          next_wheel_y + next_sine * offset_x + next_cosine * offset_y,
          next_yaw};
}
using Phase = visual_navigation::TurnManeuverPhase;
using Result = visual_navigation::TurnManeuverResult;

visual_navigation::TurnManeuverConfig TestConfig() {
  const auto path =
      ament_index_cpp::get_package_share_directory("final_navigation") +
      "/config/final_navigation.yaml";
  const auto parameters =
      YAML::LoadFile(path)["final_navigation"]["ros__parameters"];
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
        ament_index_cpp::get_package_share_directory("final_navigation") +
        "/config/final_navigation.yaml")["final_navigation"]
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
    Pose2d pose{input.context.pose.x, input.context.pose.y,
                input.context.pose.yaw};
    // Subsample the body sweep more finely than the live 50 Hz loopback.
    for (int sample = 0; sample < 5; ++sample) {
      const auto candidate = IntegrateBaseLink(
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
  Twist2d applied_;
  navigation_safety::Footprint footprint_;
  std::vector<navigation_safety::Rectangle> free_;
  std::vector<navigation_safety::Polygon> obstacles_;
};

void AssertComplete(visual_navigation::TurnManeuverInput input, bool guard) {
  const auto config = TestConfig();
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

TEST(ContrastTurn, EntryUsesLongitudinalToleranceDespiteLateralAndHeadingError) {
  const auto config = TestConfig();
  for (double drive_direction : {-1.0, 1.0}) {
    visual_navigation::TurnManeuverController controller(config);
    // 测试夹具：横向偏差 4 厘米，航向也超出原入口验收阈值。
    auto input = Input(config.entry_target_x - drive_direction * 0.03,
                       config.entry_target_y + 0.04,
                       2.0 * config.entry_yaw_tolerance);
    controller.begin(0.0);
    controller.update(input, 0.0);
    controller.update(input, kStep);
    const auto approaching = controller.update(input, 2 * kStep);
    ASSERT_EQ(approaching.phase, Phase::APPROACH_ENTRY);
    ASSERT_EQ(approaching.result, Result::RUNNING);
    EXPECT_GT(drive_direction * approaching.command.linear_x, 0.0);
    input.context.pose.x =
        config.entry_target_x - drive_direction * 1.5 * config.entry_position_tolerance;
    EXPECT_EQ(controller.update(input, 3 * kStep).phase, Phase::APPROACH_ENTRY);
    input.context.pose.x =
        config.entry_target_x - drive_direction * 0.5 * config.entry_position_tolerance;
    const auto within_tolerance = controller.update(input, 4 * kStep);
    EXPECT_EQ(within_tolerance.phase, Phase::TURN_60);
    EXPECT_DOUBLE_EQ(within_tolerance.command.linear_x, 0.0);
  }
}

TEST(ContrastTurn, EntryCrossingAdvancesDespiteLateralAndHeadingError) {
  const auto config = TestConfig();
  for (double drive_direction : {-1.0, 1.0}) {
    visual_navigation::TurnManeuverController controller(config);
    auto input = Input(config.entry_target_x - drive_direction * 0.03,
                       config.entry_target_y + 0.04, 2 * config.entry_yaw_tolerance);
    controller.begin(0.0);
    controller.update(input, 0.0);
    controller.update(input, kStep);
    ASSERT_EQ(controller.update(input, 2 * kStep).phase, Phase::APPROACH_ENTRY);
    input.context.pose.x =
        config.entry_target_x + drive_direction * 2 * config.entry_position_tolerance;
    const auto entered = controller.update(input, 3 * kStep);
    EXPECT_EQ(entered.phase, Phase::TURN_60);
    EXPECT_EQ(entered.result, Result::RUNNING);
    EXPECT_DOUBLE_EQ(entered.command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(entered.command.angular_z, 0.0);
  }
}

TEST(ContrastTurn, EntryFeedbackUsesHeadingWithoutLateralCorrection) {
  const auto config = TestConfig();
  for (double heading : {0.0, kPi / 2.0, -kPi / 2.0}) {
    for (double drive_direction : {-1.0, 1.0}) {
      for (double lateral : {-0.04, 0.0, 0.04}) {
        visual_navigation::TurnManeuverController controller(config);
        const double x = config.entry_target_x - drive_direction * 0.03;
        const double yaw_error = 0.5 * config.entry_yaw_tolerance;
        auto input = Input(x * std::cos(heading) - lateral * std::sin(heading),
                           x * std::sin(heading) + lateral * std::cos(heading),
                           heading - yaw_error);
        input.context.geometry_heading = heading;
        input.context.target_heading = heading + kPi / 2.0;
        controller.begin(0.0);
        controller.update(input, 0.0);
        controller.update(input, kStep);
        const auto decision = controller.update(input, 2 * kStep);
        ASSERT_EQ(decision.phase, Phase::APPROACH_ENTRY);
        EXPECT_GT(drive_direction * decision.command.linear_x, 0.0);
        EXPECT_NEAR(decision.command.angular_z,
                    config.entry_heading_gain * yaw_error, 1e-12);
        const double near_x = config.entry_target_x -
            drive_direction * 0.5 * config.entry_position_tolerance;
        input.context.pose.x = near_x * std::cos(heading) - lateral * std::sin(heading);
        input.context.pose.y = near_x * std::sin(heading) + lateral * std::cos(heading);
        EXPECT_EQ(controller.update(input, 3 * kStep).phase, Phase::TURN_60);
      }
    }
  }
}

TEST(ContrastTurn, ReachedTurnAngleCompletesDespiteExitLateralError) {
  const auto config = TestConfig();
  for (double direction : {-1.0, 1.0}) {
    visual_navigation::TurnManeuverController controller(config);
    auto input = Input(config.entry_target_x,
                       direction * config.entry_target_y, 0.0, direction);
    controller.begin(0.0);
    controller.update(input, 0.0);
    controller.update(input, kStep);
    ASSERT_EQ(controller.update(input, 2 * kStep).phase, Phase::TURN_60);
    input.context.pose.yaw = direction * kPi / 3.0;
    ASSERT_EQ(controller.update(input, 3 * kStep).phase, Phase::DIAGONAL_CREEP);
    input.context.pose.x += std::cos(input.context.pose.yaw) * config.creep_distance * 1.1;
    input.context.pose.y += std::sin(input.context.pose.yaw) * config.creep_distance * 1.1;
    ASSERT_EQ(controller.update(input, 4 * kStep).phase, Phase::TURN_30);
    input.context.pose.x = 2 * config.exit_lateral_tolerance;
    input.context.pose.yaw = direction * kPi / 2.0;
    const auto completed = controller.update(input, 5 * kStep);
    EXPECT_EQ(completed.phase, Phase::COMPLETE);
    EXPECT_EQ(completed.result, Result::COMPLETED);
    EXPECT_DOUBLE_EQ(completed.command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(completed.command.angular_z, 0.0);
    EXPECT_FALSE(controller.active());
  }
}

TEST(ContrastTurn, CreepDistanceOverrunWaitsForHeadingInsteadOfRejectingPose) {
  const auto config = TestConfig();
  for (double direction : {-1.0, 1.0}) {
    visual_navigation::TurnManeuverController controller(config);
    auto input = Input(config.entry_target_x,
                       direction * config.entry_target_y, 0.0, direction);
    controller.begin(0.0);
    controller.update(input, 0.0);
    controller.update(input, kStep);
    ASSERT_EQ(controller.update(input, 2 * kStep).phase, Phase::TURN_60);
    input.context.pose.yaw = direction * kPi / 3.0;
    ASSERT_EQ(controller.update(input, 3 * kStep).phase, Phase::DIAGONAL_CREEP);
    const double distance = config.creep_distance + 3 * config.entry_position_tolerance;
    input.context.pose.x += std::cos(input.context.pose.yaw) * distance;
    input.context.pose.y += std::sin(input.context.pose.yaw) * distance;
    input.context.pose.yaw += direction * 2 * config.entry_yaw_tolerance;
    const auto correcting = controller.update(input, 4 * kStep);
    EXPECT_EQ(correcting.phase, Phase::DIAGONAL_CREEP);
    EXPECT_EQ(correcting.result, Result::RUNNING);
    EXPECT_GT(correcting.command.linear_x, 0.0);
    EXPECT_LT(direction * correcting.command.angular_z, 0.0);
    input.context.pose.yaw = direction * kPi / 3.0;
    const auto entered = controller.update(input, 5 * kStep);
    EXPECT_EQ(entered.phase, Phase::TURN_30);
    EXPECT_EQ(entered.result, Result::RUNNING);
  }
}

TEST(ContrastTurn, PausedFixedActionKeepsCapturedTurnAndExcludesWallTime) {
  // 参数全部来自现有正常转弯 YAML；这里只验证暂停语义，不假定该参数
  // 能在地图上完成安全转弯，更不以此替代真实转弯场景验收。
  const auto config = TestConfig();
  for (int gate = 0; gate < 3; ++gate) {
    visual_navigation::TurnManeuverController controller(config);
    auto input = Input(config.entry_target_x, config.entry_target_y, 0.0);
    controller.begin(0.0);
    controller.update(input, 0.0);
    controller.update(input, kStep);
    const auto entered = controller.update(input, 2 * kStep);
    ASSERT_EQ(entered.phase, Phase::TURN_60);
    input.context.pose.yaw = 0.1;
    if (gate == 0) input.motion_allowed = false;
    if (gate == 1) input.localization_valid = false;
    if (gate == 2) input.obstacle_blocked = true;
    const auto paused = controller.update(input, 3 * kStep);
    ASSERT_EQ(paused.phase, Phase::PAUSED);
    EXPECT_DOUBLE_EQ(paused.command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(paused.command.angular_z, 0.0);
    EXPECT_TRUE(controller.active());
    const double resumed_at = 100.0 * config.phase_timeout;
    const auto still_paused = controller.update(input, resumed_at - kStep);
    EXPECT_EQ(still_paused.phase, Phase::PAUSED);
    EXPECT_EQ(still_paused.result, Result::RUNNING);

    // 暂停期间收到另一组上下文不能覆盖首次捕获的原路口和转向。
    input.context.geometry_heading = kPi;
    input.context.target_heading = -kPi / 2.0;
    input.motion_allowed = input.localization_valid = true;
    input.obstacle_blocked = false;
    const auto resumed = controller.update(input, resumed_at);
    ASSERT_EQ(resumed.phase, Phase::TURN_60);
    EXPECT_EQ(resumed.result, Result::RUNNING);
    EXPECT_DOUBLE_EQ(resumed.command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(resumed.command.angular_z, 0.0);
    const auto running = controller.update(input, resumed_at + kStep);
    EXPECT_EQ(running.phase, Phase::TURN_60);
    EXPECT_EQ(running.result, Result::RUNNING);
    EXPECT_GT(running.command.linear_x, 0.0);
    EXPECT_GT(running.command.angular_z, 0.0);

    // 恢复后仍应监督真实运动进度，暂停不能永久禁用超时。
    const auto stalled = controller.update(
        input, resumed_at + config.no_progress_timeout + kStep);
    EXPECT_EQ(stalled.result, Result::NEED_REPLAN);
    EXPECT_DOUBLE_EQ(stalled.command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(stalled.command.angular_z, 0.0);
  }
}
