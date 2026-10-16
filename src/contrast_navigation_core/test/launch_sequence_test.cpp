#include "visual_navigation/launch_sequence.hpp"

#include <cmath>
#include <filesystem>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace {
using visual_navigation::CornerShuntPose;
using visual_navigation::LaunchSequence;
using Phase = LaunchSequence::Phase;
constexpr double kStep = 0.02;
constexpr double kPi = 3.14159265358979323846;

YAML::Node Parameters() {
  const auto path = std::filesystem::path(__FILE__).parent_path().parent_path()
                        .parent_path() / "final_navigation/config/final_navigation.yaml";
  return YAML::LoadFile(path.string())["final_navigation"]["ros__parameters"];
}

// 数值取自正式 YAML；0.20 m 路口来自 arena.yaml 的出发道路。
struct Fixture {
  YAML::Node parameters{Parameters()};
  CornerShuntPose origin{
    parameters["launch_origin"][0].as<double>(),
    parameters["launch_origin"][1].as<double>(),
    parameters["launch_origin"][2].as<double>()};
  double probe{parameters["launch_probe_distance_m"].as<double>()};
  double tolerance{parameters["launch_position_tolerance_m"].as<double>()};
  double timeout{parameters["launch_phase_timeout_s"].as<double>()};
  double speed{parameters["launch_speed_mps"].as<double>()};
  double entry{0.20 + parameters["turn_maneuver_entry_target_x"].as<double>()};
  LaunchSequence sequence;

  Fixture() { sequence.begin(origin, probe, entry, tolerance, timeout); }
};

TEST(LaunchSequence, EveryDirectionChangeRequiresMeasuredStop) {
  Fixture f;
  EXPECT_EQ(f.sequence.phase(), Phase::INITIAL_STOP);
  f.sequence.update(f.origin, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::INITIAL_STOP);
  f.sequence.update(f.origin, true, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::FORWARD);

  auto pose = f.origin;
  pose.x += f.probe;
  f.sequence.update(pose, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::FORWARD_STOP);
  f.sequence.update(pose, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::FORWARD_STOP);
  EXPECT_EQ(f.sequence.direction(), 1);
  f.sequence.update(pose, true, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::RETURN);
  EXPECT_EQ(f.sequence.direction(), -1);

  f.sequence.update(f.origin, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::ORIGIN_STOP);
  f.sequence.update(f.origin, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::ORIGIN_STOP);
  EXPECT_TRUE(f.sequence.stopping());
  f.sequence.update(f.origin, true, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::ENTRY);
  EXPECT_EQ(f.sequence.direction(), 1);

  pose.x = f.origin.x + f.entry;
  f.sequence.update(pose, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::ENTRY_STOP);
  f.sequence.update(pose, false, kStep);
  EXPECT_TRUE(f.sequence.active());
  f.sequence.update(pose, true, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::COMPLETE);
  EXPECT_FALSE(f.sequence.active());
}

TEST(LaunchSequence, FeedbackRoundTripStopsAtEntryAndStartsRealTurnWithoutReverse) {
  Fixture f;
  auto pose = f.origin;
  std::vector<Phase> phases{f.sequence.phase()};
  double return_x = 1.0;
  for (int tick = 0; f.sequence.active() && tick < 3000; ++tick) {
    const auto before = f.sequence.phase();
    // 实测位姿由每帧实际执行速度积分，停稳阶段连续三帧不移动后才放行。
    static constexpr int kStopSamples = 3;
    int stopped_samples = 0;
    if (f.sequence.stopping()) {
      for (; stopped_samples < kStopSamples - 1; ++stopped_samples) {
        f.sequence.update(pose, false, kStep);
        ASSERT_EQ(f.sequence.phase(), before);
      }
    } else {
      pose.x += f.sequence.direction() * f.speed * kStep * std::cos(f.origin.yaw);
      pose.y += f.sequence.direction() * f.speed * kStep * std::sin(f.origin.yaw);
    }
    f.sequence.update(pose, f.sequence.stopping(), kStep);
    if (before != f.sequence.phase()) {
      phases.push_back(f.sequence.phase());
      if (f.sequence.phase() == Phase::ORIGIN_STOP) return_x = pose.x;
    }
  }
  ASSERT_EQ(f.sequence.phase(), Phase::COMPLETE);
  EXPECT_EQ(phases, (std::vector<Phase>{Phase::INITIAL_STOP, Phase::FORWARD,
    Phase::FORWARD_STOP, Phase::RETURN, Phase::ORIGIN_STOP, Phase::ENTRY,
    Phase::ENTRY_STOP, Phase::COMPLETE}));
  EXPECT_NEAR(return_x, f.origin.x, f.tolerance);
  EXPECT_NEAR(pose.x, f.origin.x + 0.12, f.tolerance);

  visual_navigation::TurnManeuverConfig config;
  config.entry_target_x = f.parameters["turn_maneuver_entry_target_x"].as<double>();
  config.entry_target_y = f.parameters["turn_maneuver_entry_target_y"].as<double>();
  config.turn_linear_speed = f.parameters["turn_maneuver_turn_linear_speed"].as<double>();
  config.entry_position_tolerance = f.parameters["turn_maneuver_entry_tolerance"].as<double>();
  visual_navigation::TurnManeuverController turn(config);
  visual_navigation::TurnManeuverInput input;
  input.context.pose = pose;
  input.context.junction_x = f.origin.x + 0.20;
  input.context.junction_y = f.origin.y;
  input.context.geometry_heading = f.origin.yaw;
  input.context.target_heading = f.origin.yaw - kPi / 2.0;
  input.context.arrival_heading = pose.yaw;
  turn.begin(0.0);
  bool turning = false;
  for (int tick = 0; tick < 5; ++tick) {
    const auto decision = turn.update(input, tick * kStep);
    ASSERT_EQ(decision.result, visual_navigation::TurnManeuverResult::RUNNING);
    EXPECT_GE(decision.command.linear_x, 0.0);
    if (decision.phase == visual_navigation::TurnManeuverPhase::TURN_60 &&
        decision.command.angular_z < 0.0) turning = true;
  }
  EXPECT_TRUE(turning);
}

TEST(LaunchSequence, TimeoutStopsSequenceAndPausedTimeIsExcluded) {
  Fixture f;
  f.sequence.update(f.origin, true, kStep);
  ASSERT_EQ(f.sequence.phase(), Phase::FORWARD);
  f.sequence.update(f.origin, false, f.timeout / 2.0);
  const double active_time = f.sequence.time();
  // 障碍或暂停时调用方不 update；读取状态不会把墙上时钟计入动作超时。
  for (int paused_tick = 0; paused_tick < 10000; ++paused_tick) {
    EXPECT_EQ(f.sequence.phase(), Phase::FORWARD);
    ASSERT_DOUBLE_EQ(f.sequence.time(), active_time);
  }
  f.sequence.update(f.origin, false, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::FORWARD);
  f.sequence.update(f.origin, false, f.timeout / 2.0);
  EXPECT_EQ(f.sequence.phase(), Phase::FAULT);
  EXPECT_FALSE(f.sequence.active());
  f.sequence.update(f.origin, true, kStep);
  EXPECT_EQ(f.sequence.phase(), Phase::FAULT);
}

TEST(LaunchSequence, TargetsFollowOriginHeading) {
  Fixture f;
  f.origin = {0.3, -0.2, kPi / 2.0};
  f.sequence.begin(f.origin, f.probe, f.entry, f.tolerance, f.timeout);
  f.sequence.update(f.origin, true, kStep);
  auto pose = f.origin;
  pose.y += f.probe;
  EXPECT_NEAR(f.sequence.remaining(pose), 0.0, 1e-12);
  f.sequence.update(pose, false, kStep);
  f.sequence.update(pose, true, kStep);
  EXPECT_NEAR(f.sequence.remaining(pose), f.probe, 1e-12);
  EXPECT_EQ(f.sequence.direction(), -1);
}
}  // namespace
