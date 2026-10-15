#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "navigation_safety/checker.hpp"
#include "vehicle_loopback_sim/motion.hpp"
#include "visual_navigation/corner_return.hpp"
#include "visual_navigation/rate_limiter.hpp"

namespace {
constexpr double pi=3.14159265358979323846;
using Controller=visual_navigation::CornerReturnController;
using Phase=visual_navigation::CornerReturnPhase;
using Result=visual_navigation::TurnManeuverResult;
std::string root() {
  std::string path=__FILE__;
  for(int i=0;i<4;++i) path=path.substr(0,path.find_last_of('/'));
  return path;
}
visual_navigation::CornerReturnConfig config(bool normal=false) {
  const auto v=YAML::LoadFile(root()+"/src/vehicle_config/config/vehicle_physical.yaml");
  const auto p=YAML::LoadFile(root()+"/src/contrast_controller/config/contrast_controller.yaml")
    ["contrast_controller"]["ros__parameters"];
  const auto o=YAML::LoadFile(root()+"/src/visual_navigation/config/waypoint_navigation.yaml")
    ["waypoint_navigator"]["ros__parameters"];
  visual_navigation::CornerReturnConfig c;
  c.base_offset_x=v["differential_drive"]["base_link_offset_x_m"].as<double>();
  c.base_offset_y=v["differential_drive"]["base_link_offset_y_m"].as<double>();
  c.linear_acceleration=o["max_linear_acceleration"].as<double>();
  c.linear_deceleration=o["max_linear_deceleration"].as<double>();
  c.angular_acceleration=o["max_angular_acceleration"].as<double>();
  c.angular_deceleration=o["max_angular_deceleration"].as<double>();
  c.minimum_linear_speed=o["min_linear_command_speed"].as<double>();
  c.control_dt=1.0/o["control_frequency"].as<double>();
  auto &t=c.turn;
  t.entry_target_x=p[normal ? "turn_maneuver_entry_target_x":"recovery_turn_entry_target_x"].as<double>();
  t.entry_target_y=p[normal ? "turn_maneuver_entry_target_y":"recovery_turn_entry_target_y"].as<double>();
  t.turn_angular_speed=p[normal ? "turn_maneuver_angular_speed":"recovery_turn_angular_speed"].as<double>();
  t.turn_min_angular_speed=p[normal ? "turn_maneuver_min_angular_speed":"recovery_turn_min_angular_speed"].as<double>();
  t.creep_distance=p[normal ? "turn_maneuver_creep_distance":"recovery_turn_creep_distance"].as<double>();
  t.entry_position_tolerance=p[normal ? "turn_maneuver_entry_tolerance":"recovery_turn_entry_tolerance"].as<double>();
  t.entry_lateral_tolerance=p["turn_maneuver_entry_lateral_tolerance"].as<double>();
  t.entry_yaw_tolerance=p["turn_maneuver_entry_yaw_tolerance"].as<double>();
  t.entry_speed=p["turn_maneuver_entry_speed"].as<double>();
  t.creep_linear_speed=p["turn_maneuver_creep_speed"].as<double>();
  t.turn_linear_speed=p["turn_maneuver_turn_linear_speed"].as<double>();
  t.phase_timeout=p["turn_maneuver_phase_timeout"].as<double>();
  t.no_progress_timeout=p["turn_maneuver_no_progress_timeout"].as<double>();
  t.min_position_progress=p["turn_maneuver_min_position_progress"].as<double>();
  t.min_yaw_progress=p["turn_maneuver_min_yaw_progress"].as<double>();
  t.entry_heading_gain=p["turn_maneuver_entry_heading_gain"].as<double>();
  t.entry_lateral_gain=p["turn_maneuver_entry_lateral_gain"].as<double>();
  t.entry_max_angular_speed=p["turn_maneuver_entry_max_angular_speed"].as<double>();
  t.exit_lateral_tolerance=p["turn_maneuver_exit_lateral_tolerance"].as<double>();
  return c;
}
void apply(visual_navigation::CornerShuntContext &context,
           visual_navigation::CornerShuntTwist desired,
           vehicle_loopback_sim::Twist2d &applied,
           const visual_navigation::CornerReturnConfig &c) {
  if(std::abs(desired.linear_x)<1e-9 && std::abs(desired.angular_z)<1e-9) applied={};
  else {
    applied.linear_x=visual_navigation::LimitRate(desired.linear_x,applied.linear_x,
      c.linear_acceleration,c.linear_deceleration,c.control_dt);
    if(std::abs(applied.linear_x)<c.minimum_linear_speed)
      applied.linear_x=std::copysign(c.minimum_linear_speed,desired.linear_x);
    applied.angular_z=visual_navigation::LimitRate(desired.angular_z,applied.angular_z,
      c.angular_acceleration,c.angular_deceleration,c.control_dt);
  }
  const auto p=vehicle_loopback_sim::integrate_base_link(
    {context.pose.x,context.pose.y,context.pose.yaw},applied,c.control_dt,
    c.base_offset_x,c.base_offset_y);
  context.pose={p.x,p.y,p.yaw};context.applied_twist={applied.linear_x,applied.angular_z};
}
struct ArenaGuard {
  navigation_safety::Footprint footprint;
  std::vector<navigation_safety::Rectangle> free;
  std::vector<navigation_safety::Polygon> obstacles;
  explicit ArenaGuard(double margin=0.0) {
    const auto vehicle=YAML::LoadFile(root()+"/src/vehicle_config/config/vehicle_physical.yaml");
    footprint={vehicle["chassis"]["length_m"].as<double>(),
      vehicle["chassis"]["width_m"].as<double>(),0,0,margin};
    const auto arena=YAML::LoadFile(root()+"/src/new_map_planner/config/arena.yaml");
    const double ox=arena["frame"]["map_origin_m"][0].as<double>();
    const double oy=arena["frame"]["map_origin_m"][1].as<double>();
    const double h=arena["frame"]["map_x_heading_deg"].as<double>()*pi/180;
    const auto convert=[=](const YAML::Node &r) {
      navigation_safety::Polygon polygon;
      for(auto p:std::vector<navigation_safety::Point>{{r[0].as<double>(),r[1].as<double>()},
          {r[2].as<double>(),r[1].as<double>()},{r[2].as<double>(),r[3].as<double>()},
          {r[0].as<double>(),r[3].as<double>()}})
        polygon.points.push_back({std::cos(h)*(p.x-ox)+std::sin(h)*(p.y-oy),
          -std::sin(h)*(p.x-ox)+std::cos(h)*(p.y-oy)});
      return polygon;
    };
    for(const auto &r:arena["arena"]["obstacles"]) obstacles.push_back(convert(r));
    for(const auto &r:arena["arena"]["free_regions"]) {
      const auto polygon=convert(r);
      navigation_safety::Rectangle box{INFINITY,INFINITY,-INFINITY,-INFINITY};
      for(auto p:polygon.points) {
        box.minimum_x=std::min(box.minimum_x,p.x);box.minimum_y=std::min(box.minimum_y,p.y);
        box.maximum_x=std::max(box.maximum_x,p.x);box.maximum_y=std::max(box.maximum_y,p.y);
      }
      free.push_back(box);
    }
  }
  bool safe(visual_navigation::CornerShuntPose a,visual_navigation::CornerShuntPose b) const {
    return navigation_safety::sweep_is_safe({{{a.x,a.y},a.yaw},{{b.x,b.y},b.yaw}},
      footprint,free,obstacles,{},.003,.035);
  }
};
}

TEST(CornerReturn, LeftRightReturnToConfiguredEntranceWithRealAxleAndRateLimits) {
  const auto c=config();
  const ArenaGuard arena;
  for(double sign:{-1.0,1.0}) {
    Controller controller(c);
    visual_navigation::CornerShuntContext context;
    // 正式地图下方两个 L 路口：x0_y0 左转、x3_y0 右转。
    context.geometry_heading=0;context.target_heading=sign*pi/2;
    context.junction_x=4.2;context.junction_y=-sign*1.5;
    controller.begin(context,0.0);
    context.pose=controller.exit_pose();
    controller.begin(context,0.0);
    const auto entrance=controller.entrance_pose();
    visual_navigation::TurnManeuverController forward(c.turn);
    forward.begin(0);
    visual_navigation::TurnManeuverInput forward_input;
    forward_input.context=context;forward_input.context.pose=entrance;
    vehicle_loopback_sim::Twist2d forward_applied;
    visual_navigation::TurnManeuverDecision forward_decision;
    for(double t=0;t<60;t+=c.control_dt) {
      forward_decision=forward.update(forward_input,t);
      if(forward_decision.result!=Result::RUNNING) break;
      const auto before=forward_input.context.pose;
      apply(forward_input.context,forward_decision.command,forward_applied,c);
      ASSERT_TRUE(arena.safe(before,forward_input.context.pose));
    }
    ASSERT_EQ(forward_decision.result,Result::COMPLETED);
    EXPECT_NEAR(forward_input.context.pose.x,controller.exit_pose().x,1e-9);
    EXPECT_NEAR(forward_input.context.pose.y,controller.exit_pose().y,1e-9);
    context=forward_input.context;
    visual_navigation::CornerReturnInput input;input.context=context;
    vehicle_loopback_sim::Twist2d applied;
    std::vector<Phase> phases;
    visual_navigation::CornerReturnDecision decision;
    for(double t=0;t<60;t+=c.control_dt) {
      decision=controller.update(input,t);
      if(phases.empty() || phases.back()!=decision.phase) phases.push_back(decision.phase);
      if(decision.result!=Result::RUNNING) break;
      EXPECT_LE(decision.command.linear_x,0.0);
      if(decision.phase==Phase::TURN_30 || decision.phase==Phase::TURN_60) {
        EXPECT_LE(sign*decision.command.angular_z,0.0);
      }
      const auto before=input.context.pose;
      apply(input.context,decision.command,applied,c);
      ASSERT_TRUE(arena.safe(before,input.context.pose));
    }
    EXPECT_EQ(decision.result,Result::COMPLETED) << input.context.pose.x << "," << input.context.pose.y;
    EXPECT_NEAR(input.context.pose.x,entrance.x,c.turn.entry_position_tolerance);
    EXPECT_NEAR(input.context.pose.y,entrance.y,c.turn.entry_lateral_tolerance);
    EXPECT_NEAR(input.context.pose.yaw,entrance.yaw,c.turn.entry_yaw_tolerance);
    EXPECT_EQ(phases,(std::vector<Phase>{Phase::APPROACH_ENTRY,Phase::TURN_30,
      Phase::DIAGONAL_CREEP,Phase::TURN_60,Phase::RETURN_ENTRY,Phase::COMPLETE}));
  }
}

TEST(CornerReturn, UserBottomLMeasuredExitReturnsToFixedEntrance) {
  const auto c=config();
  const auto params=YAML::LoadFile(root()+"/src/contrast_controller/config/contrast_controller.yaml")
    ["contrast_controller"]["ros__parameters"];
  ArenaGuard arena(params["recovery_margin_m"].as<double>());
  // 同一 GUI 的三个实际障碍，验证时与正式恢复保留相同安全边距。
  for(const auto &box:std::vector<std::vector<double>>{
      {1.1,-1.0117478514809226,1.3,-.9117478514809226},
      {1.1,1.1213241552092383,1.3,1.2213241552092384},
      {4.1,-.7870374356818018,4.3,-.6870374356818018}})
    arena.obstacles.push_back({{{box[0],box[1]},{box[2],box[1]},
                                {box[2],box[3]},{box[0],box[3]}}});
  Controller controller(c);
  visual_navigation::CornerShuntContext context;
  // 2026-10-15 GUI 实测停车位姿及路线航向；正式 YAML 物理尺寸和输出限幅。
  context.junction_x=4.2;context.junction_y=-1.5;
  context.geometry_heading=6.163065084185959e-05;
  context.target_heading=pi/2;
  context.pose={4.188035907078143,-1.3804042360980853,1.5150177610665212};
  controller.begin(context,0);
  ASSERT_TRUE(controller.geometry_valid());
  const auto entry=controller.entrance_pose();
  visual_navigation::CornerReturnInput input;input.context=context;
  vehicle_loopback_sim::Twist2d applied;
  visual_navigation::CornerReturnDecision decision;
  for(double t=0;t<60;t+=c.control_dt) {
    decision=controller.update(input,t);
    if(decision.result!=Result::RUNNING) break;
    EXPECT_LE(decision.command.linear_x,0);
    const auto before=input.context.pose;
    apply(input.context,decision.command,applied,c);
    ASSERT_TRUE(arena.safe(before,input.context.pose)) << "phase="
      << visual_navigation::CornerReturnStatus(decision.phase) << " t=" << t
      << " pose=" << input.context.pose.x << "," << input.context.pose.y;
  }
  EXPECT_EQ(decision.result,Result::COMPLETED);
  const double dx=input.context.pose.x-entry.x,dy=input.context.pose.y-entry.y;
  EXPECT_LE(std::abs(dx*std::cos(entry.yaw)+dy*std::sin(entry.yaw)),c.turn.entry_position_tolerance);
  EXPECT_LE(std::abs(-dx*std::sin(entry.yaw)+dy*std::cos(entry.yaw)),c.turn.entry_lateral_tolerance);
  EXPECT_LE(std::abs(visual_navigation::NormalizeShuntAngle(input.context.pose.yaw-entry.yaw)),
            c.turn.entry_yaw_tolerance);
}

TEST(CornerReturn, PauseKeepsGeometryAndStopsClockButStallStillFails) {
  const auto c=config();Controller controller(c);
  visual_navigation::CornerShuntContext context;
  context.target_heading=pi/2;
  controller.begin(context,0.0);context.pose=controller.exit_pose();controller.begin(context,0.0);
  visual_navigation::CornerReturnInput input;input.context=context;
  controller.update(input,0);controller.update(input,c.control_dt);
  input.motion_allowed=false;
  EXPECT_EQ(controller.update(input,2*c.control_dt).phase,Phase::PAUSED);
  input.motion_allowed=true;
  EXPECT_EQ(controller.update(input,100).phase,Phase::TURN_30);
  EXPECT_EQ(controller.update(input,100+c.control_dt).result,Result::RUNNING);
  EXPECT_EQ(controller.update(input,100+c.turn.no_progress_timeout+.1).result,Result::NEED_REPLAN);
}

TEST(CornerReturn, AllowedYawCannotHideReturnEntryPositionOvershoot) {
  const auto c=config();Controller controller(c);
  visual_navigation::CornerShuntContext context;context.target_heading=pi/2;
  controller.begin(context,0);context.pose=controller.exit_pose();controller.begin(context,0);
  visual_navigation::CornerReturnInput input;input.context=context;
  vehicle_loopback_sim::Twist2d applied;
  for(double t=0;t<60;t+=c.control_dt) {
    const auto decision=controller.update(input,t);
    ASSERT_EQ(decision.result,Result::RUNNING);
    if(decision.phase==Phase::RETURN_ENTRY) {
      input.context.pose=controller.entrance_pose();
      input.context.pose.x-=2*c.turn.entry_position_tolerance;
      input.context.pose.yaw+=c.turn.entry_yaw_tolerance/2;
      EXPECT_EQ(controller.update(input,t+c.control_dt).result,Result::NEED_REPLAN);
      return;
    }
    apply(input.context,decision.command,applied,c);
  }
  FAIL() << "未执行到原入口返回阶段";
}

TEST(CornerReturn, MissingPhysicalParametersFailExplicitly) {
  Controller controller;
  visual_navigation::CornerShuntContext context;context.target_heading=pi/2;
  controller.begin(context,0.0);
  visual_navigation::CornerReturnInput input;input.context=context;
  EXPECT_EQ(controller.update(input,0).result,Result::FAULT);
}

TEST(CornerReturn, ExistingNormalTemplateFailureIsNotReportedAsReturnSuccess) {
  Controller controller(config(true));
  visual_navigation::CornerShuntContext context;context.target_heading=pi/2;
  controller.begin(context,0.0);
  // 现有正常 .35/.15 参数的固定动作出口校验失败，不能伪造返回模板。
  EXPECT_FALSE(controller.geometry_valid());
  visual_navigation::CornerReturnInput input;input.context=context;
  EXPECT_EQ(controller.update(input,0).result,Result::FAULT);
}
