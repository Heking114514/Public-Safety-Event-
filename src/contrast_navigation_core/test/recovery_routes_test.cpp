#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>
#include <cstdio>
#include <unistd.h>
#include "visual_navigation/pose_recovery.hpp"
#include "recovery_replay.hpp"

namespace {
using Recovery=visual_navigation::PoseRecovery;
using Candidate=mission_control_interfaces::msg::RecoveryCandidate;
constexpr double pi=3.14159265358979323846;

// 所有道路、底盘和动作参数读取隔离工作树的正式 YAML，不生成替代地图。
std::string workspace() {
  std::string path=__FILE__;
  for(int i=0;i<4;++i) path=path.substr(0,path.find_last_of('/'));
  return path;
}
Recovery make_recovery() {
  const auto root=workspace();
  const auto vehicle=YAML::LoadFile(root+"/src/vehicle_config/config/vehicle_physical.yaml");
  const auto parameters=YAML::LoadFile(root+"/src/contrast_controller/config/contrast_controller.yaml")
    ["contrast_controller"]["ros__parameters"];
  const auto output=YAML::LoadFile(root+"/src/visual_navigation/config/waypoint_navigation.yaml")
    ["waypoint_navigator"]["ros__parameters"];
  Recovery::Config config;
  config.speed=parameters["recovery_linear_speed"].as<double>();
  config.margin=parameters["recovery_margin_m"].as<double>();
  config.max_radius=parameters["recovery_max_radius_m"].as<double>();
  config.exit_distance=parameters["recovery_exit_distance_m"].as<double>();
  config.forward_obstacle_clearance_m=parameters["recovery_forward_obstacle_clearance_m"].as<double>();
  config.turn.entry_target_x=parameters["recovery_turn_entry_target_x"].as<double>();
  config.turn.entry_target_y=parameters["recovery_turn_entry_target_y"].as<double>();
  config.turn.entry_speed=parameters["turn_maneuver_entry_speed"].as<double>();
  config.turn.turn_linear_speed=parameters["turn_maneuver_turn_linear_speed"].as<double>();
  config.turn.turn_angular_speed=parameters["recovery_turn_angular_speed"].as<double>();
  config.turn.turn_min_angular_speed=parameters["recovery_turn_min_angular_speed"].as<double>();
  config.turn.creep_linear_speed=parameters["turn_maneuver_creep_speed"].as<double>();
  config.turn.creep_distance=parameters["recovery_turn_creep_distance"].as<double>();
  config.turn.entry_position_tolerance=parameters["recovery_turn_entry_tolerance"].as<double>();
  config.turn.entry_lateral_tolerance=parameters["turn_maneuver_entry_lateral_tolerance"].as<double>();
  config.turn.entry_yaw_tolerance=parameters["turn_maneuver_entry_yaw_tolerance"].as<double>();
  config.turn.exit_lateral_tolerance=parameters["turn_maneuver_exit_lateral_tolerance"].as<double>();
  config.turn.phase_timeout=parameters["turn_maneuver_phase_timeout"].as<double>();
  config.turn.no_progress_timeout=parameters["turn_maneuver_no_progress_timeout"].as<double>();
  config.turn.min_position_progress=parameters["turn_maneuver_min_position_progress"].as<double>();
  config.turn.min_yaw_progress=parameters["turn_maneuver_min_yaw_progress"].as<double>();
  config.turn.entry_heading_gain=parameters["turn_maneuver_entry_heading_gain"].as<double>();
  config.turn.entry_lateral_gain=parameters["turn_maneuver_entry_lateral_gain"].as<double>();
  config.turn.entry_max_angular_speed=parameters["turn_maneuver_entry_max_angular_speed"].as<double>();
  config.linear_acceleration=output["max_linear_acceleration"].as<double>();
  config.linear_deceleration=output["max_linear_deceleration"].as<double>();
  config.minimum_linear_speed=output["min_linear_command_speed"].as<double>();
  config.angular_acceleration=output["max_angular_acceleration"].as<double>();
  config.angular_deceleration=output["max_angular_deceleration"].as<double>();
  config.control_dt=1.0/output["control_frequency"].as<double>();
  config.base_offset_x=vehicle["differential_drive"]["base_link_offset_x_m"].as<double>();
  config.base_offset_y=vehicle["differential_drive"]["base_link_offset_y_m"].as<double>();
  Recovery recovery;
  recovery.load(root+"/src/new_map_planner/config/arena.yaml",
    {vehicle["chassis"]["length_m"].as<double>(),vehicle["chassis"]["width_m"].as<double>(),0,0,0},config);
  return recovery;
}
auto obstacle() -> navigation_safety::Polygon {
  return navigation_safety::rectangle_polygon({1.11,-1.06,1.29,-0.98});
}
const Recovery::Pose x_start{{1.2,-.66},-pi/2};

const Candidate *find(const std::vector<Candidate> &candidates,const std::string &name) {
  const auto it=std::find_if(candidates.begin(),candidates.end(),[&](const Candidate &candidate) {
    return candidate.template_id==name;
  });
  return it==candidates.end() ? nullptr : &*it;
}

void validate_route(Recovery &recovery,const Recovery::Pose &start,const Candidate &candidate,
                    std::size_t expected_turns) {
  ASSERT_TRUE(recovery.plan(start,candidate,{recovery.blocked_lane()}));
  const auto &steps=recovery.steps();
  ASSERT_FALSE(steps.empty());
  EXPECT_EQ(std::count_if(steps.begin(),steps.end(),[](const Recovery::Step &step) {
    return step.kind==Recovery::Step::Kind::FIXED_TURN;
  }),static_cast<int>(expected_turns));
  EXPECT_TRUE(recovery.exit_matches(steps.back().target,candidate));
  for(const auto &step:steps) {
    EXPECT_TRUE(recovery.safe({step.start}));
    EXPECT_TRUE(recovery.safe({step.end}));
    if(step.kind==Recovery::Step::Kind::FIXED_TURN) {
      EXPECT_NEAR(std::abs(std::atan2(std::sin(step.target_heading-step.geometry_heading),
        std::cos(step.target_heading-step.geometry_heading))),pi/2,1e-9);
    }
  }
  if(expected_turns==2) {
    std::vector<std::string> phases;
    for(const auto &step:steps) phases.push_back(step.phase);
    const auto retreat=std::find(phases.begin(),phases.end(),"RETREAT_TO_SECOND_TURN_POINT");
    ASSERT_NE(retreat,phases.end());
    ASSERT_NE(retreat,phases.begin());
    ASSERT_NE(retreat+1,phases.end());
    EXPECT_EQ(*(retreat-1),"DIRECTION_STOP");
    EXPECT_EQ(*(retreat+1),"DIRECTION_STOP");
  }
  RecoveryReplay replay(recovery,workspace());
  ASSERT_TRUE(replay.run(start,steps)) << replay.failure();
  EXPECT_EQ(replay.turns60(),expected_turns);
  EXPECT_EQ(replay.turns30(),expected_turns);
  EXPECT_TRUE(recovery.exit_matches(replay.actual(),candidate));
}
}

TEST(RecoveryRoutes, OfficialCrossHasThreeExecutableDirections) {
  auto recovery=make_recovery();
  const auto candidates=recovery.candidates(x_start,obstacle());
  for(const auto &name:{"x_junction_left","x_junction_right","x_junction_reverse"}) {
    const auto candidate=find(candidates,name);
    ASSERT_NE(candidate,nullptr) << name;
    validate_route(recovery,x_start,*candidate,std::string(name).find("reverse")!=std::string::npos ? 2 : 1);
  }
}

TEST(RecoveryRoutes, OfficialTUsesOneTurnAndRejectsUnsafeSevenCmSecondEntry) {
  auto recovery=make_recovery();
  const Recovery::Pose start{{2.36,-1.5},0};
  const auto candidates=recovery.candidates(start,
    navigation_safety::rectangle_polygon({2.68,-1.59,2.76,-1.41}));
  const auto candidate=find(candidates,"t_junction_left");
  ASSERT_NE(candidate,nullptr);
  validate_route(recovery,start,*candidate,1);
  // x0_y2 的旁支朝 +y，第二次前进转弯的固定入口在 -y 侧。
  // 7 cm 入口加 6.25 cm 半车身越过正式场地 y=-1.6 边界，必须拒绝。
  const Recovery::Pose second_entry{{2.2,-1.5+recovery.config().turn.entry_target_x},pi/2};
  EXPECT_FALSE(recovery.safe({second_entry}));
  EXPECT_EQ(find(candidates,"t_junction_reverse"),nullptr);
}

TEST(RecoveryRoutes, OfficialLongRetreatIsNotTruncatedByOldRadius) {
  auto recovery=make_recovery();
  const Recovery::Pose start{{1.2,-1.36},-pi/2};
  const auto candidates=recovery.candidates(start,
    navigation_safety::rectangle_polygon({1.11,-1.49,1.29,-1.47}));
  ASSERT_FALSE(candidates.empty());
  ASSERT_TRUE(recovery.plan(start,candidates.front(),{recovery.blocked_lane()}));
  double reverse_distance=0;
  for(const auto &step:recovery.steps()) if(step.kind==Recovery::Step::Kind::STRAIGHT && step.linear<0)
    reverse_distance+=std::hypot(step.end.position.x-step.start.position.x,
                                step.end.position.y-step.start.position.y);
  EXPECT_GT(reverse_distance,0.85);
  RecoveryReplay replay(recovery,workspace());
  ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  EXPECT_GT(replay.reverse_distance(),0.85);
  EXPECT_TRUE(recovery.exit_matches(replay.actual(),candidates.front()));
}

TEST(RecoveryRoutes, NewObstacleInvalidatesPreviouslySafeExit) {
  auto recovery=make_recovery();
  const auto initial=obstacle();
  const auto candidates=recovery.candidates(x_start,initial);
  ASSERT_FALSE(candidates.empty());
  const auto candidate=candidates.front();
  ASSERT_TRUE(recovery.plan(x_start,candidate,{recovery.blocked_lane()}));
  const auto exit=recovery.steps().back().target;
  ASSERT_TRUE(recovery.exit_matches(exit,candidate));
  const auto added=navigation_safety::rectangle_polygon({exit.position.x-.03,exit.position.y-.03,
    exit.position.x+.03,exit.position.y+.03});
  recovery.set_dynamic_obstacles({initial,added});
  EXPECT_FALSE(recovery.safe({exit}));
  EXPECT_FALSE(recovery.exit_matches(exit,candidate));
  EXPECT_FALSE(recovery.plan(x_start,candidate,{recovery.blocked_lane()}));
}

TEST(RecoveryRoutes, BlockedBorrowedBranchRejectsReverseBeforeMotion) {
  auto recovery=make_recovery();
  const auto candidates=recovery.candidates(x_start,obstacle());
  const auto candidate=find(candidates,"x_junction_reverse");
  ASSERT_NE(candidate,nullptr);
  ASSERT_FALSE(candidate->borrowed_lane_ids.empty());
  EXPECT_FALSE(recovery.plan(x_start,*candidate,{candidate->borrowed_lane_ids.front()}));
  EXPECT_TRUE(recovery.steps().empty());
}

TEST(RecoveryRoutes, CompletionRejectsWrongSideWrongLaneAndWrongHeading) {
  auto recovery=make_recovery();
  const auto candidates=recovery.candidates(x_start,obstacle());
  ASSERT_FALSE(candidates.empty());
  const auto candidate=candidates.front();
  ASSERT_TRUE(recovery.plan(x_start,candidate,{}));
  const auto exit=recovery.steps().back().target;
  ASSERT_TRUE(recovery.exit_matches(exit,candidate));
  auto wrong=exit;
  wrong.position={2*1.2-exit.position.x,2*(-.5)-exit.position.y};
  EXPECT_FALSE(recovery.exit_matches(wrong,candidate));
  wrong=exit; wrong.yaw+=pi;
  EXPECT_FALSE(recovery.exit_matches(wrong,candidate));
  auto fictional=candidate; fictional.resume_lane="nonexistent";
  EXPECT_FALSE(recovery.exit_matches(exit,fictional));
  fictional=candidate; fictional.resume_node="x0_y0";
  EXPECT_FALSE(recovery.exit_matches(exit,fictional));
}

TEST(RecoveryRoutes, PhysicalJunctionClassificationUsesAnglesAndUniqueMouths) {
  auto recovery=make_recovery();
  using Type=Recovery::RoadJunction;
  EXPECT_EQ(recovery.junction_type("launch"),Type::DEAD_END);
  EXPECT_EQ(recovery.junction_type("x0_y0"),Type::CORNER);
  EXPECT_EQ(recovery.junction_type("x0_y2"),Type::T);
  EXPECT_EQ(recovery.junction_type("x1_y3"),Type::CROSS);
  EXPECT_EQ(recovery.junction_at({4.2,-1.5}),Type::CORNER);
  EXPECT_EQ(recovery.junction_at({4.2,-1.499}),Type::INVALID);
  // 该夹具仅检验道路角度和重复逻辑边，坐标不作为生产动作配置。
  const std::string path="/tmp/recovery_topology_"+std::to_string(getpid())+".yaml";
  {
    std::ofstream file(path);
    file << "frame: {map_origin_m: [0, 0], map_x_heading_deg: 0}\n"
         << "arena: {free_regions: [[-2, -2, 2, 2]], obstacles: []}\n"
         << "nodes: {a: [0, 0], b: [1, 0], c: [-1, 0], d: [0.5, 0.5]}\n"
         << "lanes:\n"
         << "  - {id: ab, from: a, to: b, width_m: 0.2}\n"
         << "  - {id: ac, from: a, to: c, width_m: 0.2}\n"
         << "  - {id: ab_duplicate, from: a, to: b, width_m: 0.2}\n"
         << "  - {id: bd, from: b, to: d, width_m: 0.2}\n";
  }
  recovery.load(path,{0.125,0.125,0,0,0},recovery.config());
  std::remove(path.c_str());
  EXPECT_EQ(recovery.junction_type("a"),Type::STRAIGHT);
  EXPECT_EQ(recovery.junction_type("b"),Type::INVALID);
  const auto candidates=recovery.candidates({{.2,0},0},
    navigation_safety::rectangle_polygon({.7,-.05,.8,.05}));
  EXPECT_TRUE(candidates.empty());
}

TEST(RecoveryRoutes, OfficialCornerAndDeadEndCannotProduceTJunctionCandidates) {
  auto recovery=make_recovery();
  // x0_y0 是实际地图90度拐角；它不具备丁字恢复所需的第三个路口方向。
  // 从拐角沿实际 +y 道路前进，后退锚点为 x0_y0。
  auto candidates=recovery.candidates({{4.2,-1.34},pi/2},
    navigation_safety::rectangle_polygon({4.11,-1.02,4.29,-.94}));
  EXPECT_TRUE(candidates.empty());
  Candidate fabricated;
  fabricated.template_id="t_junction_left";
  fabricated.resume_node="x0_y0"; fabricated.resume_lane="ROAD_X0_Y0";
  fabricated.resume_heading=pi;
  EXPECT_FALSE(recovery.plan({{4.2,-1.34},pi/2},fabricated,{}));
  candidates=recovery.candidates({{.1,0},0},
    navigation_safety::rectangle_polygon({.17,-.03,.19,.03}));
  EXPECT_TRUE(candidates.empty());
}

TEST(RecoveryRoutes, EachFixedTurnHasExactlyTheConfiguredSignedEntry) {
  auto recovery=make_recovery();
  const auto candidates=recovery.candidates(x_start,obstacle());
  ASSERT_FALSE(candidates.empty());
  for (const auto &candidate : candidates) {
    ASSERT_TRUE(recovery.plan(x_start,candidate,{}));
    const auto &steps=recovery.steps();
    for (std::size_t i=0; i<steps.size(); ++i) {
      const auto &turn=steps[i];
      if (turn.kind!=Recovery::Step::Kind::FIXED_TURN) continue;
      ASSERT_GT(i,0u);
      const bool preceded_by_stop=steps[i-1].kind==Recovery::Step::Kind::STOP;
      if (preceded_by_stop) { ASSERT_GE(i,2u); }
      const auto &entry=preceded_by_stop ? steps[i-2] : steps[i-1];
      const double c=std::cos(turn.geometry_heading), s=std::sin(turn.geometry_heading);
      const double dx=entry.target.position.x-turn.junction.x;
      const double dy=entry.target.position.y-turn.junction.y;
      EXPECT_NEAR(dx*c+dy*s,recovery.config().turn.entry_target_x,1e-12);
      const double sign=std::sin(turn.target_heading-turn.geometry_heading)>0 ? 1 : -1;
      EXPECT_NEAR(-dx*s+dy*c,sign*recovery.config().turn.entry_target_y,1e-12);
    }
  }
}

TEST(RecoveryRoutes, ForwardAdjustmentCanMoveTowardObstacleBeforeRetreat) {
  auto recovery=make_recovery();
  // 恢复入口附近发生位姿修正，固定入路仍由已锁存道路提供。
  const Recovery::Pose start{{1.2,-.31},-pi/2};
  recovery.set_incoming_lane("ROAD_Y3_LEFT",{1.2,-.5});
  auto config=recovery.config(); config.forward_obstacle_clearance_m=.5;
  recovery.load(workspace()+"/src/new_map_planner/config/arena.yaml",
    {0.125,0.125,0,0,0},config); // 尺寸来源 vehicle_physical.yaml。
  recovery.set_incoming_lane("ROAD_Y3_LEFT",{1.2,-.5});
  const auto obstacle=navigation_safety::rectangle_polygon({1.11,-.86,1.29,-.78});
  const auto distant=recovery.candidates(start,obstacle);
  EXPECT_TRUE(std::none_of(distant.begin(),distant.end(),[](const Candidate &c) {
    return c.resume_node=="x1_y3";
  }));
  config.forward_obstacle_clearance_m=.03; // 用户要求的车身外廓净距。
  recovery.load(workspace()+"/src/new_map_planner/config/arena.yaml",
    {0.125,0.125,0,0,0},config);
  recovery.set_incoming_lane("ROAD_Y3_LEFT",{1.2,-.5});
  const auto candidates=recovery.candidates(start,obstacle);
  ASSERT_FALSE(candidates.empty());
  // 总控会把当前障碍道路列入 blocked；允许在真实障碍前的安全部分调整。
  ASSERT_TRUE(recovery.plan(start,candidates.front(),{recovery.blocked_lane()}));
  ASSERT_GE(recovery.steps().size(),3U);
  EXPECT_EQ(recovery.steps()[0].phase,"FORWARD_ADJUSTMENT");
  EXPECT_GT(recovery.steps()[0].linear,0);
  EXPECT_LT(recovery.steps()[0].end.position.y,start.position.y);
  EXPECT_GT(start.position.y-recovery.steps()[0].end.position.y,.08);
  EXPECT_EQ(recovery.steps()[1].kind,Recovery::Step::Kind::STOP);
  EXPECT_LT(recovery.steps()[2].linear,0);
  RecoveryReplay replay(recovery,workspace());
  ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  EXPECT_TRUE(recovery.exit_matches(replay.actual(),candidates.front()));
  ASSERT_FALSE(replay.adjustment_trace().empty());
  for (const auto &actual:replay.adjustment_trace()) {
    // 障碍最近边 y=-.78；独立按实车正方形外廓计算最低角点，包含偏航。
    const double half_extent=.0625*(std::abs(std::cos(actual.yaw))+std::abs(std::sin(actual.yaw)));
    EXPECT_GE(actual.position.y-half_extent-(-.78),.03);
  }
}

TEST(RecoveryRoutes, UserGuiBoundaryEntryIsUnsafeEvenAfterForwardAdjustment) {
  auto recovery=make_recovery();
  // GUI 停车真值与手放障碍快照：本次入口 y=1.57，车身越过 y=1.6。
  const Recovery::Pose start{{3.2012432507790223,1.3348632509400973},-1.5942060352318406};
  const auto candidates=recovery.candidates(start,
    navigation_safety::rectangle_polygon({3.1,.9330065359477123,3.3,1.0330065359477123}));
  EXPECT_TRUE(candidates.empty());
  EXPECT_TRUE(recovery.unsafe_entry_rejected());
  EXPECT_FALSE(recovery.safe({{{3.2,1.57},-pi/2}}));
  EXPECT_TRUE(recovery.steps().empty());
}

TEST(RecoveryRoutes, ReturnedCrossEntryUsesOtherExitsInsteadOfUnsafeRearBoundaryT) {
  auto recovery=make_recovery();
  // 用户第二次避障：十字路口转东遇障、倒转回朝北的原入口。
  const Recovery::Pose start{{2.193,-.569},1.568};
  const auto east_obstacle=navigation_safety::rectangle_polygon({2.464,-.60,2.584,-.40});
  const auto previous_obstacle=navigation_safety::rectangle_polygon({2.579,-1.60,2.699,-1.40});
  recovery.set_dynamic_obstacles({previous_obstacle,east_obstacle});
  recovery.set_incoming_lane("ROAD_Y2_LEFT",{2.2,-.5});
  const auto candidates=recovery.candidates(start,east_obstacle);
  const auto west=find(candidates,"x_junction_left");
  ASSERT_NE(west,nullptr);
  EXPECT_EQ(west->resume_node,"x1_y2");
  EXPECT_EQ(west->resume_lane,"ROAD_X1_Y2");
  ASSERT_TRUE(recovery.plan(start,*west,{"ROAD_X0_Y1","ROAD_X1_Y1"}));
  for(const auto &step:recovery.steps()) {
    if(step.kind==Recovery::Step::Kind::FIXED_TURN) {
      EXPECT_NEAR(step.junction.x,2.2,1e-9);
      EXPECT_NEAR(step.junction.y,-.5,1e-9);
    }
  }
  RecoveryReplay replay(recovery,workspace());
  ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  EXPECT_TRUE(recovery.exit_matches(replay.actual(),*west));
}

TEST(RecoveryRoutes, KnownBlockedCornerAllowsLongRetreatToRearT) {
  auto recovery=make_recovery();
  const Recovery::Pose start{{1.239,1.5},pi};
  // 最新 GUI 的两块实际障碍；前方 L 连通道路已封，不要求再次触发视线检测。
  const std::vector<navigation_safety::Polygon> obstacles{
    navigation_safety::rectangle_polygon({1.47,-1.60,1.59,-1.40}),
    navigation_safety::rectangle_polygon({1.10,1.111,1.30,1.231})};
  const std::vector<std::string> blocked{"ROAD_X0_Y2","ROAD_Y3_RIGHT"};
  EXPECT_EQ(recovery.forward_obstacle(start,obstacles,blocked),1);
  EXPECT_EQ(recovery.forward_obstacle(start,obstacles,{}),-1);
  EXPECT_EQ(recovery.forward_obstacle(start,{obstacles.front()},blocked),-1);
  EXPECT_EQ(recovery.forward_obstacle({start.position,0.0},obstacles,blocked),-1);
  recovery.set_dynamic_obstacles(obstacles);
  const auto candidates=recovery.candidates(start,obstacles[1]);
  const auto south=find(candidates,"t_junction_left");
  ASSERT_NE(south,nullptr);
  EXPECT_EQ(south->resume_node,"x3_y2");
  EXPECT_EQ(south->resume_lane,"ROAD_Y2_RIGHT");
  ASSERT_TRUE(recovery.plan(start,*south,blocked));
  double reverse=0;
  for (const auto &step:recovery.steps()) {
    if (step.phase=="RETREAT_TO_TURN_POINT") reverse+=std::abs(step.linear)*step.duration;
    if (step.kind==Recovery::Step::Kind::FIXED_TURN) {
      EXPECT_NEAR(step.junction.x,2.2,1e-9);
      EXPECT_NEAR(step.junction.y,1.5,1e-9);
    }
  }
  EXPECT_GT(reverse,1.0);
  RecoveryReplay replay(recovery,workspace());
  ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  EXPECT_TRUE(recovery.exit_matches(replay.actual(),*south));
}

TEST(RecoveryRoutes, LayerTwoEntryRecognizesBlockageBeyondLAndLaunchSpur) {
  auto recovery=make_recovery();
  // 2026-10-15 GUI 故障真值：12 个 RFID 已完成，进入隧道层。
  const Recovery::Pose start{{.23876371210204264,.49981146930033304},-3.140625147480947};
  const std::vector<navigation_safety::Polygon> obstacles{
    navigation_safety::rectangle_polygon({.1,-.55,.3,-.45}),
    navigation_safety::rectangle_polygon({1.1,-1.2263966068711274,1.3,-1.1263966068711273})};
  const std::vector<std::string> blocked{"ROAD_X1_Y3","ROAD_Y3_LEFT"};
  ASSERT_EQ(recovery.forward_obstacle(start,obstacles,blocked),0);
  EXPECT_EQ(recovery.forward_obstacle(start,obstacles,{}),-1);
  EXPECT_EQ(recovery.forward_obstacle(start,{obstacles[1]},blocked),-1);
  EXPECT_EQ(recovery.forward_obstacle({start.position,0},obstacles,blocked),-1);
  // 前方十字存在可用岔路，不能借用该远处障碍授权长倒车。
  EXPECT_EQ(recovery.forward_obstacle({{2.4,.5},pi},obstacles,blocked),-1);
  recovery.set_dynamic_obstacles(obstacles);
  const auto candidates=recovery.candidates(start,obstacles[0]);
  ASSERT_FALSE(candidates.empty());
  bool executable=false;
  for (const auto &candidate:candidates) {
    if (!recovery.plan(start,candidate,blocked)) continue;
    // 应退回 x=1.2 的 T/X 固定入口，不能在当前 L 直接前进掉头。
    ASSERT_FALSE(recovery.steps().empty());
    EXPECT_LT(recovery.steps().front().linear,0);
    RecoveryReplay replay(recovery,workspace());
    ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
    EXPECT_GT(replay.reverse_distance(),.8);
    EXPECT_TRUE(recovery.exit_matches(replay.actual(),candidate));
    executable=true;
    break;
  }
  EXPECT_TRUE(executable);
}

TEST(RecoveryRoutes, ForwardAdjustmentStopsAtBodyClearanceRatherThanCameraRange) {
  auto recovery=make_recovery();
  recovery.set_dynamic_obstacles({navigation_safety::rectangle_polygon({1.1,-.86,1.3,-.78})});
  // 车身长 0.125 m，来自 vehicle_physical.yaml；中心到障碍还有 9.25 cm 时净距为 3 cm。
  const Recovery::Pose outside{{1.2,-.6874},-pi/2},inside{{1.2,-.6876},-pi/2};
  EXPECT_NEAR(recovery.obstacle_clearance(outside),.0301,1e-9);
  EXPECT_NEAR(recovery.obstacle_clearance(inside),.0299,1e-9);
  EXPECT_TRUE(recovery.adjustment_safe({outside}));
  EXPECT_TRUE(recovery.safe({inside}));
  EXPECT_FALSE(recovery.adjustment_safe({inside}));
  // 车身有偏航时，最近的角点决定净距。
  EXPECT_FALSE(recovery.adjustment_safe({{{1.2,-.68},-pi/2+.2}}));
}

TEST(RecoveryRoutes, ClearanceLimitRejectsOtherwiseCollisionFreeAdjustment) {
  auto recovery=make_recovery();
  recovery.set_incoming_lane("ROAD_Y3_LEFT",{1.2,-.5});
  const auto candidates=recovery.candidates({{1.2,-.31},-pi/2},
    navigation_safety::rectangle_polygon({1.11,-.54,1.29,-.52}));
  EXPECT_TRUE(std::none_of(candidates.begin(),candidates.end(),[](const Candidate &c) {
    return c.resume_node=="x1_y3";
  }));
  // 更后方候选可以直接倒退，但不得向障碍前移。
  for (const auto &candidate:candidates) {
    const Recovery::Pose start{{1.2,-.31},-pi/2};
    ASSERT_TRUE(recovery.plan(start,candidate,{}));
    for (const auto &step:recovery.steps()) EXPECT_NE(step.phase,"FORWARD_ADJUSTMENT");
    RecoveryReplay replay(recovery,workspace());
    ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  }

}

TEST(RecoveryRoutes, ForwardAdjustmentCannotPassThroughObstacle) {
  auto recovery=make_recovery();
  recovery.set_incoming_lane("ROAD_Y3_LEFT",{1.2,-.5});
  const auto candidates=recovery.candidates({{1.2,-.41},-pi/2},
    navigation_safety::rectangle_polygon({1.11,-.56,1.29,-.54}));
  EXPECT_TRUE(std::none_of(candidates.begin(),candidates.end(),[](const Candidate &c) {
    return c.resume_node=="x1_y3";
  }));
  // 更后方候选可以直接倒退，但不得向障碍前移。
  for (const auto &candidate:candidates) {
    const Recovery::Pose start{{1.2,-.41},-pi/2};
    ASSERT_TRUE(recovery.plan(start,candidate,{}));
    for (const auto &step:recovery.steps()) EXPECT_NE(step.phase,"FORWARD_ADJUSTMENT");
    RecoveryReplay replay(recovery,workspace());
    ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  }

}

TEST(RecoveryRoutes, UserNegativeYContinuesPastBlockedRearT) {
  auto recovery=make_recovery();
  const Recovery::Pose start{{1.239,-1.5},pi};
  // 2026-10-15 17:04 GUI 实际双障碍：宽20cm、深10cm。
  const std::vector<navigation_safety::Polygon> obstacles{
    navigation_safety::rectangle_polygon({1.10,-1.025,1.30,-.925}),
    navigation_safety::rectangle_polygon({2.10,-1.035,2.30,-.935})};
  const std::vector<std::string> blocked{"ROAD_Y3_LEFT","ROAD_Y2_LEFT"};
  recovery.set_dynamic_obstacles(obstacles);
  const auto candidates=recovery.candidates(start,obstacles.front());
  const auto far=std::find_if(candidates.begin(),candidates.end(),[](const Candidate &c) {
    return c.resume_node=="x0_y1" && c.resume_lane=="ROAD_Y1_LEFT";
  });
  ASSERT_NE(far,candidates.end());
  EXPECT_NE(std::find(far->borrowed_lane_ids.begin(),far->borrowed_lane_ids.end(),
    "ROAD_X0_Y1"),far->borrowed_lane_ids.end());
  for (const auto &lane:far->borrowed_lane_ids)
    EXPECT_EQ(std::find(blocked.begin(),blocked.end(),lane),blocked.end());
  ASSERT_TRUE(recovery.plan(start,*far,blocked));
  ASSERT_FALSE(recovery.steps().empty());
  const auto &retreat=recovery.steps().front();
  EXPECT_EQ(retreat.phase,"RETREAT_TO_TURN_POINT");
  EXPECT_LT(retreat.linear,0);
  EXPECT_NEAR(retreat.target.position.x,3.27,1e-9);
  EXPECT_NEAR(retreat.target.position.y,-1.5,1e-9);
  EXPECT_GT(retreat.end.position.x-retreat.start.position.x,2.0);
  // 真实封路快照下的独立车辆积分，连续越过 x=2.2 的不可用 T。
  ASSERT_TRUE(recovery.plan(start,*far,blocked));
  RecoveryReplay replay(recovery,workspace());
  ASSERT_TRUE(replay.run(start,recovery.steps())) << replay.failure();
  EXPECT_GT(replay.reverse_distance(),2.0);
  EXPECT_TRUE(recovery.exit_matches(replay.actual(),*far));
  auto behind_blocked=blocked; behind_blocked.push_back("ROAD_X0_Y1");
  EXPECT_FALSE(recovery.plan(start,*far,behind_blocked));
  EXPECT_TRUE(recovery.steps().empty());
  const auto rear_obstacle=navigation_safety::rectangle_polygon({2.65,-1.60,2.75,-1.40});
  recovery.set_dynamic_obstacles({obstacles[0],obstacles[1],rear_obstacle});
  EXPECT_FALSE(recovery.plan(start,*far,blocked));
  EXPECT_TRUE(recovery.steps().empty());
}

TEST(RecoveryRoutes, ReturnedCrossCanContinueBehindItsSavedIncomingRoad) {
  auto recovery=make_recovery();
  const Recovery::Pose start{{2.193,-.569},1.568};
  recovery.set_incoming_lane("ROAD_Y2_LEFT",{2.2,-.5});
  const auto candidates=recovery.candidates(start,
    navigation_safety::rectangle_polygon({2.464,-.60,2.584,-.40}));
  // 返回十字入口后，允许沿保存的入路继续倒退至后方边界 T。
  // 原 7 cm 固定入口在此越界，所以检查枚举已到达，动作仍必须拒绝。
  Candidate rear;
  rear.resume_node="x0_y2"; rear.resume_lane="ROAD_X0_Y1";
  rear.resume_heading=0; rear.borrowed_lane_ids={"ROAD_X0_Y1","ROAD_Y2_LEFT"};
  EXPECT_FALSE(recovery.plan(start,rear,{}));
  EXPECT_TRUE(recovery.unsafe_entry_rejected());
  EXPECT_TRUE(recovery.steps().empty());
}
