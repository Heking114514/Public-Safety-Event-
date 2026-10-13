#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "visual_navigation/pose_recovery.hpp"
#include "visual_navigation/rate_limiter.hpp"
#include "vehicle_loopback_sim/motion.hpp"

namespace {
using Recovery=visual_navigation::PoseRecovery;
constexpr double pi=3.14159265358979323846;
std::string temporary_map() {
  const std::string path="/tmp/pose_recovery_test_"+std::to_string(getpid())+".yaml";
  std::ofstream out(path);
  out << "frame:\n  map_origin_m: [0, 0]\n  map_x_heading_deg: 0\n"
      << "arena:\n  free_regions: [[-2, -2, 2, 2]]\n  obstacles: []\n"
      << "nodes:\n  a: [0, 0]\n  b: [0, -1]\n  c: [1, 0]\n"
      << "lanes:\n  - {id: ab, from: a, to: b, width_m: 0.2}\n"
      << "  - {id: ac, from: a, to: c, width_m: 0.2}\n";
  return path;
}
Recovery make_recovery(double body=0.125) {
  const auto path=temporary_map();Recovery r;
  r.load(path,{body,body,0.,0.,0.},Recovery::Config{});
  std::remove(path.c_str());return r;
}
}

TEST(PoseRecovery, BaseLinkIntegrationMatchesSimulatorForwardAndReverse) {
  const auto r=make_recovery();
  for(double yaw:{-pi, -0.7, 0.0, 1.8}) for(double v:{-0.035,0.035}) for(double w:{-0.65,0.0,0.65}) {
    const Recovery::Pose p{{1.2,-0.65},yaw};
    const auto actual=r.integrate(p,v,w,0.45);
    const auto expected=vehicle_loopback_sim::integrate_base_link(
      {p.position.x,p.position.y,p.yaw},{v,w},0.45,0.030,0.0);
    EXPECT_NEAR(actual.position.x,expected.x,1e-12);
    EXPECT_NEAR(actual.position.y,expected.y,1e-12);
    EXPECT_NEAR(std::atan2(std::sin(actual.yaw-expected.yaw),std::cos(actual.yaw-expected.yaw)),0.,1e-12);
  }
}

TEST(PoseRecovery, SamplesIncludeAngularCommandRateLimit) {
  const auto r=make_recovery();double end_w=0.;
  const Recovery::Pose start{{0.,0.},0.};
  const auto poses=r.motion_samples(start,0.035,0.65,-0.65,0.45,&end_w);
  EXPECT_NEAR(end_w,0.16,1e-10); // rising limit 1.8 * .45, never instantly +.65
  vehicle_loopback_sim::Pose2d expected{0.,0.,0.};double w=-0.65;
  for(double t=0.;t<0.45-1e-10;) {
    const double dt=std::min(1./30.,0.45-t);
    w=visual_navigation::LimitRate(0.65,w,1.8,3.0,dt);
    expected=vehicle_loopback_sim::integrate_base_link(expected,{0.035,w},dt,0.030,0.);
    t+=dt;
  }
  EXPECT_NEAR(poses.back().position.x,expected.x,1e-12);
  EXPECT_NEAR(poses.back().position.y,expected.y,1e-12);
  EXPECT_NEAR(poses.back().yaw,expected.yaw,1e-12);
}

TEST(PoseRecovery, HoldDoesNotConsumeExecutionClock) {
  Recovery r;r.reset_clock(100.);
  EXPECT_DOUBLE_EQ(r.active_seconds(100.5),0.5);
  r.pause_clock();r.pause_clock();
  EXPECT_DOUBLE_EQ(r.active_seconds(170.5),0.5);
  EXPECT_DOUBLE_EQ(r.active_seconds(170.75),0.75);
  r.reset_clock(200.);EXPECT_DOUBLE_EQ(r.active_seconds(200.25),0.25);
}

TEST(PoseRecovery, SweepChecksArcInteriorAndBodyFrontAtBaseLinkCenter) {
  auto r=make_recovery(0.010);
  Recovery::Pose start{{0.,-0.2},0.};
  const auto middle=r.integrate(start,0.035,0.65,0.8);
  const auto obstacle=navigation_safety::rectangle_polygon({middle.position.x-.004,middle.position.y-.004,
    middle.position.x+.004,middle.position.y+.004});
  r.candidates(start,obstacle);
  const auto end=r.integrate(start,0.035,0.65,1.6);
  EXPECT_TRUE(r.safe({start}));EXPECT_TRUE(r.safe({end}));
  EXPECT_FALSE(r.safe_motion(start,0.035,0.65,1.6));
  auto full=make_recovery();
  // A collision in the foremost 30mm was missed by the old -base_offset footprint.
  full.candidates({{0.,-0.2},0.},navigation_safety::rectangle_polygon({0.05,-0.205,0.07,-0.195}));
  EXPECT_FALSE(full.safe({{{0.,-0.2},0.}}));
}

TEST(PoseRecovery, RejectsBlockedExitBeforeSearch) {
  auto r=make_recovery();
  const auto candidates=r.candidates({{0.,-0.2},-pi/2},
    navigation_safety::rectangle_polygon({-.09,-.65,.09,-.55}));
  ASSERT_FALSE(candidates.empty());
  EXPECT_FALSE(r.plan({{0.,-0.2},-pi/2},candidates.front(),{candidates.front().resume_lane}));
  EXPECT_EQ(r.last_expansions(),0);
}

TEST(PoseRecovery, ActualArenaObstacleHasExecutableExit) {
  const char *path=std::getenv("POSE_RECOVERY_ARENA");
  if(!path) GTEST_SKIP() << "Set POSE_RECOVERY_ARENA to the actual arena YAML";
  Recovery r;Recovery::Config config;
  r.load(path,{0.125,0.125,0.,0.,0.},config);
  const Recovery::Pose start{{1.2,-0.65},-pi/2};
  const auto obstacle=navigation_safety::rectangle_polygon({1.11,-0.99,1.29,-0.91});
  const auto candidates=r.candidates(start,obstacle);
  ASSERT_FALSE(candidates.empty());bool found=false;
  for(const auto &candidate:candidates) {
    const auto began=std::chrono::steady_clock::now();
    const bool ok=r.plan(start,candidate,{r.blocked_lane()});
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    std::cout << "arena exit=" << candidate.resume_lane << " seconds=" << seconds
      << " expansions=" << r.last_expansions() << " steps=" << r.steps().size() << " ok=" << ok << '\n';
    if(!ok) continue;
    found=true;
    for(const auto &step:r.steps()) {
      EXPECT_TRUE(r.safe(r.motion_samples(step.start,step.linear,step.angular,step.initial_angular,step.duration)));
      const auto end=r.reference(step,step.duration);
      EXPECT_NEAR(end.position.x,step.end.position.x,1e-12);
      EXPECT_NEAR(end.position.y,step.end.position.y,1e-12);
    }
    break;
  }
  EXPECT_TRUE(found);
}
