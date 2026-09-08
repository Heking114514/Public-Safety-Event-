#include "visual_navigation/control_state.hpp"
#include "gtest/gtest.h"

namespace {

using visual_navigation::ControlPhase;
using visual_navigation::ControlState;
using visual_navigation::GoalStage;
using visual_navigation::WaitAction;

TEST(ControlState, StartsInRouteFollowing) {
  const ControlState state;
  EXPECT_EQ(state.phase(), ControlPhase::FOLLOW);
  EXPECT_EQ(state.goal_stage(), GoalStage::ROUTE);
  EXPECT_FALSE(state.waiting());
  EXPECT_FALSE(state.goal_captured());
}

TEST(ControlState, ManeuverPhasesAreExclusive) {
  ControlState state;
  state.begin_path_alignment(1.0, 2.0);
  EXPECT_TRUE(state.in(ControlPhase::ALIGN_PATH));
  EXPECT_TRUE(state.anchor_valid());

  state.begin_waypoint_recovery();
  EXPECT_TRUE(state.in(ControlPhase::RECOVER_WAYPOINT));
  EXPECT_FALSE(state.in(ControlPhase::ALIGN_PATH));
  EXPECT_FALSE(state.anchor_valid());

  state.begin_braking();
  EXPECT_TRUE(state.in(ControlPhase::BRAKE));
  EXPECT_FALSE(state.in(ControlPhase::RECOVER_WAYPOINT));
}

TEST(ControlState, WaitActionIsConsumedOnce) {
  ControlState state;
  state.begin_wait(WaitAction::COMPLETE);
  EXPECT_TRUE(state.waiting());
  EXPECT_EQ(state.wait_action(), WaitAction::COMPLETE);

  EXPECT_EQ(state.finish_wait(), WaitAction::COMPLETE);
  EXPECT_EQ(state.phase(), ControlPhase::FOLLOW);
  EXPECT_EQ(state.wait_action(), WaitAction::ADVANCE);
}

TEST(ControlState, GoalRecoveryReturnsThroughBrakeAndAlignment) {
  ControlState state;
  state.capture_goal();
  EXPECT_EQ(state.goal_stage(), GoalStage::PENDING_STOP);

  state.begin_braking();
  state.finish_braking(true);
  EXPECT_EQ(state.phase(), ControlPhase::ALIGN_GOAL);
  EXPECT_TRUE(state.goal_stopped());

  state.begin_goal_recovery();
  EXPECT_EQ(state.phase(), ControlPhase::RECOVER_GOAL);
  EXPECT_TRUE(state.recovering_goal());

  state.capture_goal();
  state.begin_braking();
  state.finish_braking(true);
  EXPECT_EQ(state.phase(), ControlPhase::ALIGN_GOAL);
  EXPECT_TRUE(state.goal_stopped());
}

TEST(ControlState, ManeuverInterruptPreservesGoalAndWait) {
  ControlState state;
  state.capture_goal();
  state.begin_goal_alignment();
  state.interrupt_maneuver();
  EXPECT_EQ(state.phase(), ControlPhase::FOLLOW);
  EXPECT_TRUE(state.goal_captured());

  state.begin_wait(WaitAction::COMPLETE);
  state.interrupt_maneuver();
  EXPECT_TRUE(state.waiting());
  EXPECT_EQ(state.wait_action(), WaitAction::COMPLETE);
}

TEST(ControlState, GoalRecoveryContextSurvivesManeuverInterrupt) {
  ControlState state;
  state.begin_goal_recovery();

  state.interrupt_maneuver();

  EXPECT_EQ(state.phase(), ControlPhase::FOLLOW);
  EXPECT_TRUE(state.recovering_goal());
  EXPECT_FALSE(state.anchor_valid());
}

TEST(ControlState, RunResetClearsPersistentControlContext) {
  ControlState state;
  state.begin_goal_recovery();
  state.begin_wait(WaitAction::COMPLETE);

  state.reset_run();

  EXPECT_EQ(state.phase(), ControlPhase::FOLLOW);
  EXPECT_EQ(state.goal_stage(), GoalStage::ROUTE);
  EXPECT_EQ(state.wait_action(), WaitAction::ADVANCE);
  EXPECT_FALSE(state.path_aligned());
  EXPECT_FALSE(state.anchor_valid());
}

TEST(ControlState, SegmentResetClearsGoalAndAlignmentHistory) {
  ControlState state;
  state.begin_path_alignment(1.0, 2.0);
  state.finish_path_alignment();
  state.capture_goal();
  state.reset_segment();

  EXPECT_EQ(state.phase(), ControlPhase::FOLLOW);
  EXPECT_EQ(state.goal_stage(), GoalStage::ROUTE);
  EXPECT_FALSE(state.path_aligned());
  EXPECT_FALSE(state.anchor_valid());
}

TEST(ControlState, PersistentPhasesKeepTheirPublicStatusNames) {
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::FOLLOW),
               "FOLLOWING");
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::ALIGN_PATH),
               "ROTATING_TO_PATH");
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::RECOVER_WAYPOINT),
               "RECOVERING_WAYPOINT");
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::RECOVER_GOAL),
               "RECOVERING_WAYPOINT");
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::BRAKE),
               "BRAKING_AT_WAYPOINT");
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::WAIT),
               "WAITING_AT_WAYPOINT");
  EXPECT_STREQ(visual_navigation::phase_status(ControlPhase::ALIGN_GOAL),
               "ALIGNING_FINAL_YAW");
}

} // namespace
