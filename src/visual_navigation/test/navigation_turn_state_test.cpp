#include "gtest/gtest.h"
#include "visual_navigation/navigation_turn_state.hpp"

TEST(NavigationTurnState, RotationOwnsAnchorAndRecoveryFlags)
{
  visual_navigation::NavigationTurnState state;
  state.begin_rotation(1.0, 2.0);
  EXPECT_TRUE(state.rotating());
  EXPECT_TRUE(state.anchor_valid());
  EXPECT_DOUBLE_EQ(state.anchor_x(), 1.0);
  state.begin_waypoint_recovery();
  EXPECT_TRUE(state.waypoint_recovery());
  state.reset_control();
  EXPECT_FALSE(state.rotating());
  EXPECT_FALSE(state.waypoint_recovery());
  EXPECT_FALSE(state.anchor_valid());
}

TEST(NavigationTurnState, SegmentResetClearsPathAlignment)
{
  visual_navigation::NavigationTurnState state;
  state.mark_path_aligned();
  EXPECT_TRUE(state.path_alignment_completed());
  state.reset_segment();
  EXPECT_FALSE(state.path_alignment_completed());
}
