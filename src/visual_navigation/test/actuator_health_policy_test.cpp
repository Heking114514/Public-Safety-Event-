#include <gtest/gtest.h>

#include "visual_navigation/actuator_health_policy.hpp"

TEST(ActuatorHealthPolicy, DisabledCheckAlwaysAllowsNavigation)
{
  EXPECT_TRUE(visual_navigation::ActuatorHealthIsValid(false, false, false, false));
}

TEST(ActuatorHealthPolicy, RequiredCheckNeedsFreshConnectedStatus)
{
  EXPECT_TRUE(visual_navigation::ActuatorHealthIsValid(true, true, true, true));
  EXPECT_FALSE(visual_navigation::ActuatorHealthIsValid(true, false, true, true));
  EXPECT_FALSE(visual_navigation::ActuatorHealthIsValid(true, true, false, true));
  EXPECT_FALSE(visual_navigation::ActuatorHealthIsValid(true, true, true, false));
}

TEST(ActuatorHealthPolicy, RuntimeLossHoldsOnlyAnActiveTask)
{
  EXPECT_TRUE(visual_navigation::ShouldHoldForActuatorRecovery(true, false));
  EXPECT_FALSE(visual_navigation::ShouldHoldForActuatorRecovery(true, true));
  EXPECT_FALSE(visual_navigation::ShouldHoldForActuatorRecovery(false, false));
}
