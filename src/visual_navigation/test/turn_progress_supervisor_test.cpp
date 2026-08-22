#include <gtest/gtest.h>

#include <chrono>

#include "visual_navigation/turn_progress_supervisor.hpp"

namespace
{

using visual_navigation::TurnProgressAction;
using visual_navigation::TurnProgressSupervisor;

TEST(TurnProgressSupervisor, WideTurnProgressDoesNotUseShortFixedTimeout)
{
  TurnProgressSupervisor supervisor({6.0, 0.08, 12.0, 0.12, 40.0, 2});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 3.12, 0.30, true, start);
  for (int second = 1; second <= 10; ++second)
  {
    EXPECT_EQ(
      supervisor.Evaluate(
        7, 3.12 - 0.30 * second, 0.30, true,
        start + std::chrono::seconds(second)).action,
      TurnProgressAction::MONITORING);
  }
}

TEST(TurnProgressSupervisor, DegradedHalfTurnRemainsAllowedWhileImproving)
{
  TurnProgressSupervisor supervisor({6.0, 0.08, 12.0, 0.12, 40.0, 2});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 3.12, 0.225, true, start);
  for (int second = 1; second <= 15; ++second)
  {
    EXPECT_EQ(
      supervisor.Evaluate(
        7, 3.12 - 0.18 * second, 0.225, true,
        start + std::chrono::seconds(second)).action,
      TurnProgressAction::MONITORING);
  }
}

TEST(TurnProgressSupervisor, StalledYawRequestsRecovery)
{
  TurnProgressSupervisor supervisor({6.0, 0.08, 12.0, 0.12, 40.0, 2});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 1.0, 0.9, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 0.98, 0.9, true,
      start + std::chrono::milliseconds(5900)).action,
    TurnProgressAction::MONITORING);
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 0.98, 0.9, true,
      start + std::chrono::seconds(6)).action,
    TurnProgressAction::RECOVER);
}

TEST(TurnProgressSupervisor, TotalLimitCatchesEndlessSmallOscillation)
{
  TurnProgressSupervisor supervisor({6.0, 0.08, 2.0, 0.12, 4.0, 1});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 0.24, 0.12, true, start);
  supervisor.Evaluate(7, 0.15, 0.12, true, start + std::chrono::seconds(1));
  supervisor.Evaluate(7, 0.06, 0.12, true, start + std::chrono::seconds(2));
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 0.14, 0.12, true,
      start + std::chrono::seconds(4)).action,
    TurnProgressAction::RECOVER);
}

TEST(TurnProgressSupervisor, FiniteRecoveriesEndInFault)
{
  TurnProgressSupervisor supervisor({1.0, 0.08, 2.0, 0.12, 4.0, 1});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 1.0, 0.9, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 1.0, 0.9, true,
      start + std::chrono::seconds(1)).action,
    TurnProgressAction::RECOVER);
  supervisor.Evaluate(7, 1.0, 0.9, true, start + std::chrono::seconds(2));
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 1.0, 0.9, true,
      start + std::chrono::seconds(3)).action,
    TurnProgressAction::FAULT);
}

TEST(TurnProgressSupervisor, IntentionalInactiveStateStartsFreshWindow)
{
  TurnProgressSupervisor supervisor({2.0, 0.08, 4.0, 0.12, 10.0, 1});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 1.0, 0.9, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 1.0, 0.9, false,
      start + std::chrono::seconds(2)).action,
    TurnProgressAction::IDLE);
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 1.0, 0.9, true,
      start + std::chrono::seconds(20)).action,
      TurnProgressAction::MONITORING);
}

TEST(TurnProgressSupervisor, PausePreservesFiniteRecoveryHistory)
{
  TurnProgressSupervisor supervisor({1.0, 0.08, 2.0, 0.12, 4.0, 1});
  const auto start = TurnProgressSupervisor::TimePoint{};

  supervisor.Evaluate(7, 1.0, 0.9, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 1.0, 0.9, true,
      start + std::chrono::seconds(1)).action,
    TurnProgressAction::RECOVER);

  supervisor.Pause();
  supervisor.Evaluate(7, 1.0, 0.9, true, start + std::chrono::seconds(20));
  EXPECT_EQ(
    supervisor.Evaluate(
      7, 1.0, 0.9, true,
      start + std::chrono::seconds(21)).action,
    TurnProgressAction::FAULT);
}

}  // namespace
