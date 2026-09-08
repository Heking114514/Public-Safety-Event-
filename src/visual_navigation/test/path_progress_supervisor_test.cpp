#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "visual_navigation/path_progress_supervisor.hpp"

namespace
{

using visual_navigation::PathProgressAction;
using visual_navigation::PathProgressSupervisor;

TEST(PathProgressSupervisor, ShortCorrectionDoesNotTriggerBeforeTimeout)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  EXPECT_EQ(
    supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start).action,
    PathProgressAction::MONITORING);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 0.98, 0.24, 0.0, true,
      start + std::chrono::milliseconds(9900)).action,
    PathProgressAction::MONITORING);
}

TEST(PathProgressSupervisor, MeaningfulMonotonicProgressRefreshesWindow)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.06, 0.20, 0.0, true,
      start + std::chrono::seconds(9)).action,
    PathProgressAction::MONITORING);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.02, 0.40, 0.0, true,
      start + std::chrono::seconds(18)).action,
    PathProgressAction::MONITORING);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.02, 0.46, 0.0, true,
      start + std::chrono::seconds(19)).action,
    PathProgressAction::RECOVER);
}

TEST(PathProgressSupervisor, RecoversFiniteTimesThenLatchesFault)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start);
  auto decision = supervisor.Evaluate(
    3, 1.0, 0.25, 0.0, true, start + std::chrono::seconds(10));
  EXPECT_EQ(decision.action, PathProgressAction::RECOVER);
  EXPECT_EQ(decision.recovery_attempt, 1);

  supervisor.Evaluate(3, 1.0, 0.25, 0.0, true, start + std::chrono::seconds(11));
  decision = supervisor.Evaluate(
    3, 1.0, 0.50, 0.0, true, start + std::chrono::seconds(21));
  EXPECT_EQ(decision.action, PathProgressAction::RECOVER);
  EXPECT_EQ(decision.recovery_attempt, 2);

  supervisor.Evaluate(3, 1.0, 0.50, 0.0, true, start + std::chrono::seconds(22));
  decision = supervisor.Evaluate(
    3, 1.0, 0.75, 0.0, true, start + std::chrono::seconds(32));
  EXPECT_EQ(decision.action, PathProgressAction::FAULT);
  EXPECT_EQ(decision.recovery_attempt, 2);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 2.0, 1.0, 0.0, true,
      start + std::chrono::seconds(33)).action,
    PathProgressAction::FAULT);
}

TEST(PathProgressSupervisor, IntentionalInactiveStatesPauseTimeout)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.0, 0.20, 0.0, false,
      start + std::chrono::seconds(9)).action,
    PathProgressAction::IDLE);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.0, 0.20, 0.0, true,
      start + std::chrono::seconds(100)).action,
    PathProgressAction::MONITORING);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.0, 0.44, 0.0, true,
      start + std::chrono::seconds(109)).action,
    PathProgressAction::MONITORING);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.0, 0.45, 0.0, true,
      start + std::chrono::seconds(110)).action,
    PathProgressAction::RECOVER);
}

TEST(PathProgressSupervisor, NewWaypointClearsRecoveryHistory)
{
  PathProgressSupervisor supervisor({1.0, 0.05, 0.25, 1});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 0.0, 0.25, 0.0, true,
      start + std::chrono::seconds(1)).action,
    PathProgressAction::RECOVER);

  const auto decision = supervisor.Evaluate(
    4, 0.0, 0.25, 0.0, true, start + std::chrono::seconds(2));
  EXPECT_EQ(decision.action, PathProgressAction::MONITORING);
  EXPECT_EQ(decision.recovery_attempt, 0);
}

TEST(PathProgressSupervisor, InvalidProgressCannotCreateFalseFault)
{
  PathProgressSupervisor supervisor({1.0, 0.05, 0.25, 1});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, std::numeric_limits<double>::quiet_NaN(), 0.25, 0.0, true,
      start + std::chrono::seconds(20)).action,
    PathProgressAction::IDLE);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 0.0, 0.25, 0.0, true,
      start + std::chrono::seconds(21)).action,
    PathProgressAction::MONITORING);
}

TEST(PathProgressSupervisor, NoActualTranslationDoesNotOverlapStallDetection)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start);
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.0, 0.0, 0.0, true,
      start + std::chrono::seconds(60)).action,
    PathProgressAction::MONITORING);
}

TEST(PathProgressSupervisor, BoundedInPlaceJitterDoesNotAccumulateIntoTravel)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start);
  for (int sample = 1; sample <= 12; ++sample)
  {
    const double jitter = sample % 2 == 0 ? 0.04 : -0.04;
    EXPECT_EQ(
      supervisor.Evaluate(
        3, 1.0, jitter, -jitter, true,
        start + std::chrono::seconds(sample * 5)).action,
      PathProgressAction::MONITORING);
  }
}

TEST(PathProgressSupervisor, AlternatingTurnAndDriveStillTriggers)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2});
  const auto start = PathProgressSupervisor::TimePoint{};

  supervisor.Evaluate(3, 1.0, 0.0, 0.0, true, start);
  supervisor.Evaluate(3, 1.0, 0.02, 0.08, true, start + std::chrono::seconds(2));
  supervisor.Evaluate(3, 1.01, 0.10, 0.08, true, start + std::chrono::seconds(4));
  supervisor.Evaluate(3, 1.01, 0.12, 0.16, true, start + std::chrono::seconds(6));
  supervisor.Evaluate(3, 1.02, 0.20, 0.16, true, start + std::chrono::seconds(8));
  EXPECT_EQ(
    supervisor.Evaluate(
      3, 1.02, 0.22, 0.24, true,
      start + std::chrono::seconds(10)).action,
    PathProgressAction::RECOVER);
}

TEST(PathProgressSupervisor, CommandedMotionWithNoTranslationRecoversThenFaults)
{
  PathProgressSupervisor supervisor({10.0, 0.05, 0.25, 2, 0.08, 0.02, 3.0, 0.05});
  const auto start = PathProgressSupervisor::TimePoint{};

  // A stuck chassis never reaches the ordinary 0.25 m displacement gate, but
  // the independent command/no-motion dwell still gets two recovery chances.
  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true, start, 0.10, 0.0, true);
  EXPECT_EQ(
      supervisor.Evaluate(3, 0.0, 0.0, 0.0, true,
                          start + std::chrono::milliseconds(2990),
                          0.10, 0.0, true)
          .action,
      PathProgressAction::MONITORING);
  auto decision = supervisor.Evaluate(
      3, 0.0, 0.0, 0.0, true, start + std::chrono::seconds(3), 0.10, 0.0,
      true);
  EXPECT_EQ(decision.action, PathProgressAction::RECOVER);
  EXPECT_EQ(decision.recovery_attempt, 1);

  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true,
                      start + std::chrono::seconds(4), 0.10, 0.0, true);
  decision = supervisor.Evaluate(3, 0.0, 0.0, 0.0, true,
                                 start + std::chrono::seconds(7), 0.10, 0.0,
                                 true);
  EXPECT_EQ(decision.action, PathProgressAction::RECOVER);
  EXPECT_EQ(decision.recovery_attempt, 2);

  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true,
                      start + std::chrono::seconds(8), 0.10, 0.0, true);
  decision = supervisor.Evaluate(3, 0.0, 0.0, 0.0, true,
                                 start + std::chrono::seconds(11), 0.10, 0.0,
                                 true);
  EXPECT_EQ(decision.action, PathProgressAction::FAULT);
}

TEST(PathProgressSupervisor, InPlaceTurnDoesNotUseLinearNoMotionRule)
{
  PathProgressSupervisor supervisor({1.0, 0.05, 0.25, 2, 0.08, 0.02, 3.0, 0.05});
  const auto start = PathProgressSupervisor::TimePoint{};
  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true, start, 0.08, 0.0, true,
                      false);
  EXPECT_EQ(
      supervisor.Evaluate(3, 0.0, 0.0, 0.0, true,
                          start + std::chrono::seconds(30), 0.08, 0.0, true,
                          false)
          .action,
      PathProgressAction::MONITORING);
}

TEST(PathProgressSupervisor, SmallMeasuredMotionKeepsNoMotionTimerFromFiring)
{
  PathProgressSupervisor supervisor({1.0, 0.05, 0.25, 2, 0.08, 0.02, 3.0, 0.05});
  const auto start = PathProgressSupervisor::TimePoint{};
  supervisor.Evaluate(3, 0.0, 0.0, 0.0, true, start, 0.10, 0.03, true);
  EXPECT_EQ(
      supervisor.Evaluate(3, 0.0, 0.06, 0.0, true,
                          start + std::chrono::seconds(4), 0.10, 0.03, true)
          .action,
      PathProgressAction::MONITORING);
}

}  // namespace
