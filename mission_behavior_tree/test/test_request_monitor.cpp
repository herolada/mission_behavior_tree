#include <gtest/gtest.h>

#include "mission_behavior_tree/request_monitor.hpp"

using mission_behavior_tree::RequestMonitor;
using mission_behavior_tree::RequestOutcome;

namespace
{

constexpr double kAcceptTimeout = 10.0;
constexpr double kStuckTimeout = 30.0;

// Started at t = 0 and sent while the commander reported goal_id 5.
RequestMonitor sentMonitor()
{
  RequestMonitor m(kAcceptTimeout, kStuckTimeout);
  m.start(0.0);
  m.requestSent(5);
  return m;
}

}  // namespace

TEST(RequestMonitor, StaleIdleBeforeTakeUpIsNotSuccess)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("idle", 5, 1.0), RequestOutcome::RUNNING);
  EXPECT_FALSE(m.takenUp());
}

TEST(RequestMonitor, NotTakenUpWithinAcceptTimeoutFails)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("executing", 5, kAcceptTimeout - 0.1), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("executing", 5, kAcceptTimeout + 0.1), RequestOutcome::NOT_TAKEN_UP);
}

TEST(RequestMonitor, ExecutingThenIdleSucceeds)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("executing", 6, 1.0), RequestOutcome::RUNNING);
  EXPECT_TRUE(m.takenUp());
  // Once taken up, the accept timeout no longer applies.
  EXPECT_EQ(m.update("executing", 6, kAcceptTimeout + 100.0), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("idle", 6, kAcceptTimeout + 101.0), RequestOutcome::SUCCEEDED);
}

TEST(RequestMonitor, AlreadyAtGoalSucceedsWithoutExecuting)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("idle", 6, 0.5), RequestOutcome::SUCCEEDED);
}

TEST(RequestMonitor, SeveralBumpsStillCountAsTakenUp)
{
  // A stale route loaded on the mode switch, then the fresh one: two bumps.
  auto m = sentMonitor();
  EXPECT_EQ(m.update("executing", 6, 0.1), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("executing", 7, 0.2), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("idle", 7, 5.0), RequestOutcome::SUCCEEDED);
}

TEST(RequestMonitor, RestartedCommanderIsNotTakenUp)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("idle", 0, 1.0), RequestOutcome::RUNNING);
  EXPECT_FALSE(m.takenUp());
}

TEST(RequestMonitor, ShortStuckIsTolerated)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("stuck", 6, 1.0), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("stuck", 6, 1.0 + kStuckTimeout - 0.1), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("stuck", 6, 1.0 + kStuckTimeout + 0.1), RequestOutcome::STUCK);
}

TEST(RequestMonitor, ExecutingResetsStuckTimer)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("stuck", 6, 1.0), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("executing", 6, 20.0), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("stuck", 6, 25.0), RequestOutcome::RUNNING);
  EXPECT_EQ(m.update("stuck", 6, 25.0 + kStuckTimeout - 0.1), RequestOutcome::RUNNING);
}

TEST(RequestMonitor, StartResetsState)
{
  auto m = sentMonitor();
  EXPECT_EQ(m.update("idle", 6, 1.0), RequestOutcome::SUCCEEDED);
  m.start(100.0);
  m.requestSent(6);
  EXPECT_FALSE(m.takenUp());
  EXPECT_FALSE(m.acceptTimedOut(100.0 + kAcceptTimeout - 0.1));
  EXPECT_EQ(m.update("idle", 6, 101.0), RequestOutcome::RUNNING);
}
