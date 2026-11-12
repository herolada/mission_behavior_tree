#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Node-free decision logic of OperatorRequestAction: given the CommanderState
// stream of crl_commander, decide whether a request sent through
// commander_operator is still in progress, done, or has failed.
//
// crl_commander bumps CommanderState.goal_id whenever it takes up a new task
// (and never on a mode switch alone), so "state == idle with a goal_id newer
// than the one seen before sending" means our request has been completed. Times
// are plain seconds so this can be unit-tested without a ROS clock.
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdint>
#include <optional>
#include <string>

namespace mission_behavior_tree
{

enum class RequestOutcome
{
  RUNNING,       // not taken up yet (within accept_timeout), or executing
  SUCCEEDED,     // taken up and the commander is idle again
  NOT_TAKEN_UP,  // goal_id did not move within accept_timeout
  STUCK          // commander has been stuck for longer than stuck_timeout
};

std::string toString(RequestOutcome outcome);

class RequestMonitor
{
public:
  // accept_timeout_sec: from start() until the commander must have taken the
  // request up (goal_id bumped). stuck_timeout_sec: how long a continuous
  // "stuck" state is tolerated once it has.
  RequestMonitor(double accept_timeout_sec, double stuck_timeout_sec);

  // The request is about to be made (BT onStart).
  void start(double now_sec);
  // Whether accept_timeout has run out since start().
  bool acceptTimedOut(double now_sec) const;
  // The request was sent while the commander reported goal_id_before.
  void requestSent(uint64_t goal_id_before);

  // Evaluates the latest CommanderState (state string, goal_id). Only
  // meaningful after requestSent().
  RequestOutcome update(const std::string & state, uint64_t goal_id, double now_sec);

  bool takenUp() const {return taken_up_;}

private:
  double accept_timeout_sec_;
  double stuck_timeout_sec_;

  double start_sec_{0.0};
  uint64_t goal_id_before_{0};
  bool taken_up_{false};
  std::optional<double> stuck_since_sec_;
};

}  // namespace mission_behavior_tree
