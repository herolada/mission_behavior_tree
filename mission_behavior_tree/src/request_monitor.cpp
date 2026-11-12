#include "mission_behavior_tree/request_monitor.hpp"

#include <crl_commander_interfaces/msg/commander_state.hpp>

namespace mission_behavior_tree
{

std::string toString(RequestOutcome outcome)
{
  switch (outcome) {
    case RequestOutcome::RUNNING:      return "RUNNING";
    case RequestOutcome::SUCCEEDED:    return "SUCCEEDED";
    case RequestOutcome::NOT_TAKEN_UP: return "NOT_TAKEN_UP";
    case RequestOutcome::STUCK:        return "STUCK";
  }
  return "UNKNOWN";
}

RequestMonitor::RequestMonitor(double accept_timeout_sec, double stuck_timeout_sec)
: accept_timeout_sec_(accept_timeout_sec), stuck_timeout_sec_(stuck_timeout_sec)
{
}

void RequestMonitor::start(double now_sec)
{
  start_sec_ = now_sec;
  goal_id_before_ = 0;
  taken_up_ = false;
  stuck_since_sec_.reset();
}

bool RequestMonitor::acceptTimedOut(double now_sec) const
{
  return now_sec - start_sec_ > accept_timeout_sec_;
}

void RequestMonitor::requestSent(uint64_t goal_id_before)
{
  goal_id_before_ = goal_id_before;
}

RequestOutcome RequestMonitor::update(
  const std::string & state, uint64_t goal_id, double now_sec)
{
  using CommanderState = crl_commander_interfaces::msg::CommanderState;

  // Strictly greater: a restarted commander starts counting from 0 again, which
  // must not be mistaken for our request having been taken up.
  if (!taken_up_ && goal_id > goal_id_before_) {
    taken_up_ = true;
  }
  if (!taken_up_) {
    return acceptTimedOut(now_sec) ? RequestOutcome::NOT_TAKEN_UP : RequestOutcome::RUNNING;
  }

  if (state == CommanderState::STATE_IDLE) {
    return RequestOutcome::SUCCEEDED;
  }
  if (state == CommanderState::STATE_STUCK) {
    if (!stuck_since_sec_) stuck_since_sec_ = now_sec;
    return (now_sec - *stuck_since_sec_ > stuck_timeout_sec_) ?
           RequestOutcome::STUCK : RequestOutcome::RUNNING;
  }
  stuck_since_sec_.reset();
  return RequestOutcome::RUNNING;
}

}  // namespace mission_behavior_tree
