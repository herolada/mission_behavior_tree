#include "mission_behavior_tree/operator_request_action.hpp"

#include <chrono>

namespace mission_behavior_tree
{

OperatorRequestAction::OperatorRequestAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  const std::string service = getInput<std::string>("service").value();
  const std::string state_topic = getInput<std::string>("state_topic").value();

  client_ = node_->create_client<SendOperatorRequest>(service);
  state_sub_ = node_->create_subscription<CommanderState>(
    state_topic, rclcpp::QoS(10),
    [this](CommanderState::ConstSharedPtr msg) {latest_state_ = *msg;});
}

double OperatorRequestAction::nowSec() const
{
  return node_->now().seconds();
}

BT::NodeStatus OperatorRequestAction::fail(const std::string & reason)
{
  RCLCPP_ERROR(node_->get_logger(), "[%s] %s failed: %s",
    name().c_str(), description_.c_str(), reason.c_str());
  forgetPendingRequest();
  return BT::NodeStatus::FAILURE;
}

void OperatorRequestAction::forgetPendingRequest()
{
  if (pending_response_) {
    client_->remove_pending_request(pending_response_->request_id);
    pending_response_.reset();
  }
}

BT::NodeStatus OperatorRequestAction::onStart()
{
  auto goal = getInput<std::string>("goal");
  if (!goal || goal->empty()) {
    description_ = "request";
    return fail("no 'goal' given");
  }
  request_.goal  = *goal;
  request_.route = getInput<bool>("route").value();
  request_.wgs   = getInput<bool>("wgs").value();
  request_.route_goal_point = getInput<std::string>("route_goal_point").value();
  request_.loop  = getInput<bool>("loop").value();
  description_ = std::string(request_.route ? "route" : "waypoint") +
    (request_.wgs ? " (wgs)" : "") + " '" + request_.goal + "'" +
    (request_.route_goal_point.empty() ? "" : " to '" + request_.route_goal_point + "'") +
    (request_.loop ? " (loop)" : "");

  monitor_ = RequestMonitor(
    getInput<double>("accept_timeout").value(), getInput<double>("stuck_timeout").value());
  monitor_.start(nowSec());
  sent_ = false;
  forgetPendingRequest();

  // Sending happens in onRunning(), so a commander / commander_operator that is
  // not up yet is simply waited for (up to accept_timeout).
  return onRunning();
}

BT::NodeStatus OperatorRequestAction::onRunning()
{
  const double now = nowSec();

  if (!sent_) {
    if (!latest_state_ || !client_->service_is_ready()) {
      if (monitor_.acceptTimedOut(now)) {
        return fail(!latest_state_ ?
          "no CommanderState received on '" + std::string(state_sub_->get_topic_name()) + "'" :
          "service '" + std::string(client_->get_service_name()) + "' not available");
      }
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] Waiting for crl_commander state and commander_operator service…", name().c_str());
      return BT::NodeStatus::RUNNING;
    }

    auto req = std::make_shared<SendOperatorRequest::Request>();
    req->request = request_;
    monitor_.requestSent(latest_state_->goal_id);
    pending_response_.emplace(client_->async_send_request(req));
    sent_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] Sent %s (commander goal_id %lu).",
      name().c_str(), description_.c_str(), static_cast<unsigned long>(latest_state_->goal_id));
    return BT::NodeStatus::RUNNING;
  }

  // commander_operator answers once crl_commander has accepted (or rejected)
  // the mode switch; only then is the goal_id meaningful.
  if (pending_response_) {
    if (pending_response_->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return monitor_.acceptTimedOut(now) ?
             fail("no response from commander_operator") : BT::NodeStatus::RUNNING;
    }
    const bool success = pending_response_->future.get()->success;
    pending_response_.reset();
    if (!success) {
      return fail("rejected by commander_operator (see its log)");
    }
  }

  const bool was_taken_up = monitor_.takenUp();
  const auto outcome = monitor_.update(latest_state_->state, latest_state_->goal_id, now);
  if (!was_taken_up && monitor_.takenUp()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] %s taken up by the commander (goal_id %lu).",
      name().c_str(), description_.c_str(), static_cast<unsigned long>(latest_state_->goal_id));
  }

  switch (outcome) {
    case RequestOutcome::RUNNING:
      return BT::NodeStatus::RUNNING;
    case RequestOutcome::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "[%s] %s completed.", name().c_str(), description_.c_str());
      return BT::NodeStatus::SUCCESS;
    case RequestOutcome::NOT_TAKEN_UP:
      return fail("not taken up by the commander in time");
    case RequestOutcome::STUCK:
      return fail("commander stuck");
  }
  return BT::NodeStatus::FAILURE;
}

void OperatorRequestAction::onHalted()
{
  if (sent_) {
    RCLCPP_INFO(node_->get_logger(), "[%s] %s halted.", name().c_str(), description_.c_str());
  }
  forgetPendingRequest();
  sent_ = false;
}

}  // namespace mission_behavior_tree
