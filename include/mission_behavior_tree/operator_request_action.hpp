#pragma once

#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <commander_operator_interfaces/srv/send_operator_request.hpp>
#include <crl_commander_interfaces/msg/commander_state.hpp>

#include "mission_behavior_tree/request_monitor.hpp"

namespace mission_behavior_tree
{

// Sends one named waypoint / route request through commander_operator's
// ~/operator_request service and stays RUNNING until crl_commander has
// completed it:
//   - SUCCESS once the commander took the request up (CommanderState.goal_id
//     moved past the value seen before sending) and is idle again. A looping
//     route is never idle, so it stays RUNNING until halted.
//   - FAILURE if commander_operator rejects the request (unknown name, the
//     commander is down or refused the mode), if the request is not taken up
//     within accept_timeout, or if the commander stays stuck for stuck_timeout.
//
// The request is sent exactly once per entry (onStart → first onRunning) – every
// request makes crl_commander cancel and replan, so it must never be re-sent
// while the node is merely being re-ticked. Halting sends nothing; whichever
// branch takes over sends its own request.
//
// Expects the rclcpp::Node::SharedPtr in the blackboard entry "node", which must
// be spun by a single-threaded executor between ticks (the subscription and
// service client live in the node's default callback group).
class OperatorRequestAction : public BT::StatefulActionNode
{
public:
  using SendOperatorRequest = commander_operator_interfaces::srv::SendOperatorRequest;
  using CommanderState = crl_commander_interfaces::msg::CommanderState;

  OperatorRequestAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal", "Name of the commander_operator waypoint / route"),
      BT::InputPort<bool>("route", false, "true: 'goal' names a route, false: a waypoint"),
      BT::InputPort<bool>("wgs", false, "true: look 'goal' up among the WGS waypoints / routes"),
      BT::InputPort<std::string>(
        "route_goal_point", std::string(""),
        "Only for routes: waypoint of the route to drive to, empty = the whole route"),
      BT::InputPort<bool>(
        "loop", false,
        "Only for whole routes: continue from the first waypoint after the last one"),
      BT::InputPort<std::string>(
        "service", std::string("/commander_operator/operator_request"),
        "commander_operator's SendOperatorRequest service"),
      BT::InputPort<std::string>(
        "state_topic", std::string("/crl_commander/state"),
        "crl_commander's CommanderState topic"),
      BT::InputPort<double>(
        "accept_timeout", 10.0,
        "[s] from entry until the commander must have taken the request up"),
      BT::InputPort<double>(
        "stuck_timeout", 30.0,
        "[s] of continuous 'stuck' state after which the request fails"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Drops a response still in flight, so it can never be mistaken for the one
  // of a later request.
  void forgetPendingRequest();
  BT::NodeStatus fail(const std::string & reason);
  double nowSec() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SendOperatorRequest>::SharedPtr client_;
  rclcpp::Subscription<CommanderState>::SharedPtr state_sub_;
  std::optional<CommanderState> latest_state_;

  // Per entry (read in onStart)
  commander_operator_interfaces::msg::OperatorRequest request_;
  std::string description_;  // e.g. "waypoint 'home'", for logging
  RequestMonitor monitor_{0.0, 0.0};
  bool sent_{false};
  std::optional<rclcpp::Client<SendOperatorRequest>::FutureAndRequestId> pending_response_;
};

}  // namespace mission_behavior_tree
