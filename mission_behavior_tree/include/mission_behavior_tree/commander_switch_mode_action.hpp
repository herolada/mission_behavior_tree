#pragma once

#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <crl_commander_interfaces/srv/switch_mode.hpp>

namespace mission_behavior_tree
{

// Calls crl_commander's ~/switch_mode service once per entry and stays RUNNING
// until it answers:
//   - SUCCESS if the commander accepted the mode. It applies the change on its
//     next main loop, this node does not wait for that.
//   - FAILURE if the commander rejected the mode, or if the service is not
//     available or does not answer within `timeout`.
//
// Expects the rclcpp::Node::SharedPtr in the blackboard entry "node", spun by a
// single-threaded executor between ticks.
class CommanderSwitchModeAction : public BT::StatefulActionNode
{
public:
  using SwitchMode = crl_commander_interfaces::srv::SwitchMode;

  CommanderSwitchModeAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "Mode to switch to, e.g. stop"),
      BT::InputPort<std::string>(
        "service", std::string("/crl_commander/switch_mode"), "crl_commander's SwitchMode service"),
      BT::InputPort<double>(
        "timeout", 5.0, "[s] from entry until the commander must have answered"),
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

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SwitchMode>::SharedPtr client_;

  // Per entry (read in onStart)
  std::string mode_;
  double timeout_{0.0};
  double started_{0.0};
  std::optional<rclcpp::Client<SwitchMode>::FutureAndRequestId> pending_response_;
};

}  // namespace mission_behavior_tree
