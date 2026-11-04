#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace mission_behavior_tree
{

// Subscribes to `topic` on entry and stays RUNNING until a message arrives
// there (SUCCESS), or until `timeout` passes (FAILURE, 0 waits forever). Works
// for any message type, given by name. Only messages published after entry
// count; the subscription is dropped again on exit.
//
// Typical use: hold off a tree whose conditions read a topic until that topic
// is alive, e.g. nav2's IsBatteryLow reports "not low" before its first message.
// Match the QoS of the node that reads the topic (`best_effort`), otherwise this
// may hear a publisher the other one does not.
//
// Expects the rclcpp::Node::SharedPtr in the blackboard entry "node", spun by a
// single-threaded executor between ticks.
class WaitForMessageAction : public BT::StatefulActionNode
{
public:
  WaitForMessageAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "Topic to wait on"),
      BT::InputPort<std::string>("type", "Message type, e.g. sensor_msgs/msg/BatteryState"),
      BT::InputPort<bool>("best_effort", false, "Subscribe best-effort instead of reliable"),
      BT::InputPort<double>("timeout", 0.0, "[s] until FAILURE, 0 waits forever"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus finish(BT::NodeStatus status);

  rclcpp::Node::SharedPtr node_;
  rclcpp::GenericSubscription::SharedPtr sub_;
  std::string topic_;
  double timeout_{0.0};
  rclcpp::Time started_;
  bool received_{false};
};

}  // namespace mission_behavior_tree
