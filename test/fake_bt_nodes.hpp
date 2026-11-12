#pragma once

// Fake IsBatteryLow / OperatorRequest / WaitForMessage / CommanderSwitchMode
// nodes with the ports of the real ones, for running the mission trees without
// the rest of the system. They record what the tree asked for in the globals
// below; reset them with fake::reset() before each test.

#include <map>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <commander_operator_interfaces/msg/operator_request.hpp>

#include "mission_behavior_tree/commander_switch_mode_action.hpp"
#include "mission_behavior_tree/operator_request_action.hpp"
#include "mission_behavior_tree/wait_for_message_action.hpp"

namespace fake
{

inline double battery = 100.0;
inline std::vector<std::string> sent;    // goal of every request sent, in order
inline std::vector<std::string> halted;  // goal of every request halted, in order
inline std::map<std::string, BT::NodeStatus> result;  // onRunning() result per goal
// Latest request per goal, as read from the ports.
inline std::map<std::string, commander_operator_interfaces::msg::OperatorRequest> request;
inline std::vector<std::string> switched;  // mode of every CommanderSwitchMode call

inline void reset()
{
  battery = 100.0;
  sent.clear();
  halted.clear();
  result.clear();
  request.clear();
  switched.clear();
}

class BatteryLow : public BT::ConditionNode
{
public:
  BatteryLow(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery"),
      BT::InputPort<std::string>("battery_topic"),
      BT::InputPort<bool>("is_voltage"),
    };
  }

  BT::NodeStatus tick() override
  {
    return battery <= getInput<double>("min_battery").value() ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class OperatorRequest : public BT::StatefulActionNode
{
public:
  OperatorRequest(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return mission_behavior_tree::OperatorRequestAction::providedPorts();
  }

  BT::NodeStatus onStart() override
  {
    goal_ = getInput<std::string>("goal").value();
    sent.push_back(goal_);
    auto & r = request[goal_];
    r.goal = goal_;
    r.route = getInput<bool>("route").value();
    r.wgs = getInput<bool>("wgs").value();
    r.route_goal_point = getInput<std::string>("route_goal_point").value();
    r.loop = getInput<bool>("loop").value();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto it = result.find(goal_);
    return it == result.end() ? BT::NodeStatus::RUNNING : it->second;
  }

  void onHalted() override {halted.push_back(goal_);}

private:
  std::string goal_;
};

// The node types the mission trees use, with their real names. The battery
// message is taken to have arrived already, the commander accepts every mode.
inline void registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<BatteryLow>("IsBatteryLow");
  factory.registerNodeType<OperatorRequest>("OperatorRequest");
  factory.registerSimpleAction("WaitForMessage",
    [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
    mission_behavior_tree::WaitForMessageAction::providedPorts());
  factory.registerSimpleAction("CommanderSwitchMode",
    [](BT::TreeNode & node) {
      switched.push_back(node.getInput<std::string>("mode").value());
      return BT::NodeStatus::SUCCESS;
    },
    mission_behavior_tree::CommanderSwitchModeAction::providedPorts());
}

}  // namespace fake
