#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "mission_behavior_tree_interfaces/srv/load_mission.hpp"
#include "mission_behavior_tree_interfaces/srv/set_mode.hpp"

namespace mission_behavior_tree
{

// Generic runner for mission behavior trees. Everything mission specific comes
// from the parameters (see config/*.yaml):
//   tree_file      XML to run, relative to share/mission_behavior_tree/behavior_trees
//   bt_plugins     BT.CPP plugin libraries to load (e.g. nav2 condition nodes)
//   blackboard.*   copied into the blackboard, so the XML can refer to them as {name}
// and ticks the tree from a timer, restarting it whenever it finishes.
//
// The ~/load_mission service swaps in another mission at runtime.
//
// A mission may also declare operator-selectable modes:
//   modes          e.g. [PATROL, HOME, STOP]
//   initial_mode   one of them, defaults to the first
// The current mode is kept in the blackboard entry 'mode', changed with the
// ~/set_mode service and published (latched) on ~/mode. Loading a mission
// starts it in its initial_mode.
//
// Not an rclcpp::Node subclass on purpose: the blackboard holds the node, so the
// node must not own the tree as well. The node must be spun by a single-threaded
// executor, which keeps the service from running in the middle of a tick.
class MissionBehaviorTree
{
public:
  explicit MissionBehaviorTree(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp::Node::SharedPtr node() const {return node_;}

  void shutdown();

private:
  using LoadMission = mission_behavior_tree_interfaces::srv::LoadMission;
  using SetMode = mission_behavior_tree_interfaces::srv::SetMode;

  // Builds the mission and only then replaces the running one, so if this throws
  // while validating, the running mission is untouched.
  void loadMission(const std::vector<rclcpp::Parameter> & parameters);
  void onLoadMission(const LoadMission::Request::SharedPtr request,
    LoadMission::Response::SharedPtr response);
  void onSetMode(const SetMode::Request::SharedPtr request,
    SetMode::Response::SharedPtr response);
  void publishMode();
  void replaceNodeParameters(const std::vector<rclcpp::Parameter> & parameters);
  void warnAboutUnknownParameters(const std::vector<rclcpp::Parameter> & parameters);
  void tick();

  rclcpp::Node::SharedPtr node_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<LoadMission>::SharedPtr load_mission_service_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;

  std::vector<std::string> mission_parameters_;  // names currently declared on the node
  std::vector<std::string> modes_;  // of the running mission, empty if it has none
  std::string mode_;                // current one, mirrors the blackboard entry 'mode'
  double retry_delay_sec_{5.0};
  std::optional<rclcpp::Time> retry_at_;
  BT::NodeStatus last_status_{BT::NodeStatus::IDLE};
};

}  // namespace mission_behavior_tree
