// ─────────────────────────────────────────────────────────────────────────────
// mission_bt_node – generic runner for mission behavior trees. Everything
// mission specific comes from the parameters (see config/*.yaml):
//   tree_file      XML to run, relative to share/mission_behavior_tree/behavior_trees
//   bt_plugins     BT.CPP plugin libraries to load (e.g. nav2 condition nodes)
//   blackboard.*   copied into the blackboard, so the XML can refer to them as {name}
// and ticks the tree from a timer, restarting it whenever it finishes.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/utils/shared_library.h>
#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/blackboard_parameters.hpp"
#include "mission_behavior_tree/operator_request_action.hpp"
#include "mission_behavior_tree/wait_for_message_action.hpp"

namespace mission_behavior_tree
{

namespace
{

// Parameters may already be declared from the YAML overrides (see NodeOptions
// in the constructor), in which case declare_parameter() would throw.
template<typename T>
T param(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) return node.declare_parameter(name, default_value);
  const auto p = node.get_parameter(name);
  if constexpr (std::is_same_v<T, double>) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(p.as_int());
    }
  }
  return p.get_value<T>();
}

}  // namespace

// Not an rclcpp::Node subclass on purpose: the blackboard holds the node, so the
// node must not own the tree as well.
class MissionBehaviorTree
{
public:
  MissionBehaviorTree()
  : node_(std::make_shared<rclcpp::Node>("mission_behavior_tree",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)))
  {
    auto tree_file = param<std::string>(*node_, "tree_file", "");
    const auto tick_rate_hz = param(*node_, "tick_rate_hz", 10.0);
    retry_delay_sec_ = param(*node_, "retry_delay_sec", 5.0);
    const auto groot_port = param<int64_t>(*node_, "groot_port", 1667);
    const auto bt_plugins = param(*node_, "bt_plugins", std::vector<std::string>{});
    warnAboutUnknownParameters();

    if (tree_file.empty()) {
      throw std::runtime_error("Parameter 'tree_file' is not set.");
    }
    if (std::filesystem::path(tree_file).is_relative()) {
      tree_file = ament_index_cpp::get_package_share_directory("mission_behavior_tree") +
        "/behavior_trees/" + tree_file;
    }

    auto blackboard = BT::Blackboard::create();
    for (const auto & key : loadBlackboardParameters(*node_, *blackboard)) {
      RCLCPP_INFO(node_->get_logger(), "  blackboard: %s = %s", key.c_str(),
        node_->get_parameter("blackboard." + key).value_to_string().c_str());
    }
    blackboard->set("node", node_);  // after the parameters, so it cannot be overridden

    BT::BehaviorTreeFactory factory;
    for (const auto & plugin : bt_plugins) {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }
    factory.registerNodeType<OperatorRequestAction>("OperatorRequest");
    factory.registerNodeType<WaitForMessageAction>("WaitForMessage");

    RCLCPP_INFO(node_->get_logger(), "Loading behavior tree '%s'.", tree_file.c_str());
    tree_ = factory.createTreeFromFile(tree_file, blackboard);

    if (groot_port > 0) {
      groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, static_cast<unsigned>(groot_port));
    }

    timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / tick_rate_hz), [this]() {tick();});
  }

  rclcpp::Node::SharedPtr node() const {return node_;}

  void shutdown()
  {
    timer_.reset();
    tree_.haltTree();
  }

private:
  // A mission parameter put at the top level instead of under 'blackboard:'
  // would otherwise be ignored silently.
  void warnAboutUnknownParameters()
  {
    static const std::set<std::string> known{
      "tree_file", "tick_rate_hz", "retry_delay_sec", "groot_port", "bt_plugins", "use_sim_time"};
    for (const auto & [name, value] :
      node_->get_node_parameters_interface()->get_parameter_overrides())
    {
      (void)value;
      if (known.count(name) || name.rfind("blackboard.", 0) == 0 ||
        name.rfind("qos_overrides.", 0) == 0)
      {
        continue;
      }
      RCLCPP_WARN(node_->get_logger(), "Unknown parameter '%s' ignored (mission parameters "
        "belong under 'blackboard:').", name.c_str());
    }
  }

  void tick()
  {
    const auto now = node_->now();
    if (retry_at_ && now < *retry_at_) return;
    retry_at_.reset();

    BT::NodeStatus status;
    try {
      status = tree_.tickOnce();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Behavior tree threw: %s", ex.what());
      status = BT::NodeStatus::FAILURE;
    }

    if (status != last_status_) {
      RCLCPP_INFO(node_->get_logger(), "Behavior tree: %s → %s",
        BT::toStr(last_status_).c_str(), BT::toStr(status).c_str());
      last_status_ = status;
    }

    if (status == BT::NodeStatus::RUNNING) return;

    // Finished: start over on the next tick – immediately after a success
    // (e.g. a non-looping patrol route done), after retry_delay on a failure.
    tree_.haltTree();
    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(node_->get_logger(), "Behavior tree failed, restarting in %.1f s.",
        retry_delay_sec_);
      retry_at_ = now + rclcpp::Duration::from_seconds(retry_delay_sec_);
    }
  }

  rclcpp::Node::SharedPtr node_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double retry_delay_sec_{5.0};
  std::optional<rclcpp::Time> retry_at_;
  BT::NodeStatus last_status_{BT::NodeStatus::IDLE};
};

}  // namespace mission_behavior_tree

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    mission_behavior_tree::MissionBehaviorTree mission;
    rclcpp::spin(mission.node());
    mission.shutdown();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_behavior_tree"), "%s", ex.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
