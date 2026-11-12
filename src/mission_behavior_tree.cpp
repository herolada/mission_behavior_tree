#include "mission_behavior_tree/mission_behavior_tree.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/utils/shared_library.h>
#include <rclcpp/parameter_map.hpp>

#include "mission_behavior_tree/blackboard_parameters.hpp"
#include "mission_behavior_tree/commander_switch_mode_action.hpp"
#include "mission_behavior_tree/operator_request_action.hpp"
#include "mission_behavior_tree/wait_for_message_action.hpp"

namespace mission_behavior_tree
{

namespace
{

const std::string kBlackboardPrefix = "blackboard.";

bool isBlackboardParameter(const std::string & name)
{
  return name.rfind(kBlackboardPrefix, 0) == 0;
}

template<typename T>
T read(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name,
  const T & default_value)
{
  const auto it = std::find_if(parameters.begin(), parameters.end(),
      [&name](const rclcpp::Parameter & p) {return p.get_name() == name;});
  if (it == parameters.end()) return default_value;
  if constexpr (std::is_same_v<T, double>) {
    if (it->get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(it->as_int());
    }
  }
  return it->template get_value<T>();
}

std::string inPackageShare(const std::string & subdirectory, const std::string & file)
{
  if (std::filesystem::path(file).is_absolute()) return file;
  return ament_index_cpp::get_package_share_directory("mission_behavior_tree") + "/" +
         subdirectory + "/" + file;
}

}  // namespace

MissionBehaviorTree::MissionBehaviorTree(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("mission_behavior_tree", options))
{
  std::vector<rclcpp::Parameter> parameters;
  for (const auto & [name, value] :
    node_->get_node_parameters_interface()->get_parameter_overrides())
  {
    parameters.emplace_back(name, value);
  }
  loadMission(parameters);

  load_mission_service_ = node_->create_service<LoadMission>(
    "~/load_mission",
    [this](const LoadMission::Request::SharedPtr request,
    LoadMission::Response::SharedPtr response) {onLoadMission(request, response);});
}

void MissionBehaviorTree::shutdown()
{
  timer_.reset();
  tree_.haltTree();
}

void MissionBehaviorTree::loadMission(const std::vector<rclcpp::Parameter> & parameters)
{
  warnAboutUnknownParameters(parameters);

  const auto tree_file = read<std::string>(parameters, "tree_file", "");
  const auto tick_rate_hz = read(parameters, "tick_rate_hz", 10.0);
  const auto retry_delay_sec = read(parameters, "retry_delay_sec", 5.0);
  const auto groot_port = read<int64_t>(parameters, "groot_port", 1667);
  const auto bt_plugins = read(parameters, "bt_plugins", std::vector<std::string>{});

  if (tree_file.empty()) {
    throw std::runtime_error("Parameter 'tree_file' is not set.");
  }
  if (tick_rate_hz <= 0.0) {
    throw std::runtime_error("Parameter 'tick_rate_hz' must be positive.");
  }

  auto blackboard = BT::Blackboard::create();
  loadBlackboardParameters(parameters, *blackboard, node_->get_logger());
  blackboard->set("node", node_);  // after the parameters, so it cannot be overridden

  BT::BehaviorTreeFactory factory;
  for (const auto & plugin : bt_plugins) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  }
  factory.registerNodeType<CommanderSwitchModeAction>("CommanderSwitchMode");
  factory.registerNodeType<OperatorRequestAction>("OperatorRequest");
  factory.registerNodeType<WaitForMessageAction>("WaitForMessage");

  const auto tree_path = inPackageShare("behavior_trees", tree_file);
  RCLCPP_INFO(node_->get_logger(), "Loading behavior tree '%s'.", tree_path.c_str());
  auto tree = factory.createTreeFromFile(tree_path, blackboard);

  // Nothing below may fail: the new mission is valid, the old one is replaced.
  if (timer_) {
    timer_.reset();
    tree_.haltTree();
  }
  groot_publisher_.reset();  // before its tree, and it has to free the port for the new one
  tree_ = std::move(tree);

  retry_delay_sec_ = retry_delay_sec;
  retry_at_.reset();
  last_status_ = BT::NodeStatus::IDLE;

  if (groot_port > 0) {
    try {
      groot_publisher_ = std::make_unique<BT::Groot2Publisher>(
        tree_, static_cast<unsigned>(groot_port));
    } catch (const std::exception & ex) {
      RCLCPP_WARN(node_->get_logger(), "Groot2 monitoring disabled: %s", ex.what());
    }
  }

  std::vector<rclcpp::Parameter> effective{
    rclcpp::Parameter("tree_file", tree_file),
    rclcpp::Parameter("tick_rate_hz", tick_rate_hz),
    rclcpp::Parameter("retry_delay_sec", retry_delay_sec),
    rclcpp::Parameter("groot_port", groot_port),
    rclcpp::Parameter("bt_plugins", bt_plugins),
  };
  std::copy_if(parameters.begin(), parameters.end(), std::back_inserter(effective),
    [](const rclcpp::Parameter & p) {return isBlackboardParameter(p.get_name());});
  replaceNodeParameters(effective);

  timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / tick_rate_hz), [this]() {tick();});
}

void MissionBehaviorTree::onLoadMission(
  const LoadMission::Request::SharedPtr request, LoadMission::Response::SharedPtr response)
{
  try {
    const auto file = inPackageShare("config", request->config_file);
    const std::string node_name = node_->get_fully_qualified_name();
    const auto parameters = rclcpp::parameters_from_map(
      rclcpp::parameter_map_from_yaml_file(file, node_name.c_str()), node_name.c_str());
    if (parameters.empty()) {
      throw std::runtime_error("'" + file + "' has no parameters for node '" + node_name + "'.");
    }
    RCLCPP_INFO(node_->get_logger(), "Loading mission '%s'.", file.c_str());
    loadMission(parameters);
    response->success = true;
    response->message = "Loaded '" + file + "'.";
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load mission '%s', keeping the current one: %s",
      request->config_file.c_str(), ex.what());
    response->success = false;
    response->message = ex.what();
  }
}

void MissionBehaviorTree::replaceNodeParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & name : mission_parameters_) {
    node_->undeclare_parameter(name);
  }
  mission_parameters_.clear();

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;  // the next mission may use another type
  for (const auto & parameter : parameters) {
    // The overrides from process start outlive every mission, and would win
    // over the value given here unless ignored.
    node_->declare_parameter(
      parameter.get_name(), parameter.get_parameter_value(), descriptor, true);
    mission_parameters_.push_back(parameter.get_name());
    RCLCPP_INFO(node_->get_logger(), "  %s = %s", parameter.get_name().c_str(),
      parameter.value_to_string().c_str());
  }
}

// A mission parameter put at the top level instead of under 'blackboard:'
// would otherwise be ignored silently.
void MissionBehaviorTree::warnAboutUnknownParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  static const std::set<std::string> known{
    "tree_file", "tick_rate_hz", "retry_delay_sec", "groot_port", "bt_plugins", "use_sim_time"};
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (known.count(name) || isBlackboardParameter(name) || name.rfind("qos_overrides.", 0) == 0) {
      continue;
    }
    RCLCPP_WARN(node_->get_logger(), "Unknown parameter '%s' ignored (mission parameters "
      "belong under 'blackboard:').", name.c_str());
  }
}

void MissionBehaviorTree::tick()
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

}  // namespace mission_behavior_tree
