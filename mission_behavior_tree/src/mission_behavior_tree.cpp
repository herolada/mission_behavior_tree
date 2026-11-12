#include "mission_behavior_tree/mission_behavior_tree.hpp"

#include <algorithm>
#include <cctype>
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

bool equalIgnoringCase(const std::string & a, const std::string & b)
{
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(), [](unsigned char x, unsigned char y) {
             return std::tolower(x) == std::tolower(y);
           });
}

// The entry of `modes` that `name` names, ignoring case.
std::vector<std::string>::const_iterator findMode(
  const std::vector<std::string> & modes, const std::string & name)
{
  return std::find_if(modes.begin(), modes.end(),
           [&name](const std::string & mode) {return equalIgnoringCase(mode, name);});
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
  // Before the first loadMission, which publishes the initial mode.
  mode_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "~/mode", rclcpp::QoS(1).transient_local());

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
  set_mode_service_ = node_->create_service<SetMode>(
    "~/set_mode",
    [this](const SetMode::Request::SharedPtr request,
    SetMode::Response::SharedPtr response) {onSetMode(request, response);});
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
  const auto modes = read(parameters, "modes", std::vector<std::string>{});
  auto initial_mode = read<std::string>(parameters, "initial_mode", "");

  if (tree_file.empty()) {
    throw std::runtime_error("Parameter 'tree_file' is not set.");
  }
  if (tick_rate_hz <= 0.0) {
    throw std::runtime_error("Parameter 'tick_rate_hz' must be positive.");
  }
  for (auto it = modes.begin(); it != modes.end(); ++it) {
    if (it->empty()) {
      throw std::runtime_error("Parameter 'modes' contains an empty mode.");
    }
    if (findMode(modes, *it) != it) {  // modes are matched ignoring case
      throw std::runtime_error("Parameter 'modes' lists '" + *it + "' twice.");
    }
  }
  if (modes.empty() && !initial_mode.empty()) {
    throw std::runtime_error("Parameter 'initial_mode' is set, but 'modes' is not.");
  }
  if (!modes.empty()) {
    if (initial_mode.empty()) {
      initial_mode = modes.front();
    }
    const auto it = findMode(modes, initial_mode);
    if (it == modes.end()) {
      throw std::runtime_error(
        "Parameter 'initial_mode' ('" + initial_mode + "') is not one of 'modes'.");
    }
    initial_mode = *it;  // spelled as in 'modes', which is what the tree matches
  }

  auto blackboard = BT::Blackboard::create();
  loadBlackboardParameters(parameters, *blackboard, node_->get_logger());
  // After the parameters, so they cannot be overridden.
  blackboard->set("node", node_);
  if (!modes.empty()) {
    blackboard->set("mode", initial_mode);
  }

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
  modes_ = modes;
  mode_ = initial_mode;
  publishMode();

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
    rclcpp::Parameter("modes", modes),
    rclcpp::Parameter("initial_mode", initial_mode),
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

void MissionBehaviorTree::onSetMode(
  const SetMode::Request::SharedPtr request, SetMode::Response::SharedPtr response)
{
  // Ignoring case, but stored spelled as in 'modes', which is what the tree matches.
  const auto it = findMode(modes_, request->mode);
  const std::string mode = it == modes_.end() ? request->mode : *it;
  if (modes_.empty()) {
    response->success = false;
    response->message = "The running mission has no modes.";
  } else if (it == modes_.end()) {
    std::string known;
    for (const auto & m : modes_) known += (known.empty() ? "" : ", ") + m;
    response->success = false;
    response->message = "Unknown mode '" + mode + "', expected one of: " + known + ".";
  } else if (mode == mode_) {
    response->success = true;
    response->message = "Already in mode " + mode + ".";
  } else {
    // The tree sees it on its next tick; the switch there halts the old branch.
    tree_.rootBlackboard()->set("mode", mode);
    response->success = true;
    response->message = "Mode " + mode_ + " → " + mode + ".";
    mode_ = mode;
    // A tree waiting out retry_delay after a failure picks the new mode up now
    // (e.g. STOP while a failing patrol is being retried).
    retry_at_.reset();
    publishMode();
  }

  if (response->success) {
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "Cannot set mode: %s", response->message.c_str());
  }
}

// Empty when the mission has no modes, so a latched old value does not linger.
void MissionBehaviorTree::publishMode()
{
  std_msgs::msg::String msg;
  msg.data = mode_;
  mode_publisher_->publish(msg);
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
    "tree_file", "tick_rate_hz", "retry_delay_sec", "groot_port", "bt_plugins", "modes",
    "initial_mode", "use_sim_time"};
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
