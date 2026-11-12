#include "mission_behavior_tree/wait_for_message_action.hpp"

namespace mission_behavior_tree
{

WaitForMessageAction::WaitForMessageAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus WaitForMessageAction::onStart()
{
  auto topic = getInput<std::string>("topic");
  auto type = getInput<std::string>("type");
  if (!topic || topic->empty() || !type || type->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] 'topic' and 'type' must be given.", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  topic_ = *topic;
  timeout_ = getInput<double>("timeout").value();

  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  if (getInput<bool>("best_effort").value()) qos.best_effort();

  received_ = std::make_shared<bool>(false);
  started_ = node_->now();
  try {
    // Not capturing `this`: when ~/load_mission destroys this tree, the executor
    // can still deliver a message it had already picked up.
    sub_ = node_->create_generic_subscription(topic_, *type, qos,
      [received = received_](std::shared_ptr<const rclcpp::SerializedMessage>) {*received = true;});
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Cannot subscribe to '%s' [%s]: %s",
      name().c_str(), topic_.c_str(), type->c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForMessageAction::onRunning()
{
  if (*received_) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Got a message on '%s'.", name().c_str(), topic_.c_str());
    return finish(BT::NodeStatus::SUCCESS);
  }
  const double waited = (node_->now() - started_).seconds();
  if (timeout_ > 0.0 && waited >= timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] No message on '%s' within %.1f s.",
      name().c_str(), topic_.c_str(), timeout_);
    return finish(BT::NodeStatus::FAILURE);
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "[%s] Waiting for a message on '%s'…", name().c_str(), topic_.c_str());
  return BT::NodeStatus::RUNNING;
}

void WaitForMessageAction::onHalted()
{
  finish(BT::NodeStatus::IDLE);
}

BT::NodeStatus WaitForMessageAction::finish(BT::NodeStatus status)
{
  sub_.reset();
  return status;
}

}  // namespace mission_behavior_tree
