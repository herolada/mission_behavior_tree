#include "mission_behavior_tree/commander_switch_mode_action.hpp"

#include <chrono>

namespace mission_behavior_tree
{

CommanderSwitchModeAction::CommanderSwitchModeAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<SwitchMode>(getInput<std::string>("service").value());
}

BT::NodeStatus CommanderSwitchModeAction::fail(const std::string & reason)
{
  RCLCPP_ERROR(node_->get_logger(), "[%s] Switching to mode '%s' failed: %s",
    name().c_str(), mode_.c_str(), reason.c_str());
  forgetPendingRequest();
  return BT::NodeStatus::FAILURE;
}

void CommanderSwitchModeAction::forgetPendingRequest()
{
  if (pending_response_) {
    client_->remove_pending_request(pending_response_->request_id);
    pending_response_.reset();
  }
}

BT::NodeStatus CommanderSwitchModeAction::onStart()
{
  const auto mode = getInput<std::string>("mode");
  mode_ = mode ? *mode : "";
  if (mode_.empty()) return fail("no 'mode' given");

  timeout_ = getInput<double>("timeout").value();
  started_ = node_->now().seconds();
  forgetPendingRequest();
  return onRunning();  // sending happens there, so a commander not up yet is waited for
}

BT::NodeStatus CommanderSwitchModeAction::onRunning()
{
  if (!pending_response_) {
    if (client_->service_is_ready()) {
      auto request = std::make_shared<SwitchMode::Request>();
      request->mode = mode_;
      pending_response_.emplace(client_->async_send_request(request));
      RCLCPP_INFO(node_->get_logger(), "[%s] Requested mode '%s'.", name().c_str(), mode_.c_str());
      return BT::NodeStatus::RUNNING;
    }
    if (node_->now().seconds() - started_ >= timeout_) {
      return fail("service '" + std::string(client_->get_service_name()) + "' not available");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (pending_response_->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return node_->now().seconds() - started_ >= timeout_ ?
           fail("no response from the commander") : BT::NodeStatus::RUNNING;
  }

  const auto response = pending_response_->future.get();
  pending_response_.reset();
  return response->success ? BT::NodeStatus::SUCCESS : fail(response->message);
}

void CommanderSwitchModeAction::onHalted()
{
  forgetPendingRequest();
}

}  // namespace mission_behavior_tree
