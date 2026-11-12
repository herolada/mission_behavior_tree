// CommanderSwitchMode against a fake crl_commander service.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/commander_switch_mode_action.hpp"

namespace
{

using SwitchMode = mission_behavior_tree::CommanderSwitchModeAction::SwitchMode;

class CommanderSwitchModeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_commander_switch_mode");
    factory_.registerNodeType<mission_behavior_tree::CommanderSwitchModeAction>(
      "CommanderSwitchMode");
  }

  void startCommander(bool accept)
  {
    service_ = node_->create_service<SwitchMode>(kService,
        [this, accept](const SwitchMode::Request::SharedPtr request,
        SwitchMode::Response::SharedPtr response) {
          received_.push_back(request->mode);
          response->success = accept;
          response->message = "fake answer";
        });
  }

  // Ticks at ~50 Hz, spinning the node in between like mission_bt_node does.
  BT::NodeStatus run(const std::string & attributes, double max_sec = 5.0)
  {
    auto bb = BT::Blackboard::create();
    bb->set("node", node_);
    auto tree = factory_.createTreeFromText(
      R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">
           <CommanderSwitchMode service="/test_commander_switch_mode/switch_mode" )" +
      attributes + R"(/>
         </BehaviorTree></root>)", bb);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(max_sec);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      rclcpp::spin_some(node_);
    }
    return status;
  }

  static constexpr const char * kService = "/test_commander_switch_mode/switch_mode";
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<SwitchMode>::SharedPtr service_;
  std::vector<std::string> received_;
  BT::BehaviorTreeFactory factory_;
};

}  // namespace

TEST_F(CommanderSwitchModeTest, SucceedsAndSendsOnceWhenAccepted)
{
  startCommander(true);
  EXPECT_EQ(run(R"(mode="stop")"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(received_, std::vector<std::string>{"stop"});
}

TEST_F(CommanderSwitchModeTest, FailsWhenRejected)
{
  startCommander(false);
  EXPECT_EQ(run(R"(mode="no_such_mode")"), BT::NodeStatus::FAILURE);
}

TEST_F(CommanderSwitchModeTest, FailsAfterTimeoutWithoutService)
{
  EXPECT_EQ(run(R"(mode="stop" timeout="0.3")", 2.0), BT::NodeStatus::FAILURE);
}

TEST_F(CommanderSwitchModeTest, FailsWithoutMode)
{
  startCommander(true);
  EXPECT_EQ(run(""), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(received_.empty());
}
