// WaitForMessage against a real publisher (std_msgs/String, by type name).

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "mission_behavior_tree/wait_for_message_action.hpp"

namespace
{

class WaitForMessageTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_wait_for_message");
    factory_.registerNodeType<mission_behavior_tree::WaitForMessageAction>("WaitForMessage");
  }

  // Ticks at ~50 Hz, spinning the node in between like mission_bt_node does;
  // publishes on each round if `publish`.
  BT::NodeStatus run(const std::string & attributes, bool publish, double max_sec = 5.0)
  {
    auto bb = BT::Blackboard::create();
    bb->set("node", node_);
    auto tree = factory_.createTreeFromText(
      R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">
           <WaitForMessage )" + attributes + R"(/>
         </BehaviorTree></root>)", bb);

    auto pub = node_->create_publisher<std_msgs::msg::String>(kTopic, 10);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(max_sec);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
      if (publish) pub->publish(std_msgs::msg::String());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      rclcpp::spin_some(node_);
    }
    return status;
  }

  static constexpr const char * kTopic = "/test_wait_for_message";
  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

}  // namespace

TEST_F(WaitForMessageTest, SucceedsOnceAMessageArrives)
{
  EXPECT_EQ(run(R"(topic="/test_wait_for_message" type="std_msgs/msg/String")", true),
    BT::NodeStatus::SUCCESS);
}

TEST_F(WaitForMessageTest, BestEffortHearsReliablePublisher)
{
  EXPECT_EQ(run(R"(topic="/test_wait_for_message" type="std_msgs/msg/String" best_effort="true")",
    true), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitForMessageTest, FailsAfterTimeoutWithoutMessages)
{
  EXPECT_EQ(run(R"(topic="/test_wait_for_message" type="std_msgs/msg/String" timeout="0.3")",
    false, 2.0), BT::NodeStatus::FAILURE);
}

TEST_F(WaitForMessageTest, KeepsWaitingWithoutTimeout)
{
  EXPECT_EQ(run(R"(topic="/test_wait_for_message" type="std_msgs/msg/String")", false, 0.5),
    BT::NodeStatus::RUNNING);
}

TEST_F(WaitForMessageTest, FailsForUnknownType)
{
  EXPECT_EQ(run(R"(topic="/test_wait_for_message" type="no_such_pkg/msg/Nope")", false),
    BT::NodeStatus::FAILURE);
}
