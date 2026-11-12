// Runs patrol_home_stop.xml with fake nodes (fake_bt_nodes.hpp), switching the
// blackboard entry 'mode' the way ~/set_mode does. The blackboard is loaded
// from config/patrol_home_stop.yaml like mission_bt_node does, so the config
// (including its 'modes') is checked against the tree as well.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/blackboard_parameters.hpp"

#include "fake_bt_nodes.hpp"

namespace
{

using Goals = std::vector<std::string>;

class PatrolHomeStopTree : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    fake::reset();
    fake::registerNodes(factory_);

    // Same node name as mission_bt_node, so the YAML applies.
    const char * node_name = "/mission_behavior_tree";
    const auto parameters = rclcpp::parameters_from_map(
      rclcpp::parameter_map_from_yaml_file(PATROL_HOME_STOP_CONFIG, node_name), node_name);
    blackboard_ = BT::Blackboard::create();
    mission_behavior_tree::loadBlackboardParameters(
      parameters, *blackboard_, rclcpp::get_logger("test_patrol_home_stop_tree"));
    for (const auto & p : parameters) {
      if (p.get_name() == "modes") modes_ = p.as_string_array();
      if (p.get_name() == "initial_mode") blackboard_->set("mode", p.as_string());
    }

    tree_ = factory_.createTreeFromFile(PATROL_HOME_STOP_TREE_XML, blackboard_);
  }

  BT::NodeStatus tickTimes(int n)
  {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    for (int i = 0; i < n; ++i) status = tree_.tickOnce();
    return status;
  }

  void setMode(const std::string & mode) {blackboard_->set("mode", mode);}

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  std::vector<std::string> modes_;
};

}  // namespace

TEST_F(PatrolHomeStopTree, ConfigModesAllHaveABranch)
{
  ASSERT_EQ(modes_, (std::vector<std::string>{"PATROL", "HOME", "STOP"}));
  for (const auto & mode : modes_) {
    setMode(mode);
    EXPECT_EQ(tickTimes(2), BT::NodeStatus::RUNNING) << mode;
  }
  setMode("NO_SUCH_MODE");
  EXPECT_EQ(tickTimes(1), BT::NodeStatus::FAILURE);
}

TEST_F(PatrolHomeStopTree, StartsPatrolling)
{
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, Goals{"patrol"});
  EXPECT_TRUE(fake::switched.empty());
}

TEST_F(PatrolHomeStopTree, PatrolStillGoesHomeToChargeOnItsOwn)
{
  tickTimes(3);
  fake::battery = 15.0;
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "home"}));
  EXPECT_EQ(fake::halted, Goals{"patrol"});
}

TEST_F(PatrolHomeStopTree, HomeGoesHomeOnceAndHoldsUntilPatrol)
{
  tickTimes(3);
  setMode("HOME");
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "home"}));
  EXPECT_EQ(fake::halted, Goals{"patrol"});

  // Arrived: holds there, does not request home again.
  fake::result["home"] = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(tickTimes(10), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "home"}));

  // A full battery does not send it patrolling while in HOME.
  fake::battery = 100.0;
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "home"}));

  setMode("PATROL");
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "home", "patrol"}));
}

TEST_F(PatrolHomeStopTree, HomeUsesTheSameRequestAsGoingHomeToCharge)
{
  setMode("HOME");
  tickTimes(2);
  ASSERT_EQ(fake::sent, Goals{"home"});
  EXPECT_TRUE(fake::request["home"].route);
  EXPECT_TRUE(fake::request["home"].wgs);
  EXPECT_FALSE(fake::request["home"].loop);
  EXPECT_EQ(fake::request["home"].route_goal_point, "home");
}

TEST_F(PatrolHomeStopTree, StopHaltsPatrolStopsOnceAndHolds)
{
  tickTimes(3);
  setMode("STOP");
  EXPECT_EQ(tickTimes(10), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::switched, std::vector<std::string>{"stop"});
  EXPECT_EQ(fake::halted, Goals{"patrol"});
  EXPECT_EQ(fake::sent, Goals{"patrol"});

  setMode("PATROL");
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "patrol"}));
  EXPECT_EQ(fake::switched, std::vector<std::string>{"stop"});
}

TEST_F(PatrolHomeStopTree, StopWhileGoingHomeHaltsIt)
{
  setMode("HOME");
  tickTimes(2);
  setMode("STOP");
  tickTimes(2);
  EXPECT_EQ(fake::halted, Goals{"home"});
  EXPECT_EQ(fake::switched, std::vector<std::string>{"stop"});
}

TEST_F(PatrolHomeStopTree, BackToPatrolWhileStillChargingGoesHomeAgain)
{
  // The recharging latch outlives the other modes: the battery is still below
  // battery_charged, so PATROL resumes charging rather than patrolling.
  fake::battery = 15.0;
  tickTimes(2);
  fake::result["home"] = BT::NodeStatus::SUCCESS;
  tickTimes(2);
  setMode("STOP");
  fake::battery = 50.0;
  tickTimes(2);
  setMode("PATROL");
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"home", "home"}));
}

TEST_F(PatrolHomeStopTree, FailingHomeFailsTheTreeAndKeepsTheMode)
{
  setMode("HOME");
  tickTimes(1);
  fake::result["home"] = BT::NodeStatus::FAILURE;
  EXPECT_EQ(tickTimes(1), BT::NodeStatus::FAILURE);
  tree_.haltTree();  // as mission_bt_node does before restarting

  fake::result.erase("home");
  EXPECT_EQ(tickTimes(2), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"home", "home"}));
}
