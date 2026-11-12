// Runs battery_patrol.xml with fake IsBatteryLow / OperatorRequest /
// WaitForMessage nodes, to check the branch switching and the charging
// hysteresis without the rest of the system. The blackboard is loaded from
// config/battery_patrol.yaml like mission_bt_node does, so the config is
// checked against the tree as well.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/blackboard_parameters.hpp"

#include "fake_bt_nodes.hpp"

namespace
{

class BatteryPatrolTree : public ::testing::Test
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
      rclcpp::parameter_map_from_yaml_file(BATTERY_PATROL_CONFIG, node_name), node_name);
    auto bb = BT::Blackboard::create();
    mission_behavior_tree::loadBlackboardParameters(
      parameters, *bb, rclcpp::get_logger("test_battery_patrol_tree"));

    tree_ = factory_.createTreeFromFile(BATTERY_PATROL_TREE_XML, bb);
  }

  BT::NodeStatus tickTimes(int n)
  {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    for (int i = 0; i < n; ++i) status = tree_.tickOnce();
    return status;
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

using Goals = std::vector<std::string>;

}  // namespace

TEST_F(BatteryPatrolTree, PatrolsOnceWithFullBattery)
{
  fake::battery = 50.0;
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, Goals{"patrol"});
}

TEST_F(BatteryPatrolTree, LowBatteryHaltsPatrolAndGoesHomeOnce)
{
  fake::battery = 50.0;
  tickTimes(3);
  fake::battery = 15.0;
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"patrol", "home"}));
  EXPECT_EQ(fake::halted, Goals{"patrol"});
}

TEST_F(BatteryPatrolTree, StaysHomeUntilChargedThenResumesPatrol)
{
  fake::battery = 15.0;
  tickTimes(3);
  fake::result["home"] = BT::NodeStatus::SUCCESS;  // arrived
  tickTimes(3);

  // Above battery_low but below battery_charged: the latch keeps it home.
  fake::battery = 50.0;
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, Goals{"home"});

  fake::battery = 95.0;
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"home", "patrol"}));

  // Latch released: dropping back below battery_charged keeps patrolling.
  fake::battery = 50.0;
  tickTimes(3);
  EXPECT_EQ(fake::sent, (Goals{"home", "patrol"}));
}

TEST_F(BatteryPatrolTree, FailingToGoHomeFailsTheTreeInsteadOfPatrolling)
{
  fake::battery = 15.0;
  tickTimes(2);
  fake::result["home"] = BT::NodeStatus::FAILURE;
  EXPECT_EQ(tickTimes(1), BT::NodeStatus::FAILURE);
  EXPECT_EQ(fake::sent, Goals{"home"});
}

TEST_F(BatteryPatrolTree, RestartAfterFailureWhileRechargingKeepsGoingHome)
{
  // Low → home; on the way the battery reads a little higher, then the home
  // request fails. The latch survives the restart, so it heads home again.
  fake::battery = 15.0;
  tickTimes(2);
  fake::battery = 50.0;
  fake::result["home"] = BT::NodeStatus::FAILURE;
  EXPECT_EQ(tickTimes(1), BT::NodeStatus::FAILURE);
  tree_.haltTree();

  fake::result.erase("home");
  EXPECT_EQ(tickTimes(2), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fake::sent, (Goals{"home", "home"}));
}

TEST_F(BatteryPatrolTree, LoopsThePatrolAndGoesHomeAlongWgsRoute)
{
  fake::battery = 100.0;
  tickTimes(3);
  fake::battery = 15.0;
  tickTimes(3);
  ASSERT_EQ(fake::sent, (Goals{"patrol", "home"}));

  EXPECT_TRUE(fake::request["patrol"].route);
  EXPECT_TRUE(fake::request["patrol"].loop);
  EXPECT_EQ(fake::request["patrol"].route_goal_point, "");

  EXPECT_TRUE(fake::request["home"].route);
  EXPECT_TRUE(fake::request["home"].wgs);
  EXPECT_FALSE(fake::request["home"].loop);
  EXPECT_EQ(fake::request["home"].route_goal_point, "home");
}

TEST(Nav2Plugins, IsBatteryLowLoadsWithThePortsTheTreeUses)
{
  // The same lookup mission_bt_node does for its default bt_plugins.
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("nav2_is_battery_low_condition_bt_node"));

  const auto & manifests = factory.manifests();
  const auto it = manifests.find("IsBatteryLow");
  ASSERT_NE(it, manifests.end());
  for (const char * port : {"min_battery", "battery_topic", "is_voltage"}) {
    EXPECT_EQ(it->second.ports.count(port), 1u) << port;
  }
}
