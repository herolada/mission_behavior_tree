// Runs battery_patrol.xml with fake IsBatteryLow / OperatorRequest /
// WaitForMessage nodes, to check the branch switching and the charging
// hysteresis without the rest of the system. The blackboard is loaded from
// config/battery_patrol.yaml like mission_bt_node does, so the config is
// checked against the tree as well.

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/blackboard_parameters.hpp"
#include "mission_behavior_tree/operator_request_action.hpp"
#include "mission_behavior_tree/wait_for_message_action.hpp"

namespace
{

double g_battery = 100.0;
std::vector<std::string> g_sent;    // goal of every request sent, in order
std::vector<std::string> g_halted;  // goal of every request halted, in order
std::map<std::string, BT::NodeStatus> g_result;  // onRunning() result per goal
// Latest request per goal, as read from the ports.
std::map<std::string, commander_operator_interfaces::msg::OperatorRequest> g_request;

class FakeBatteryLow : public BT::ConditionNode
{
public:
  FakeBatteryLow(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery"),
      BT::InputPort<std::string>("battery_topic"),
      BT::InputPort<bool>("is_voltage"),
    };
  }

  BT::NodeStatus tick() override
  {
    return g_battery <= getInput<double>("min_battery").value() ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class FakeOperatorRequest : public BT::StatefulActionNode
{
public:
  FakeOperatorRequest(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return mission_behavior_tree::OperatorRequestAction::providedPorts();
  }

  BT::NodeStatus onStart() override
  {
    goal_ = getInput<std::string>("goal").value();
    g_sent.push_back(goal_);
    auto & request = g_request[goal_];
    request.goal = goal_;
    request.route = getInput<bool>("route").value();
    request.wgs = getInput<bool>("wgs").value();
    request.route_goal_point = getInput<std::string>("route_goal_point").value();
    request.loop = getInput<bool>("loop").value();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto it = g_result.find(goal_);
    return it == g_result.end() ? BT::NodeStatus::RUNNING : it->second;
  }

  void onHalted() override {g_halted.push_back(goal_);}

private:
  std::string goal_;
};

class BatteryPatrolTree : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    g_battery = 100.0;
    g_sent.clear();
    g_halted.clear();
    g_result.clear();
    g_request.clear();

    factory_.registerNodeType<FakeBatteryLow>("IsBatteryLow");
    factory_.registerNodeType<FakeOperatorRequest>("OperatorRequest");
    factory_.registerSimpleAction("WaitForMessage",
      [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
      mission_behavior_tree::WaitForMessageAction::providedPorts());

    // Same node name and options as mission_bt_node, so the YAML applies.
    auto node = std::make_shared<rclcpp::Node>("mission_behavior_tree",
      rclcpp::NodeOptions()
      .arguments({"--ros-args", "--params-file", BATTERY_PATROL_CONFIG})
      .automatically_declare_parameters_from_overrides(true));
    auto bb = BT::Blackboard::create();
    mission_behavior_tree::loadBlackboardParameters(*node, *bb);

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
  g_battery = 50.0;
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_sent, Goals{"patrol"});
}

TEST_F(BatteryPatrolTree, LowBatteryHaltsPatrolAndGoesHomeOnce)
{
  g_battery = 50.0;
  tickTimes(3);
  g_battery = 15.0;
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_sent, (Goals{"patrol", "home"}));
  EXPECT_EQ(g_halted, Goals{"patrol"});
}

TEST_F(BatteryPatrolTree, StaysHomeUntilChargedThenResumesPatrol)
{
  g_battery = 15.0;
  tickTimes(3);
  g_result["home"] = BT::NodeStatus::SUCCESS;  // arrived
  tickTimes(3);

  // Above battery_low but below battery_charged: the latch keeps it home.
  g_battery = 50.0;
  EXPECT_EQ(tickTimes(5), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_sent, Goals{"home"});

  g_battery = 95.0;
  EXPECT_EQ(tickTimes(3), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_sent, (Goals{"home", "patrol"}));

  // Latch released: dropping back below battery_charged keeps patrolling.
  g_battery = 50.0;
  tickTimes(3);
  EXPECT_EQ(g_sent, (Goals{"home", "patrol"}));
}

TEST_F(BatteryPatrolTree, FailingToGoHomeFailsTheTreeInsteadOfPatrolling)
{
  g_battery = 15.0;
  tickTimes(2);
  g_result["home"] = BT::NodeStatus::FAILURE;
  EXPECT_EQ(tickTimes(1), BT::NodeStatus::FAILURE);
  EXPECT_EQ(g_sent, Goals{"home"});
}

TEST_F(BatteryPatrolTree, RestartAfterFailureWhileRechargingKeepsGoingHome)
{
  // Low → home; on the way the battery reads a little higher, then the home
  // request fails. The latch survives the restart, so it heads home again.
  g_battery = 15.0;
  tickTimes(2);
  g_battery = 50.0;
  g_result["home"] = BT::NodeStatus::FAILURE;
  EXPECT_EQ(tickTimes(1), BT::NodeStatus::FAILURE);
  tree_.haltTree();

  g_result.erase("home");
  EXPECT_EQ(tickTimes(2), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_sent, (Goals{"home", "home"}));
}

TEST_F(BatteryPatrolTree, LoopsThePatrolAndGoesHomeAlongWgsRoute)
{
  g_battery = 100.0;
  tickTimes(3);
  g_battery = 15.0;
  tickTimes(3);
  ASSERT_EQ(g_sent, (Goals{"patrol", "home"}));

  EXPECT_TRUE(g_request["patrol"].route);
  EXPECT_TRUE(g_request["patrol"].loop);
  EXPECT_EQ(g_request["patrol"].route_goal_point, "");

  EXPECT_TRUE(g_request["home"].route);
  EXPECT_TRUE(g_request["home"].wgs);
  EXPECT_FALSE(g_request["home"].loop);
  EXPECT_EQ(g_request["home"].route_goal_point, "home");
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
