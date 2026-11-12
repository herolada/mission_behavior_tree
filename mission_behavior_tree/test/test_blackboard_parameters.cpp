// Checks that parameters of every type end up in the blackboard in a form the
// typed ports (and scripts) of a tree can read.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "mission_behavior_tree/blackboard_parameters.hpp"

namespace
{

struct Read
{
  double as_double{0.0};
  int as_int{0};
  unsigned as_unsigned{0};
  std::vector<double> doubles;
  std::vector<int> ints;
  std::vector<std::string> strings;
  std::string text;
};
Read g_read;

class ReadPorts : public BT::SyncActionNode
{
public:
  ReadPorts(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("as_double"),
      BT::InputPort<int>("as_int"),
      BT::InputPort<unsigned>("as_unsigned"),
      BT::InputPort<std::vector<double>>("doubles"),
      BT::InputPort<std::vector<int>>("ints"),
      BT::InputPort<std::vector<std::string>>("strings"),
      BT::InputPort<std::string>("text"),
    };
  }

  BT::NodeStatus tick() override
  {
    g_read.as_double = getInput<double>("as_double").value();
    g_read.as_int = getInput<int>("as_int").value();
    g_read.as_unsigned = getInput<unsigned>("as_unsigned").value();
    g_read.doubles = getInput<std::vector<double>>("doubles").value();
    g_read.ints = getInput<std::vector<int>>("ints").value();
    g_read.strings = getInput<std::vector<std::string>>("strings").value();
    g_read.text = getInput<std::string>("text").value();
    return BT::NodeStatus::SUCCESS;
  }
};

const char * kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="T">
  <BehaviorTree ID="T">
    <Sequence>
      <ReadPorts as_double="{an_int}" as_int="{an_int}" as_unsigned="{an_int}"
                 doubles="{double_array}" ints="{int_array}" strings="{string_array}"
                 text="{a_string}"/>
      <ScriptCondition code="flag == true &amp;&amp; ratio > 0.25"/>
      <SetBlackboard output_key="flag" value="false"/>
      <ScriptCondition code="!flag"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

}  // namespace

TEST(BlackboardParameters, EveryTypeReachesTypedPortsAndScripts)
{
  auto bb = BT::Blackboard::create();
  for (const auto & p : std::vector<rclcpp::Parameter>{
      {"an_int", 30},
      {"ratio", 0.3},
      {"flag", true},
      {"a_string", "hello"},
      {"double_array", std::vector<double>{0.1, 2.5}},
      {"int_array", std::vector<int64_t>{1, -2}},
      {"string_array", std::vector<std::string>{"a", "b"}}})
  {
    ASSERT_TRUE(mission_behavior_tree::setBlackboardFromParameter(*bb, p.get_name(), p))
      << p.get_name();
  }

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ReadPorts>("ReadPorts");
  auto tree = factory.createTreeFromText(kTree, bb);
  ASSERT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);

  EXPECT_DOUBLE_EQ(g_read.as_double, 30.0);
  EXPECT_EQ(g_read.as_int, 30);
  EXPECT_EQ(g_read.as_unsigned, 30u);
  EXPECT_EQ(g_read.doubles, (std::vector<double>{0.1, 2.5}));
  EXPECT_EQ(g_read.ints, (std::vector<int>{1, -2}));
  EXPECT_EQ(g_read.strings, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ(g_read.text, "hello");
  EXPECT_FALSE(bb->get<bool>("flag"));
}

TEST(BlackboardParameters, ByteArraysAndUnsetAreRejected)
{
  BT::Blackboard::Ptr bb = BT::Blackboard::create();
  EXPECT_FALSE(mission_behavior_tree::setBlackboardFromParameter(
    *bb, "bytes", rclcpp::Parameter("bytes", std::vector<uint8_t>{1, 2})));
  EXPECT_FALSE(mission_behavior_tree::setBlackboardFromParameter(
    *bb, "unset", rclcpp::Parameter("unset")));
  EXPECT_EQ(bb->getEntry("bytes"), nullptr);
}
