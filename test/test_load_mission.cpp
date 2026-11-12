// ~/load_mission on a running MissionBehaviorTree, with the real mission configs
// and trees (paths and ports rewritten) and a fake crl_commander service.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <crl_commander_interfaces/srv/switch_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/mission_behavior_tree.hpp"
#include "mission_behavior_tree/srv/load_mission.hpp"

namespace
{

using LoadMission = mission_behavior_tree::srv::LoadMission;
using SwitchMode = crl_commander_interfaces::srv::SwitchMode;
namespace fs = std::filesystem;

constexpr const char * kSwitchMode = "/test_load_mission/switch_mode";

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

class LoadMissionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    dir_ = fs::temp_directory_path() / ("test_load_mission_" + std::to_string(::getpid()));
    fs::create_directories(dir_);

    helper_ = std::make_shared<rclcpp::Node>("test_load_mission_helper");
    switch_mode_ = helper_->create_service<SwitchMode>(kSwitchMode,
        [this](const SwitchMode::Request::SharedPtr request,
        SwitchMode::Response::SharedPtr response) {
          const std::lock_guard<std::mutex> lock(mutex_);
          modes_.push_back(request->mode);
          response->success = true;
        });
    load_mission_ = helper_->create_client<LoadMission>("/mission_behavior_tree/load_mission");
    executor_.add_node(helper_);
    spinner_ = std::thread([this]() {executor_.spin();});
  }

  void TearDown() override
  {
    if (mission_) mission_->shutdown();
    executor_.cancel();
    spinner_.join();
    if (mission_) executor_.remove_node(mission_->node());
    executor_.remove_node(helper_);
    mission_.reset();
    fs::remove_all(dir_);
  }

  void startMission(const std::vector<rclcpp::Parameter> & parameters)
  {
    // Added while the executor spins: it picks new nodes up by itself.
    mission_ = std::make_unique<mission_behavior_tree::MissionBehaviorTree>(
      rclcpp::NodeOptions().parameter_overrides(parameters));
    executor_.add_node(mission_->node());
  }

  std::string write(const std::string & name, const std::string & text)
  {
    const auto path = (dir_ / name).string();
    std::ofstream(path) << text;
    return path;
  }

  // A tree that requests `mode` once and stays RUNNING, to see which one runs.
  std::string modeTree(const std::string & name, const std::string & mode)
  {
    return write(name,
      R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T"><Sequence>
           <CommanderSwitchMode mode=")" + mode + R"(" service="/test_load_mission/switch_mode"/>
           <KeepRunningUntilFailure><AlwaysSuccess/></KeepRunningUntilFailure>
         </Sequence></BehaviorTree></root>)");
  }

  std::string config(const std::string & tree, const std::string & rest = "")
  {
    return "mission_behavior_tree:\n  ros__parameters:\n    tree_file: " + tree +
           "\n    groot_port: 0\n" + rest;
  }

  // One of the mission configs of this package, runnable from dir_.
  std::string realConfig(const std::string & source, const std::string & tree)
  {
    auto text = std::regex_replace(readFile(source), std::regex(R"(tree_file: \S+)"),
        "tree_file: " + tree);
    text = std::regex_replace(text, std::regex(R"(groot_port: \d+)"), "groot_port: 0");
    text = std::regex_replace(text, std::regex(R"(/crl_commander/switch_mode)"), kSwitchMode);
    return write(fs::path(source).filename().string(), text);
  }

  LoadMission::Response::SharedPtr load(const std::string & file)
  {
    EXPECT_TRUE(load_mission_->wait_for_service(std::chrono::seconds(5)));
    auto request = std::make_shared<LoadMission::Request>();
    request->config_file = file;
    auto future = load_mission_->async_send_request(request);
    EXPECT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    return future.get();
  }

  std::vector<std::string> modes()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return modes_;
  }

  bool waitForModes(size_t count, double max_sec = 10.0)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(max_sec);
    while (modes().size() < count && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return modes().size() >= count;
  }

  rclcpp::Node & node() {return *mission_->node();}

  fs::path dir_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Service<SwitchMode>::SharedPtr switch_mode_;
  rclcpp::Client<LoadMission>::SharedPtr load_mission_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  std::unique_ptr<mission_behavior_tree::MissionBehaviorTree> mission_;
  std::mutex mutex_;
  std::vector<std::string> modes_;
};

}  // namespace

TEST_F(LoadMissionTest, ReplacesTreeAndForgetsOldParameters)
{
  startMission({
    rclcpp::Parameter("tree_file", modeTree("first.xml", "first")),
    rclcpp::Parameter("groot_port", 0),
    rclcpp::Parameter("retry_delay_sec", 1.5),
    rclcpp::Parameter("blackboard.only_in_first", "x"),
    rclcpp::Parameter("blackboard.shared", "from_first"),
  });
  ASSERT_TRUE(waitForModes(1));

  const auto file = write("second.yaml",
      config(modeTree("second.xml", "second"), "    blackboard:\n      shared: from_second\n"));
  const auto response = load(file);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(waitForModes(2));
  EXPECT_EQ(modes(), (std::vector<std::string>{"first", "second"}));

  EXPECT_FALSE(node().has_parameter("blackboard.only_in_first"));
  EXPECT_EQ(node().get_parameter("blackboard.shared").as_string(), "from_second");
  EXPECT_DOUBLE_EQ(node().get_parameter("retry_delay_sec").as_double(), 5.0)
    << "a key missing from the new file must not come back from the startup overrides";
  EXPECT_TRUE(node().has_parameter("use_sim_time"));
}

TEST_F(LoadMissionTest, BadConfigsLeaveTheRunningMissionAlone)
{
  const auto first_tree = modeTree("first.xml", "first");
  startMission({
    rclcpp::Parameter("tree_file", first_tree),
    rclcpp::Parameter("groot_port", 0),
    rclcpp::Parameter("blackboard.key", "kept"),
  });
  ASSERT_TRUE(waitForModes(1));

  const auto unknown_node = write("unknown_node.xml",
      R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">
           <NoSuchNode/>
         </BehaviorTree></root>)");
  const std::vector<std::string> bad_files{
    (dir_ / "missing.yaml").string(),
    write("not_for_us.yaml", "other_node:\n  ros__parameters:\n    tree_file: x.xml\n"),
    write("no_tree.yaml", "mission_behavior_tree:\n  ros__parameters:\n    groot_port: 0\n"),
    write("missing_tree.yaml", config((dir_ / "missing.xml").string())),
    write("unknown_node.yaml", config(unknown_node)),
    write("bad_rate.yaml", config(first_tree, "    tick_rate_hz: 0.0\n")),
  };
  for (const auto & file : bad_files) {
    const auto response = load(file);
    EXPECT_FALSE(response->success) << file;
    EXPECT_FALSE(response->message.empty()) << file;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(modes(), std::vector<std::string>{"first"}) << "the tree must not have restarted";
  EXPECT_EQ(node().get_parameter("blackboard.key").as_string(), "kept");
}

TEST_F(LoadMissionTest, StopMissionRequestsStopOnceAndHolds)
{
  startMission({
    rclcpp::Parameter("tree_file", modeTree("first.xml", "first")),
    rclcpp::Parameter("groot_port", 0),
  });
  ASSERT_TRUE(waitForModes(1));

  const auto response = load(realConfig(STOP_CONFIG, STOP_TREE_XML));
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(waitForModes(2));

  std::this_thread::sleep_for(std::chrono::seconds(1));  // ~10 ticks
  EXPECT_EQ(modes(), (std::vector<std::string>{"first", "stop"}));
}

// Also covers loading the same BT plugin again in a new factory.
TEST_F(LoadMissionTest, SwitchesBetweenRealMissionsRepeatedly)
{
  const auto patrol = realConfig(BATTERY_PATROL_CONFIG, BATTERY_PATROL_TREE_XML);
  const auto stop = realConfig(STOP_CONFIG, STOP_TREE_XML);

  startMission({
    rclcpp::Parameter("tree_file", modeTree("first.xml", "first")),
    rclcpp::Parameter("groot_port", 0),
  });
  ASSERT_TRUE(waitForModes(1));

  size_t stops = 1;
  for (int i = 0; i < 3; ++i) {
    const auto to_patrol = load(patrol);
    ASSERT_TRUE(to_patrol->success) << to_patrol->message;
    EXPECT_TRUE(node().has_parameter("blackboard.patrol_route"));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const auto to_stop = load(stop);
    ASSERT_TRUE(to_stop->success) << to_stop->message;
    EXPECT_FALSE(node().has_parameter("blackboard.patrol_route"));
    ASSERT_TRUE(waitForModes(++stops));
  }
}

TEST_F(LoadMissionTest, GrootPortIsReusedByTheNextMission)
{
  constexpr int kPort = 16710;
  startMission({
    rclcpp::Parameter("tree_file", modeTree("first.xml", "first")),
    rclcpp::Parameter("groot_port", kPort),
  });
  ASSERT_TRUE(waitForModes(1));

  const auto response = load(write("second.yaml",
      "mission_behavior_tree:\n  ros__parameters:\n    tree_file: " +
      modeTree("second.xml", "second") + "\n    groot_port: " + std::to_string(kPort) + "\n"));
  ASSERT_TRUE(response->success) << response->message;

  // The runner only warns when it cannot publish, so check the port is taken.
  BT::BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">
         <AlwaysSuccess/></BehaviorTree></root>)");
  EXPECT_ANY_THROW(BT::Groot2Publisher(tree, kPort));
}
