#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "mission_behavior_tree/mission_behavior_tree.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    mission_behavior_tree::MissionBehaviorTree mission;
    rclcpp::spin(mission.node());
    mission.shutdown();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_behavior_tree"), "%s", ex.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
