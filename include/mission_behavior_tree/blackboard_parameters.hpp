#pragma once

#include <string>
#include <vector>

#include <behaviortree_cpp/blackboard.h>
#include <rclcpp/rclcpp.hpp>

namespace mission_behavior_tree
{

// Stores one ROS parameter in the blackboard under `key`:
//   - bool, double, string: as such.
//   - integer: as a string, so it can feed int, unsigned and double ports alike
//     (a typed int64 entry would be rejected by every port not declared int64).
//   - arrays: as a ';'-joined string, BT.CPP's format for std::vector ports.
// Returns false (and stores nothing) for byte arrays and unset parameters.
bool setBlackboardFromParameter(
  BT::Blackboard & blackboard, const std::string & key, const rclcpp::Parameter & param);

// Copies every "<prefix>.<key>" entry of `parameters` into the blackboard as
// <key> and returns the keys set. Parameters that cannot be stored are skipped
// with a warning.
std::vector<std::string> loadBlackboardParameters(
  const std::vector<rclcpp::Parameter> & parameters, BT::Blackboard & blackboard,
  const rclcpp::Logger & logger, const std::string & prefix = "blackboard");

}  // namespace mission_behavior_tree
