#include "mission_behavior_tree/blackboard_parameters.hpp"

#include <charconv>

namespace mission_behavior_tree
{

namespace
{

std::string toString(bool value) {return value ? "true" : "false";}
std::string toString(int64_t value) {return std::to_string(value);}
std::string toString(const std::string & value) {return value;}

std::string toString(double value)
{
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);  // shortest round-trip
  return std::string(buffer, result.ptr);
}

template<typename T>
std::string join(const std::vector<T> & values)
{
  std::string out;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) out += ';';
    out += toString(static_cast<T>(values[i]));
  }
  return out;
}

}  // namespace

bool setBlackboardFromParameter(
  BT::Blackboard & blackboard, const std::string & key, const rclcpp::Parameter & param)
{
  using rclcpp::ParameterType;
  switch (param.get_type()) {
    case ParameterType::PARAMETER_BOOL:
      blackboard.set(key, param.as_bool());
      return true;
    case ParameterType::PARAMETER_DOUBLE:
      blackboard.set(key, param.as_double());
      return true;
    case ParameterType::PARAMETER_STRING:
      blackboard.set(key, param.as_string());
      return true;
    case ParameterType::PARAMETER_INTEGER:
      blackboard.set(key, toString(param.as_int()));
      return true;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      blackboard.set(key, join(param.as_bool_array()));
      return true;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      blackboard.set(key, join(param.as_integer_array()));
      return true;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      blackboard.set(key, join(param.as_double_array()));
      return true;
    case ParameterType::PARAMETER_STRING_ARRAY:
      blackboard.set(key, join(param.as_string_array()));
      return true;
    default:
      return false;
  }
}

std::vector<std::string> loadBlackboardParameters(
  const std::vector<rclcpp::Parameter> & parameters, BT::Blackboard & blackboard,
  const rclcpp::Logger & logger, const std::string & prefix)
{
  const std::string dotted_prefix = prefix + ".";

  std::vector<std::string> keys;
  for (const auto & param : parameters) {
    if (param.get_name().rfind(dotted_prefix, 0) != 0) continue;
    const auto key = param.get_name().substr(dotted_prefix.size());
    if (setBlackboardFromParameter(blackboard, key, param)) {
      keys.push_back(key);
    } else {
      RCLCPP_WARN(logger, "Parameter '%s' has unsupported type '%s', not put in "
        "the blackboard.", param.get_name().c_str(), param.get_type_name().c_str());
    }
  }
  return keys;
}

}  // namespace mission_behavior_tree
