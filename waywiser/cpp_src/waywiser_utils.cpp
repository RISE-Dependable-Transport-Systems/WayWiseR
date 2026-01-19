#include <cmath>
#include <cstdio>

#include "waywiser/waywiser_utils.hpp"

namespace QOS_PROFILES
{
// Reliable + Transient Local QoS (e.g., for latched topics)
const rclcpp::QoS RELIABLE_TRANSIENT_LOCAL_QOS = rclcpp::QoS(
  rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)
)
  .reliable()
  .transient_local()
  .keep_last(1);

// Reliable + Volatile QoS (e.g., normal pub/sub)
const rclcpp::QoS RELIABLE_VOLATILE_QOS = rclcpp::QoS(
  rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)
)
  .reliable()
  .durability_volatile()
  .keep_last(1);
}

std::string vector3_t::to_string() const
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), "[%.4f, %.4f, %.4f]", x, y, z);
  return std::string(buf);
}

const char * vector3_t::c_str() const
{
  static thread_local std::string buffer;
  buffer = to_string();
  return buffer.c_str();
}

std::optional<vector3_t> RosUtils::get_vector3_param(
  rclcpp::Node * node, const std::string & param_name,
  const std::vector<double> & default_value)
{
  auto vec = node->declare_parameter(param_name, default_value);
  if (vec.size() != 3) {
    return std::nullopt;
  }
  return vector3_t{vec[0], vec[1], vec[2]};
}

std::string RosUtils::joinFrame(
  const std::string & frame_prefix,
  const std::string & frame_name)
{
  if (frame_name.empty()) {
    return frame_name;
  }

  std::string name = frame_name;
  std::string prefix = frame_prefix;

  // Remove leading '/' from frame name
  while (!name.empty() && name.front() == '/') {
    name.erase(0, 1);
  }

  // Remove leading and trailing '/' from prefix
  while (!prefix.empty() && prefix.front() == '/') {
    prefix.erase(0, 1);
  }
  while (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }

  if (prefix.empty()) {
    return name;
  }

  return prefix + "/" + name;
}
