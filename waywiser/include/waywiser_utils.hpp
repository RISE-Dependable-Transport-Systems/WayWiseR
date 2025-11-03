#ifndef WAYWISER_UTILS_HPP_
#define WAYWISER_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

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

}  // namespace QOS_PROFILES

#endif  // WAYWISER_UTILS_HPP_
