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


struct vector3_t
{
  double x;
  double y;
  double z;

  template<typename T>
  T to_type() const
  {
    return {x, y, z};
  }

  // Vector addition operator overload
  vector3_t operator+(const vector3_t & other) const
  {
    return {x + other.x, y + other.y, z + other.z};
  }

  // Scalar addition operator overload
  vector3_t operator+(const double scalar) const
  {
    return {x + scalar, y + scalar, z + scalar};
  }

  // Friend function for scalar addition from the left
  friend vector3_t operator+(const double scalar, const vector3_t & vec)
  {
    return vec + scalar;
  }

  // Vector subtraction operator overload
  vector3_t operator-(const vector3_t & other) const
  {
    return {x - other.x, y - other.y, z - other.z};
  }

  // Scalar subtraction operator overload
  vector3_t operator-(const double scalar) const
  {
    return {x - scalar, y - scalar, z - scalar};
  }

  // Friend function for scalar subtraction from the left
  friend vector3_t operator-(const double scalar, const vector3_t & vec)
  {
    return vector3_t{scalar, scalar, scalar} - vec;
  }

  // Unary minus operator overload
  vector3_t operator-() const
  {
    return {-x, -y, -z};
  }

  std::string to_string() const
  {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "[%.4f, %.4f, %.4f]", x, y, z);
    return std::string(buf);
  }

  const char * c_str() const
  {
    static thread_local std::string buffer;
    buffer = to_string();
    return buffer.c_str();
  }
};

inline std::optional<vector3_t> get_vector3_param(
  rclcpp::Node * node, const std::string & param_name,
  const std::vector<double> & default_value = {0.0})
{
  auto vec = node->declare_parameter(param_name, default_value);
  if (vec.size() != 3) {
    return std::nullopt;
  }
  return vector3_t{vec[0], vec[1], vec[2]};
}
#endif  // WAYWISER_UTILS_HPP_
