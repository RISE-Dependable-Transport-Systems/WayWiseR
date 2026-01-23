#ifndef WAYWISER_UTILS_HPP_
#define WAYWISER_UTILS_HPP_

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double DEG2RAD = M_PI / 180.0;

namespace QOS_PROFILES
{

// Reliable + Transient Local QoS (e.g., for latched topics)
extern const rclcpp::QoS RELIABLE_TRANSIENT_LOCAL_QOS;

// Reliable + Volatile QoS (e.g., normal pub/sub)
extern const rclcpp::QoS RELIABLE_VOLATILE_QOS;

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

  std::string to_string() const;

  const char * c_str() const;
};

class RosUtils
{
public:
  static std::optional<vector3_t> get_vector3_param(
    rclcpp::Node * node, const std::string & param_name,
    const std::vector<double> & default_value = {0.0, 0.0, 0.0});

  static std::string joinFrame(
    const std::string & frame_prefix,
    const std::string & frame_name);
};
#endif  // WAYWISER_UTILS_HPP_
