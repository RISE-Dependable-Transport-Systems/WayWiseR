#ifndef WAYWISE_CAR_AUTOPILOT_HPP_
#define WAYWISE_CAR_AUTOPILOT_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QObject>

#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/logger/logger.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/autopilot/followpoint.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mavsdk/mavsdk.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class WaywiseCarAutopilot : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WaywiseCarAutopilot(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywise_car_autopilot_node")
  : QObject(), Node(node_name, options) {}

  virtual ~WaywiseCarAutopilot() = default;

  virtual void initialize_node();

protected:
  virtual void setup_parameters();
  virtual void setup_publishers();
  virtual void setup_subscribers();
  virtual void setup_timers();
  virtual void setup_autopilot();
  void setup_autopilot(QSharedPointer<CarState> carState);
  virtual void provide_parameters_to_parameter_server();

  // Callback methods
  void autopilot_timer_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // Utility methods

  // ROS parameters
  std::string odom_topic_;
  float speed_to_erpm_factor_;
  float length_, width_, wheelbase_, min_turning_radius_;
  int autopilot_cmd_publish_rate_;
  std::string waywise_control_tower_address_;
  float purepursuit_radius_;

  float standstill_velocity_threshold_;
  float max_angular_velocity_;

  std::string odom_frame_;
  std::string base_frame_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr autopilot_timer_;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;
  QSharedPointer<FollowPoint> mFollowPoint;

  // Internal variables
  PosType waywise_posType_used_ = PosType::simulated;
};

#endif  // WAYWISE_CAR_AUTOPILOT_HPP_
