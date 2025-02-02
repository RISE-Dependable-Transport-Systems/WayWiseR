#ifndef WAYWISE_CAR_HPP_
#define WAYWISE_CAR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QObject>

#include "WayWise/core/simplewatchdog.h"
#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/logger/logger.h"
#include "WayWise/sensors/fusion/sdvpvehiclepositionfuser.h"
#include "WayWise/sensors/gnss/rtcmclient.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/sensors/imu/bno055orientationupdater.h"
#include "WayWise/sensors/imu/imuorientationupdater.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

class WayWiseCar : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  WayWiseCar(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywise_car_node")
  : QObject(), Node(node_name, options) {}

  virtual ~WayWiseCar() = default;

  virtual void initialize_node();

protected:
  virtual void setup_parameters();
  virtual void setup_publishers();
  virtual void setup_subscribers();
  virtual void setup_timers();
  virtual void setup_hardware();
  void setup_hardware(QSharedPointer<CarState> carState);

  // Callback methods
  virtual void simulation_timer_callback();
  virtual void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  virtual void updated_waywise_odomPos_callback(
    QSharedPointer<VehicleState> vehicleState,
    double distanceDriven);

  // Utility methods
  virtual void publish_odom_and_tfs(double timePassed_ms);
  void publish_ublox_nav_sat_fix(const ubx_nav_pvt & ubxPvt);
  void publish_enu_refernce(const llh_t enuRef);
  float clip_min_max(float value, float min_value, float max_value) const;

  // ROS parameters
  std::string odom_topic_;
  float erpm_min_, erpm_max_, speed_to_erpm_factor_;
  bool invert_servo_output_;
  float servo_min_, servo_max_, servo_offset_;
  float length_, width_, wheelbase_, min_turning_radius_;
  bool publish_odom_to_baselink_tf_;
  bool publish_world_to_odom_tf_;
  int odom_and_tf_publish_rate_;
  bool update_world_position_with_odom_;
  bool enable_imu_for_odom_;
  std::string imu_for_position_fusion_;
  float standstill_velocity_threshold_;
  float max_angular_velocity_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string world_frame_;

  std::string nav_sat_fix_topic_;
  std::string enu_refernce_topic_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr enu_refernce_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<VESCMotorController> mVESCMotorController;
  QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;
  QSharedPointer<UbloxRover> mUbloxRover;
  SDVPVehiclePositionFuser * positionFuser;
  RtcmClient * rtcmClient;
  SimpleWatchdog * watchdog;

  // Internal variables
  std::chrono::milliseconds mUpdateVehicleStatePeriod;

// private:
};

#endif  // WAYWISE_CAR_HPP_
