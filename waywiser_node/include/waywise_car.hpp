#ifndef WAYWISE_CAR_HPP_
#define WAYWISE_CAR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QObject>

#include "WayWise/logger/logger.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/sensors/imu/imuorientationupdater.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
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
  void updated_waywise_imuPos_callback(QSharedPointer<VehicleState> vehicleState);


  // Utility methods
  virtual void publish_odom_and_tf(double timePassed_ms);
  float clip_min_max(float value, float min_value, float max_value) const;

  // ROS parameters
  std::string odom_topic_;
  float erpm_min_, erpm_max_, speed_to_erpm_factor_;
  bool invert_servo_output_;
  float servo_min_, servo_max_, servo_offset_;
  float length_, width_, wheelbase_, min_turning_radius_;
  bool publish_odom_to_baselink_tf_;
  int odom_publish_rate_;
  bool enable_imu_for_odom_;
  float standstill_velocity_threshold_;
  float max_angular_velocity_;

  std::string odom_frame_;
  std::string base_frame_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

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

  // Internal variables
  std::chrono::milliseconds mUpdateVehicleStatePeriod;
  PosType waywise_posType_used_ = PosType::simulated;
  double mPosIMUyawOffset = 0.0;

// private:
};

#endif  // WAYWISE_CAR_HPP_
