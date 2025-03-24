#ifndef WAYWISE_TRUCK_HPP_
#define WAYWISE_TRUCK_HPP_

#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"

#ifdef WAYWISE_HW_INTERFACE_
#include <map>
#include "WayWise/sensors/angle/as5600updater.h"
#include "WayWise/sensors/tof/vl53l0xtofsensor.h"
#endif

#include "waywise_car.hpp"

class WayWiseTruck : public WayWiseCar
{
  Q_OBJECT

public:
  WayWiseTruck(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywise_truck_node")
  : WayWiseCar(options, node_name) {}

  virtual ~WayWiseTruck() = default;

protected:
  virtual void setup_parameters() override;
  virtual void setup_publishers() override;
  virtual void setup_subscribers() override;
  virtual void setup_timers() override;
  virtual void setup_autopilot() override;
  #ifdef WAYWISE_HW_INTERFACE_
  virtual void setup_hardware() override;
  #endif

  // Callback methods
  #ifdef WAYWISE_HW_INTERFACE_
  void updated_tof_distance_callback(const std::string & tof_sensor_name, double distance_m);
  #else
  void angle_sensor_callback(const std_msgs::msg::Float32::SharedPtr angle_msg);
  #endif

  // Utility methods
  virtual void update_world_positon(geometry_msgs::msg::Pose world_pose) override;
  virtual double update_joint_states_msg(
    sensor_msgs::msg::JointState & joint_state_msg,
    double timePassed_ms) override;
  #ifdef WAYWISE_HW_INTERFACE_
  virtual void publish_odom_and_tfs() override;
  void publish_trailer_angle();
  #endif

  // ROS parameters
  float purepursuit_forward_gain_, purepursuit_reverse_gain_;

  std::string hitch_frame_;
  bool has_trailer_;
  float trailer_length_, trailer_width_, trailer_wheelbase_;
  std::string angle_sensor_topic_;

  std::string trailer_base_frame_;
  std::string trailer_rear_axle_frame_;
  std::string trailer_center_frame_;
  std::string trailer_rear_end_frame_;
  std::string trailer_hitch_frame_;

  std::vector<double> rear_axle_frame_to_hitch_frame_offset_;
  std::vector<double> trailer_rear_axle_frame_to_trailer_base_frame_offset_;
  std::vector<double> trailer_rear_axle_frame_to_trailer_center_frame_offset_;
  std::vector<double> trailer_rear_axle_frame_to_trailer_rear_end_frame_offset_;
  std::vector<double> trailer_rear_axle_frame_to_trailer_hitch_frame_offset_;

  std::vector<std::string> trailer_wheel_joint_names_;
  std::string truck_trailer_link_joint_name_;
  bool invert_trailer_joint_state_;

  std::string trailer_pose_topic_;

  #ifdef WAYWISE_HW_INTERFACE_
  float angle_sensor_offset_;
  std::vector<std::string> tof_sensor_names;
  #endif

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trailer_pose_pub_;
  #ifdef WAYWISE_HW_INTERFACE_
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  #endif

  // Subscribers
  #ifndef WAYWISE_HW_INTERFACE_
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  #endif

  // WayWise components
  QSharedPointer<TruckState> mTruckState;
  QSharedPointer<TrailerState> mTrailerState;
  #ifdef WAYWISE_HW_INTERFACE_
  QSharedPointer<AngleSensorUpdater> mAngleSensorUpdater;
  QSharedPointer<ToFSensor> mToFSensor;
  #endif

  // Internal variables
  int mTrailerMavlinkComponentID = -1;
  #ifdef WAYWISE_HW_INTERFACE_
  struct ToFSensorInfo
  {
    int i2c_addr;
    std::string topic_name;
    QSharedPointer<ToFSensor> sensor;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
  };
  std::map<std::string, ToFSensorInfo> tof_sensors_;
  #endif

// private:
};

#endif  // WAYWISE_TRUCK_HPP_
