#ifndef WAYWISE_TRUCK_AUTOPILOT_HPP_
#define WAYWISE_TRUCK_AUTOPILOT_HPP_

#include "waywise_car_autopilot.hpp"
#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"

#include "std_msgs/msg/float32.hpp"

class WaywiseTruckAutopilot : public WaywiseCarAutopilot
{
  Q_OBJECT

public:
  WaywiseTruckAutopilot(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "waywise_truck_autopilot_node")
  : WaywiseCarAutopilot(options, node_name) {}

  virtual ~WaywiseTruckAutopilot() = default;

protected:
  virtual void setup_parameters() override;
  virtual void setup_publishers() override;
  virtual void setup_subscribers() override;
  virtual void setup_timers() override;
  virtual void setup_autopilot() override;

  // Callback methods
  void angle_sensor_callback(const std_msgs::msg::Float32::SharedPtr angle_msg);

  // Utility methods
  virtual double update_joint_states_msg(
    sensor_msgs::msg::JointState & joint_state_msg,
    double timePassed_ms) override;

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

  std::vector<std::string> trailer_wheel_joint_names_;
  std::string truck_trailer_link_joint_name_;
  bool invert_trailer_joint_state_;

  // Publishers

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sensor_sub_;

  // Timers

  // WayWise components
  QSharedPointer<TruckState> mTruckState;
  QSharedPointer<TrailerState> mTrailerState;

  // Internal variables
  int mTrailerMavlinkComponentID = -1;
};

#endif  // WAYWISE_TRUCK_AUTOPILOT_HPP_
